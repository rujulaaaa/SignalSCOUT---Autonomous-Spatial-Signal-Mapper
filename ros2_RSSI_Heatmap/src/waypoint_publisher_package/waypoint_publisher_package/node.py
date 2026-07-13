# ROS
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
import tf2_ros
from tf2_ros import TransformException

# Utils
import os
import tkinter  # noqa: F401
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt  # noqa: E402
import cv2
import yaml
import numpy as np
from collections import namedtuple
from multiprocessing import Process


def show_map(map_img):
    """Display map of waypoints in a separate process."""
    plt.imshow(map_img)
    plt.axis('off')
    plt.title('Waypoints (green=valid, red=too close to obstacle)')
    plt.show()


Waypoint = namedtuple('Waypoint', 'x y')


class FollowWaypointsClient(Node):
    def __init__(self):
        super().__init__('navigate_through_poses_client')
        self._action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.publisher = self.create_publisher(Empty, '/heatmap_generator_trigger', 1)

        self.declare_parameter('density', 8)
        self.declare_parameter('collision_range', 4)
        self.declare_parameter('path_to_yaml', 'map.yaml')
        self.declare_parameter('batch_size', 20)
        self.declare_parameter('nearest_first', True)

        self.density = self.get_parameter('density').get_parameter_value().integer_value
        self.collision_range = self.get_parameter('collision_range').get_parameter_value().integer_value
        self.batch_size = self.get_parameter('batch_size').get_parameter_value().integer_value
        self.nearest_first = self.get_parameter('nearest_first').get_parameter_value().bool_value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        yaml_path = self.get_parameter('path_to_yaml').get_parameter_value().string_value
        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)

        self.origin = Waypoint(data['origin'][0], data['origin'][1])
        self.resolution = data['resolution']

        image_path = data['image']
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)

        self.map = cv2.imread(image_path)
        if self.map is None:
            self.get_logger().error(f'Failed to load map image: {image_path}')
            raise FileNotFoundError(f'Map image not found: {image_path}')

        self.robot_frame_waypoint_array = []
        self.remaining_waypoints = []
        self.batch_index = 0
        self.p = Process(target=show_map, args=(self.map,))

    def _get_robot_position(self):
        """Look up the robot's current (x, y) in the map frame via TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time())
            return (transform.transform.translation.x,
                    transform.transform.translation.y)
        except TransformException as ex:
            self.get_logger().warn(f'Could not look up robot pose: {ex}')
            return None

    def _pop_nearest_batch(self, current_pos):
        """Greedily pop up to batch_size waypoints, always choosing the
        nearest unvisited one to the (updated) current position."""
        batch = []
        position = current_pos
        while self.remaining_waypoints and len(batch) < self.batch_size:
            nearest = min(
                self.remaining_waypoints,
                key=lambda wp: (wp.x - position[0]) ** 2 + (wp.y - position[1]) ** 2)
            self.remaining_waypoints.remove(nearest)
            batch.append(nearest)
            position = (nearest.x, nearest.y)
        return batch

    def send_goal(self):
        """Generate waypoints and send first batch."""
        self._generate_waypoints()
        self.remaining_waypoints = list(self.robot_frame_waypoint_array)

        total = len(self.robot_frame_waypoint_array)
        mode = 'nearest-first' if self.nearest_first else 'in generation order'
        self.get_logger().info(
            f'Generated {total} waypoints — sending in batches of '
            f'{self.batch_size} ({mode})')

        self._send_next_batch()

    def _generate_waypoints(self):
        """Generate waypoints from the occupancy map."""

        gray = cv2.cvtColor(self.map, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape

        # Free space is bright (254), obstacles are dark (0), unknown is mid-gray (205).
        # Treat everything that is not clearly free space as occupied.
        obstacle_mask = (gray < 250).astype(np.uint8)

        kernel_size = 2 * self.collision_range + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        inflated_obstacle_mask = cv2.dilate(obstacle_mask, kernel)

        for y in range(0, height, self.density):
            for x in range(0, width, self.density):
                if inflated_obstacle_mask[y, x] == 0:
                    self.map[y, x] = (0, 255, 0)
                    world_x = self.origin.x + x * self.resolution
                    world_y = self.origin.y + (height - y) * self.resolution
                    self.robot_frame_waypoint_array.append(Waypoint(world_x, world_y))
                else:
                    self.map[y, x] = (255, 0, 0)

        self.p.start()

        self.get_logger().info(
            f'{len(self.robot_frame_waypoint_array)} valid waypoints generated')

    def _send_next_batch(self):
        """Send the next batch of waypoints to Nav2."""

        if self.nearest_first:
            current_pos = self._get_robot_position() or self.origin
            batch = self._pop_nearest_batch(current_pos)
        else:
            start = self.batch_index * self.batch_size
            end = min(start + self.batch_size,
                      len(self.robot_frame_waypoint_array))
            batch = self.robot_frame_waypoint_array[start:end]

        if not batch:
            self.get_logger().info(
                'All batches complete — publishing trigger...')
            self.publisher.publish(Empty())

            if self.p.is_alive():
                self.p.kill()

            rclpy.shutdown()
            return

        remaining_after = len(self.remaining_waypoints) if self.nearest_first else \
            len(self.robot_frame_waypoint_array) - end
        self.get_logger().info(
            f'Sending batch {self.batch_index + 1} '
            f'({len(batch)} waypoints, {remaining_after} remaining) '
            f'of {len(self.robot_frame_waypoint_array)} total')

        msg = FollowWaypoints.Goal()

        poses = []
        for waypoint in batch:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = waypoint.x
            pose.pose.position.y = waypoint.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            poses.append(pose)
        msg.poses = poses

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(msg)
        self._send_goal_future.add_done_callback(self._response_callback)

    def _response_callback(self, future):
        goal_handle = future.result()

        self.get_logger().info(
            f'Batch {self.batch_index + 1} accepted by server')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        self.get_logger().info(
            f'Batch {self.batch_index + 1} complete')

        self.batch_index += 1
        self._send_next_batch()


def main(args=None):
    rclpy.init(args=args)

    action_client = FollowWaypointsClient()
    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
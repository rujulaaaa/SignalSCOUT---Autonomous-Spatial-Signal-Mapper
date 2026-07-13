# ROS
import rclpy
from rclpy.node import Node
from rosbot_interfaces.msg import RssiAtWaypoint
from std_msgs.msg import Empty
import os
from collections import namedtuple
import yaml
import cv2
from .submodules.generate_heatmap import generate_heatmap, add_heatmap, add_waypoints, cmapGR
import tkinter  # noqa: F401
from multiprocessing import Process
import datetime
from matplotlib import gridspec, pyplot as plt
import matplotlib
matplotlib.use('TkAgg')

RssiWaypoint = namedtuple('RssiWaypoint', 'x y rssi')
Waypoint = namedtuple('Waypoint', 'x y')

class HeatmapGenerator(Node):
    def __init__(self):
        super().__init__('heatmap_generator')
        self.create_subscription(RssiAtWaypoint,'/rssi_data',self.rssi_data_callback,10)
        self.create_subscription(Empty,'/heatmap_generator_trigger',self.trigger_callback,10)

        self.declare_parameter('path_to_yaml','map.yaml')
        self.declare_parameter('heatmaps_dir',os.path.expanduser('~/ros2_RSSI_Heatmap/src/heatmaps'))

        yaml_path=self.get_parameter('path_to_yaml').get_parameter_value().string_value
        with open(yaml_path,'r') as file:
            map_data=yaml.safe_load(file)

        self.map_origin=Waypoint(map_data['origin'][0],map_data['origin'][1])
        self.map_resolution=map_data['resolution']

        image_path=map_data['image']
        if not os.path.isabs(image_path):
            image_path=os.path.join(os.path.dirname(yaml_path),image_path)

        self.map=cv2.imread(image_path)
        if self.map is None:
            raise FileNotFoundError(image_path)

        self.rssi_data=[]
        self.serving_aps=[]
        self.heatmaps_dir=self.get_parameter('heatmaps_dir').get_parameter_value().string_value
        os.makedirs(self.heatmaps_dir,exist_ok=True)

    def rssi_data_callback(self,msg):

        x=int((msg.coordinates.x-self.map_origin.x)/self.map_resolution)
        y=len(self.map)-int((msg.coordinates.y-self.map_origin.y)/self.map_resolution)
        self.rssi_data.append(RssiWaypoint(x,y,int(msg.rssi)))
        self.serving_aps.append(int(msg.serving_ap))

    def display_maps(self,*args):
        (map_with_waypoints,heatmap,final_map,
         rel_heatmap,rel_final_map,rssi_bounds)=args
        minrssi,maxrssi=rssi_bounds

        plt.figure('Waypoints')
        plt.imshow(map_with_waypoints)
        plt.axis('off')
        plt.title('Surveyed Waypoints')

        fig_abs=plt.figure('Absolute RSSI Heatmap')
        gs=gridspec.GridSpec(1,2,width_ratios=[20,1])
        ax=fig_abs.add_subplot(gs[0])
        cax=fig_abs.add_subplot(gs[1])
        ax.imshow(final_map)
        ax.axis('off')
        ax.set_title('Absolute RSSI Heatmap (dBm)')
        norm=matplotlib.colors.Normalize(vmin=-100,vmax=0)
        fig_abs.colorbar(matplotlib.cm.ScalarMappable(norm=norm,cmap=cmapGR),cax=cax)

        fig_rel=plt.figure('Relative RSSI Heatmap')
        gs=gridspec.GridSpec(1,2,width_ratios=[20,1])
        ax=fig_rel.add_subplot(gs[0])
        cax=fig_rel.add_subplot(gs[1])
        ax.imshow(rel_final_map)
        ax.axis('off')
        ax.set_title('Relative RSSI Heatmap')
        norm=matplotlib.colors.Normalize(vmin=minrssi,vmax=maxrssi)
        fig_rel.colorbar(matplotlib.cm.ScalarMappable(norm=norm,cmap=cmapGR),cax=cax)

        plt.show()

    def trigger_callback(self,msg):
        if len(self.rssi_data)<3:
            return

        rssi_values=[wp.rssi for wp in self.rssi_data]
        ap_counts={}
        for ap in self.serving_aps:
            ap_counts[ap]=ap_counts.get(ap,0)+1
        self.get_logger().info(
            f'Survey stats — {len(self.rssi_data)} waypoints, '
            f'avg RSSI={sum(rssi_values)/len(rssi_values):.1f} dBm, '
            f'coverage by AP: {ap_counts}')

        map_with_waypoints=add_waypoints(self.map,self.rssi_data)

        heatmap=generate_heatmap(
            self.rssi_data,len(self.map),len(self.map[0]),1,filtered=False)[0]

        rel_heatmap,rssi_bounds=generate_heatmap(
            self.rssi_data,len(self.map),len(self.map[0]),1,
            filtered=True,relative=True)

        final_map=add_heatmap(self.map,heatmap)
        rel_final_map=add_heatmap(self.map,rel_heatmap)

        p=Process(target=self.display_maps,
                  args=(map_with_waypoints,heatmap,final_map,
                        rel_heatmap,rel_final_map,rssi_bounds))
        p.start()

        now=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        cv2.imwrite(os.path.join(self.heatmaps_dir,f'waypoints_{now}.png'),cv2.cvtColor(map_with_waypoints,cv2.COLOR_RGB2BGR))
        cv2.imwrite(os.path.join(self.heatmaps_dir,f'heatmap_abs_{now}.png'),cv2.cvtColor(final_map,cv2.COLOR_RGB2BGR))
        cv2.imwrite(os.path.join(self.heatmaps_dir,f'heatmap_rel_{now}.png'),cv2.cvtColor(rel_final_map,cv2.COLOR_RGB2BGR))

def main(args=None):
    rclpy.init(args=args)
    node=HeatmapGenerator()
    rclpy.spin(node)

if __name__=="__main__":
    main()

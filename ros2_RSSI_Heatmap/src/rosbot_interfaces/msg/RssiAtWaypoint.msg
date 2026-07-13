#Interface for measuring rssi at waypoints
#Waypoint coordinates
geometry_msgs/Point coordinates
#Rssi value (from -100 to 0), strongest signal among all configured APs
int8 rssi
#Index of the access point that served this waypoint (strongest signal)
int8 serving_ap
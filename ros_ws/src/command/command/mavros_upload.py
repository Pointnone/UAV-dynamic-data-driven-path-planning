import rclpy
from rclpy.node import Node

import mavros_msgs.msg as mv_msg
import mavros_msgs.srv as mv_srv

import math

try:
    from simple_path import *
    from utils import *
except ImportError:
    from .simple_path import *
    from .utils import *

def convert_to_mavros(waypoints):
    home = waypoints[0]
    dest = waypoints[-1]
    wps = [ mv_msg.Waypoint(command=16, x_lat=wp[1], y_long=wp[0], z_alt=50.0, is_current=False, autocontinue=True, param1=0.2, param2=10.0, param3=0.0, param4=math.nan) for i, wp in enumerate(waypoints) ]
    wps.insert(0, mv_msg.Waypoint(command=22, x_lat=home[1], y_long=home[0], z_alt=50.0, is_current=True, autocontinue=True, param1=0.0, param4=math.nan))
    wps.insert(len(wps), mv_msg.Waypoint(command=21, x_lat=dest[1], y_long=dest[0], z_alt=25.0, is_current=False, autocontinue=False, param1=25.0, param2=0.0, param4=math.nan))
    return wps

def init():
    global mv_node; mv_node = Node('mavros_upload')
    global wp_clear_client; wp_clear_client = mv_node.create_client(mv_srv.WaypointClear, "mavros/mission/clear")
    global wp_push_client; wp_push_client = mv_node.create_client(mv_srv.WaypointPush, "mavros/mission/push")

    while not wp_clear_client.wait_for_service(timeout_sec=5.0):
        mv_node.get_logger().info("mission/clear service not available")
    while not wp_push_client.wait_for_service(timeout_sec=5.0):
        mv_node.get_logger().info("mission/push service not available")

def send_mission(waypoints):
    wp_clear_req = mv_srv.WaypointClear.Request()
    clear_future = wp_clear_client.call_async(wp_clear_req)
    rclpy.spin_until_future_complete(mv_node, clear_future)
    
    wp_push_req = mv_srv.WaypointPush.Request()
    wp_push_req.waypoints = convert_to_mavros(waypoints)
    wp_push_req.start_index = 0
    
    future = wp_push_client.call_async(wp_push_req)
    rclpy.spin_until_future_complete(mv_node, future)
    return future.result()

def main(args=None):
    print(args)
    rclpy.init(args=args)

    init()

    env = environmentVars()
    print(env)
    engine, meta, dbsm = setupDB(env['DB_USER'], env['DB_PASS'])

    waypoints = find_path((10.3245895, 55.4718524), (10.3145895, 55.4518524), engine, meta, dbsm)

    #waypoints = [(10.3245895, 55.4718524), (10.3045895, 55.4718524), (10.3145895, 55.4518524)]
    send_mission(waypoints)

    mv_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
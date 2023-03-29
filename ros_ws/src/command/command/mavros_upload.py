import rclpy
from rclpy.node import Node

import mavros_msgs.msg as mv_msg
import mavros_msgs.srv as mv_srv

from pyproj import Geod

def convert_to_mavros(waypoints):
    wps = [ mv_msg.Waypoint(frame=0, command=16, x_lat=wp[0], y_long=wp[1], z_alt=50.0, is_current=(i==0), autocontinue=True, param1=0.2, param2=10.0, param3=0.0, param4=0.0) for i, wp in enumerate(waypoints) ]
    #wp = mv_msg.Waypoint()
    #print(wps)
    return wps

def segment_path(waypoints, max_length = 750):
    

class minTestPub(Node):
    def __init__(self):
        super().__init__('mavros_upload')
        #self.publisher_ = self.create_publisher(mv_msg.Waypoint, "mavros/WaypointPush", 10)
        self.wp_push_client = self.create_client(mv_srv.WaypointPush, "mavros/mission/push")
        self.wp_clear_client = self.create_client(mv_srv.WaypointClear, "mavros/mission/clear")
        #self.timer = self.create_timer(1.0, self.t_cb)
        while not self.wp_clear_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("mission/clear service not available")
        while not self.wp_push_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("mission/push service not available")
        #self.req = mv_srv.WaypointPush.Request()
        #print(self.req)
    
    #def t_cb(self):
    #    msg = mv_msg.Waypoint()
    #    print(msg)

    def send_mission(self, waypoints):
        wp_clear_req = mv_srv.WaypointClear.Request()
        clear_future = self.wp_clear_client.call_async(wp_clear_req)
        rclpy.spin_until_future_complete(self, clear_future)
        print(clear_future.result())

        wp_push_req = mv_srv.WaypointPush.Request()
        wp_push_req.waypoints = convert_to_mavros(waypoints)
        wp_push_req.start_index = 0
        print(wp_push_req)

        self.future = self.wp_push_client.call_async(wp_push_req)
        rclpy.spin_until_future_complete(self, self.future)
        print(self.future.exception())
        return self.future.result()

def main(args=None):
    #print('Hi from command.')
    rclpy.init(args=args)

    m_tp = minTestPub()
    waypoints = [(9.538999, 55.706185), (9.538999, 54.979835), (12.004298, 54.979835)]
    #convert_to_mavros(waypoints)
    print(m_tp.send_mission(waypoints))

    #rclpy.spin(m_tp)

    m_tp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

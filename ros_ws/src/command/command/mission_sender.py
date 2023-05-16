import rclpy
from rclpy.node import Node

import mavros_msgs.msg as mv_msg
import mavros_msgs.srv as mv_srv
from std_msgs.msg import String

import math
import sys

from shapely import LineString, box

try:
    from simple_path import *
    from utils import *
    from command_interfaces.srv import *
    from path_info_extract import *
    from RRT_path import *
except ImportError:
    from .simple_path import *
    from .utils import *
    from command_interfaces.srv import *
    from .path_info_extract import *
    from .RRT_path import *

home = (10.3245895, 55.4718524)
dest = (10.3145895, 55.2518524)
drone='Test1' 
req_datetime='2023-05-10 12:00:00'
algo='RRT'

linestring = "LINESTRING (10.32459 55.471852, 10.312143 55.460384, 10.296547 55.450236, 10.297777 55.436781, 10.297857 55.423308, 10.295857 55.409882, 10.294274 55.396439, 10.288941 55.383312, 10.287216 55.369875, 10.283074 55.356609, 10.287078 55.343331, 10.29108 55.330052, 10.294665 55.316734, 10.295594 55.303271, 10.300616 55.290106, 10.30141 55.27664, 10.305652 55.263385, 10.31459 55.251852)"

class MissionSender(Node):
    def __init__(self):
        super().__init__("missionsender")

        self.mv_node = Node('mission_sender')
        self.mission_req_client = self.mv_node.create_client(RequestPath, "commander/request_path")
        self.upload_client = self.mv_node.create_client(UploadLineString, "commander/upload_linestring")

        while not self.mission_req_client.wait_for_service(timeout_sec=5.0):
            self.mv_node.get_logger().info("request_path service not available")

    def send(self):
        req_path_req = RequestPath.Request()
        #print(req_path_req)
        req_path_req.home.x = home[0]; req_path_req.home.y = home[1]; req_path_req.home.z = 0.0
        req_path_req.dest.x = dest[0]; req_path_req.dest.y = dest[1]; req_path_req.dest.z = 0.0
        req_path_req.drone = drone
        req_path_req.req_datetime = req_datetime
        req_path_req.algo = algo
        print(req_path_req)

        req_path_future = self.mission_req_client.call_async(req_path_req)

        rclpy.spin_until_future_complete(self.mv_node, req_path_future)
        print(req_path_future.result())
        print("Sent it")  

    def send_linestring(self):
        ULS_req = UploadLineString.Request()
        #print(req_path_req)
        #req_path_req.home.x = home[0]; req_path_req.home.y = home[1]; req_path_req.home.z = 0.0
        #req_path_req.dest.x = dest[0]; req_path_req.dest.y = dest[1]; req_path_req.dest.z = 0.0
        #req_path_req.drone = drone
        #req_path_req.req_datetime = req_datetime
        #req_path_req.algo = algo
        ULS_req.linestring = linestring
        print(ULS_req)

        ULS_req_future = self.upload_client.call_async(ULS_req)

        rclpy.spin_until_future_complete(self.mv_node, ULS_req_future)
        print(ULS_req_future.result())
        print("Sent it")      

rclpy.init()
ms = MissionSender()
ms.send_linestring()
print("Shutting down")
ms.destroy_node()
rclpy.shutdown()
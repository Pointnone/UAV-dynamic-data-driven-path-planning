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

path_replanning_cost_margin = 0.05 # 5% percent

tables = ["df", "dmi", "cell", "zone"]

def convert_to_mavros(waypoints):
    home = waypoints[0]
    dest = waypoints[-1]
    wps = [ mv_msg.Waypoint(command=16, x_lat=wp[1], y_long=wp[0], z_alt=50.0, is_current=False, autocontinue=True, param1=0.2, param2=10.0, param3=0.0, param4=math.nan) for i, wp in enumerate(waypoints) ]
    wps.insert(0, mv_msg.Waypoint(command=22, x_lat=home[1], y_long=home[0], z_alt=50.0, is_current=True, autocontinue=True, param1=0.0, param4=math.nan))
    wps.insert(len(wps), mv_msg.Waypoint(command=21, x_lat=dest[1], y_long=dest[0], z_alt=25.0, is_current=False, autocontinue=False, param1=25.0, param2=0.0, param4=math.nan))
    return wps

def linestring_to_coords(ls):
    x, y = ls.xy
    coords = [(x, y) for x, y in zip(x,y)]
    return coords

class PathPlanner(Node):
    def __init__(self):
        super().__init__("pathplanner")

        env = environmentVars()
        self.engine, self.meta, self.dbsm = setupDB(env['DB_USER'], env['DB_PASS'])
        self.db =  DB_Handler()

        self.mv_node = Node('mavros_upload')
        self.wp_clear_client = self.mv_node.create_client(mv_srv.WaypointClear, "mavros/mission/clear")
        self.wp_push_client = self.mv_node.create_client(mv_srv.WaypointPush, "mavros/mission/push")
        #self.wp_pull_client = self.mv_node.create_client(mv_srv.WaypointPull, "mavros/mission/pull") # TODO: Use to get current mission and crop mission planning to the new items

        while not self.wp_clear_client.wait_for_service(timeout_sec=5.0):
            self.mv_node.get_logger().info("mission/clear service not available")
        while not self.wp_push_client.wait_for_service(timeout_sec=5.0):
            self.mv_node.get_logger().info("mission/push service not available")

        self.ingest_sub = self.create_subscription(String, 'ingest/update', self.handle_ingest_update, 10)
        self.mv_node.create_service(RequestPath, 'request_path', self.handle_mission_path_req)

    def handle_ingest_update(self, msg):
        db_handle = DB_Handler()
        drones = db_handle.extract_drone_paths()
        print(f"Got drones: {drones}")

        for d in drones:
            print(f"Checking drone: {d['drone']}")
            path_coords = linestring_to_coords(d['path'])
            home = path_coords[0]
            dest = path_coords[-1]
            start_time = d['start_time']
            bounds = list(LineString([home, dest]).bounds)
            radius = 0.25
            bounds[0] = bounds[0] - radius
            bounds[1] = bounds[1] - radius
            bounds[2] = bounds[2] + radius
            bounds[3] = bounds[3] + radius
            bbox = box(bounds[0], bounds[1], bounds[2], bounds[3])
            print(f"Extracting data")
            path_dat = extract_db_data(tables, self.engine, self.meta, self.dbsm, None, 0.25, bbox)

            print("Re-analysing cost")
            ca = Cost_Analyser(path_dat, [])
            new_cost, _ = ca.analyse_cost(path_coords, start_time)

            if(d['cost'] * (1-path_replanning_cost_margin) > new_cost or d['cost'] * (1+path_replanning_cost_margin) < new_cost):
                # Replan
                print("Doing replan")
                point_path, new_path_cost = find_path(home, dest, path_dat=path_dat, start_time=start_time)
                new_wps = linestring_to_coords(LineString(point_path))
                #new_path_cost, new_cost_per_waypoint = ca.analyse_cost(d['path'], start_time)
                print(f"Found new path with cost: {new_path_cost}, old path cost: {new_cost}, original cost: {d['cost']}")
                print(f"New path: {LineString(new_wps)}\n\nOld path: {d['path']}")

                print("Uploading new mission")
                self.send_mission(new_wps)

                print("Update DB with new mission data")
                d['cost'] = new_path_cost
                d['path'] = LineString(new_wps)
                db_handle.insert_into_table("drone_paths", [d])
            else:
                print("Not doing replan, cost within margin")
                print(f"Found new cost: {new_cost}, original cost: {d['cost']}")

    def handle_mission_path_req(self, req, res):
        home = (req.home[0], req.home[1])
        dest = (req.dest[0], req.dest[1])
        point_path, cost = find_path(home, dest, engine, meta, dbsm)
        waypoints = linestring_to_coords(LineString(point_path))

        drone_path_record = {"drone": req.drone, "path": LineString(waypoints), "cost": cost, "start_time": req.req_datetime}
        self.db.insert_into_table("drone_paths", [drone_path_record])

        self.send_mission(waypoints)
        return res

    def send_mission(self, waypoints):
        wp_clear_req = mv_srv.WaypointClear.Request()
        clear_future = self.wp_clear_client.call_async(wp_clear_req)
        rclpy.spin_until_future_complete(self.mv_node, clear_future)
        
        wp_push_req = mv_srv.WaypointPush.Request()
        wp_push_req.waypoints = convert_to_mavros(waypoints)
        wp_push_req.start_index = 0
        
        future = self.wp_push_client.call_async(wp_push_req)
        rclpy.spin_until_future_complete(self.mv_node, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)

    pathplanner = PathPlanner()

    rclpy.spin(pathplanner)
    #waypoints = find_path((10.3245895, 55.4718524), (10.3145895, 55.4518524), engine, meta, dbsm)

    #waypoints = [(10.3245895, 55.4718524), (10.3045895, 55.4718524), (10.3145895, 55.4518524)]
    #send_mission(waypoints)

    pathplanner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
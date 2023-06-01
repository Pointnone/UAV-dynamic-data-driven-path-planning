import rclpy
from rclpy.node import Node

import mavros_msgs.msg as mv_msg
import mavros_msgs.srv as mv_srv
from std_msgs.msg import String

import math
import sys
import time

from shapely import LineString, box, to_wkt, from_wkt

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
        self.db = DB_Handler()

        self.mv_node = Node('mavros_upload')
        self.wp_clear_client = self.mv_node.create_client(mv_srv.WaypointClear, "mavros/mission/clear")
        self.wp_push_client = self.mv_node.create_client(mv_srv.WaypointPush, "mavros/mission/push")
        self.wp_pull_client = self.mv_node.create_client(mv_srv.WaypointPull, "mavros/mission/pull")

        while not self.wp_clear_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("mission/clear service not available")
        while not self.wp_push_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("mission/push service not available")
        while not self.wp_pull_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("mission/pull service not available")

        self.ingest_sub = self.create_subscription(String, 'ingest/update', self.handle_ingest_update, 10)

        self.waypoints_sub = self.mv_node.create_subscription(mv_msg.WaypointList, 'mavros/mission/waypoints', self.handle_waypoints_update, 10)
        self.wp_update = False
        self.wp_result = None

        self.req_path_srv = self.create_service(RequestPath, 'commander/request_path', self.handle_mission_path_req)
        self.upload_ls_srv = self.create_service(UploadLineString, 'commander/upload_linestring', self.handle_upload_linestring)

    def handle_upload_linestring(self, req, res):
        coords = linestring_to_coords(from_wkt(req.linestring))
        items = convert_to_mavros(coords)
        print(items)
        self.send_mission(items, False)
        return res

    def handle_waypoints_update(self, msg):
        self.wp_update = True
        self.wp_result = msg

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
                active = start_time < datetime.now()
                if(not active):
                    point_path, new_path_cost = find_path(home, dest, path_dat=path_dat, start_time=start_time)
                else:
                    # Pull path
                    wp_pull_req = mv_srv.WaypointPull.Request()
                    self.wp_update = False

                    while(not self.wp_update):
                        print("Getting state")
                        pull_future = self.wp_pull_client.call_async(wp_pull_req)
                        rclpy.spin_until_future_complete(self.mv_node, pull_future)
                        time.sleep(5)

                    # Stop the drone at the next waypoint
                    print("Pausing at next waypoint")
                    wps = self.wp_result.waypoints
                    wps[self.wp_result.current_seq].autocontinue = False
                    self.send_mission(wps, False)

                    home = (wps[self.wp_result.current_seq].y_long, wps[self.wp_result.current_seq].x_lat) # Start from stopped at waypoint
                    point_path, new_path_cost = find_path(home, dest, path_dat=path_dat, start_time=start_time)

                new_wps = linestring_to_coords(LineString(point_path))
                #new_path_cost, new_cost_per_waypoint = ca.analyse_cost(d['path'], start_time)
                print(f"Found new path with cost: {new_path_cost}, old path cost: {new_cost}, original cost: {d['cost']}")
                print(f"New path: {LineString(new_wps)}\n\nOld path: {d['path']}")

                print("Uploading new mission")
                mavros_mission = convert_to_mavros(new_wps)
                if(active):
                    mavros_mission[0].is_current = False # Disable take-off
                    mavros_mission[1].is_current = True # Set first waypoint, which is the current to is_current
                    self.send_mission(mavros_mission, False)
                else:
                    self.send_mission(mavros_mission)

                print("Update DB with new mission data")
                d['cost'] = new_path_cost
                d['path'] = to_wkt(LineString(new_wps))
                db_handle.insert_drone_into_drone_paths([d])
            else:
                print("Not doing replan, cost within margin")
                print(f"Found new cost: {new_cost}, original cost: {d['cost']}")

    def handle_mission_path_req(self, req, res):
        print("Got a path req")
        print(req)
        home = (req.home.x, req.home.y)
        dest = (req.dest.x, req.dest.y)
        print("Pathfinding")
        point_path, cost = find_path(home, dest, self.engine, self.meta, self.dbsm)
        waypoints = linestring_to_coords(LineString(point_path))

        print("Putting in DB")
        drone_path_record = {"drone": req.drone, "path": to_wkt(LineString(waypoints)), "cost": cost, "start_time": req.req_datetime}
        self.db.insert_drone_into_drone_paths([drone_path_record])

        print("Uploading to drone")
        mavros_mission = convert_to_mavros(waypoints)
        self.send_mission(mavros_mission)
        print("Sent mission")
        res.success = True # Set success to true. TODO: should be dependent on the result of pathfinding.
        return res

    def send_mission(self, mission_items, clear=True):
        print("Got into send_mission")
        if(clear):
            wp_clear_req = mv_srv.WaypointClear.Request()
            clear_future = self.wp_clear_client.call_async(wp_clear_req)
            rclpy.spin_until_future_complete(self.mv_node, clear_future)
            print("Sent clear")
        
        wp_push_req = mv_srv.WaypointPush.Request()
        wp_push_req.waypoints = mission_items
        wp_push_req.start_index = 0
        
        future = self.wp_push_client.call_async(wp_push_req)
        print("Wait spinning")
        rclpy.spin_until_future_complete(self.mv_node, future)
        print("Done spinning")
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
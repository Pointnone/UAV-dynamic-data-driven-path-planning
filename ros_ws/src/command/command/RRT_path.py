try:
    from utils import *
    from path_info_extract import *
    from path_sim import *
    from simple_path import Cost_Analyser
except ImportError:
    from .utils import *
    from .path_info_extract import *
    from .path_sim import *
    from .simple_path import Cost_Analyser

from shapely import Point, LineString, STRtree, distance, prepare, contains, intersects, envelope, buffer, box, to_geojson
import pyproj as proj
from datetime import datetime
from functools import reduce
import numpy as np
import time

import random as rnd

tables = ["df", "cell", "zone"]
g = Geod(ellps="WGS84")

# Implementation cross between Anytime RRT and MA-RRT
choose_target_goal_sample_rate = 0.25
edge_length = 150
k = 10

kappa_rate = 0.000625
def get_k_Nearest_Neighbours(stree, points, target, k):
    #kappa = kappa_rate * 2
    kappa_delta = kappa_rate

    nearest_idx = stree.nearest(target)
    kappa = distance(points[nearest_idx], target) + kappa_delta

    idxs = stree.query(target, predicate="dwithin", distance=kappa).tolist()

    k = k if k <= len(points) else len(points)

    while(len(idxs) < k and len(points) > len(idxs) and kappa < 1.0): # Limit search to one degree
        kappa += kappa_delta
        kappa_delta += kappa_rate
        idxs = stree.query(target, predicate="dwithin", distance=kappa).tolist()
    
    #print(f'k nearest calculates: {len(idxs)}')
    
    sorted_idxs = np.array([ distance(points[i], target) for i in idxs ]).argsort()
    k_nearest_points = [ points[idxs[sorted_idxs[i]]] for i in range(k) ]
    k_nearest_idxs = [ idxs[sorted_idxs[i]] for i in range(k) ]
    return k_nearest_idxs, k_nearest_points

class Node:
    def __init__(self, point, time_of_arrival, cost, parent):
        self.point = point
        self.toa = time_of_arrival
        self.cost = cost
        self.parent = parent
        if(parent == None):
            self.rank = 0
        else:
            self.rank = parent.rank + 1

class Tree:
    def __init__(self, path_dat, wps = [], bounds = [0.0, 0.0, 0.0, 0.0]):
        self.home = Point(wps[0])
        self.dest = Point(wps[-1])

        #print(bounds)

        #epsg25832 = proj.CRS.from_epsg(25832)
        #epsg4326 = proj.CRS.from_epsg(4326)
        #transformer = proj.Transformer.from_crs(epsg4326, epsg25832, True)

        #coord_convert = lambda lon,lat : Point(transformer.transform(lon,lat))
        #print(coord_convert(bounds[0], bounds[1]))
        #print(coord_convert(bounds[2], bounds[3]))

        self.bounds = bounds

        self.cost_analyser = Cost_Analyser(path_dat, [])
        self.h_val = 1
        self.nodes = []

    def _choose_target(self):
        if(rnd.random() < choose_target_goal_sample_rate):
            return self.dest
        
        b_min_x = self.bounds[0]
        b_min_y = self.bounds[1]
        b_max_x = self.bounds[2]
        b_max_y = self.bounds[3]
        
        target = Point([rnd.uniform(b_min_x, b_max_x), rnd.uniform(b_min_y, b_max_y)])
        return target
    
    def _cost_edge(self, node, target):
        p_x, p_y = node.point.xy; p_x = list(p_x)[0]; p_y = list(p_y)[0]
        t_x, t_y = target.xy; t_x = list(t_x)[0]; t_y = list(t_y)[0]
        wps = [(p_x, p_y), (t_x, t_y)]

        #print(f'_cost_edge coords: {p_x, p_y, t_x, t_y}')

        cost, _ = self.cost_analyser.analyse_cost(wps, node.toa)
        #print(f'_cost_edge cost: {cost}')
        return cost
    
    # Returns the closest nodes to targets only from the nodes to the targets in cost-space
    def _nearest_neighbour(self, targets):
        nodes = self.nodes
        points = [ n.point for n in nodes ]
        stree = STRtree(points)

        nearest_neighbours = []

        for t in targets:
            # Get k closest in euclidean distance
            k_nearest_idxs, _ = get_k_Nearest_Neighbours(stree, points, t, k)

            # Estimate cost to each candidate and pick the lowest
            #costs = [ nodes[i].cost + self.h_val * distance(t, nodes[i].point) for i in k_nearest_idxs ]
            #costs = [ self.h_val * distance(t, nodes[i].point) for i in k_nearest_idxs ]
            #selected_nodes = [ nodes[i] for i in k_nearest_idxs ]
            #print([ (n.point,n.rank,e) for n,e in zip(selected_nodes, est_costs) ])
            
            #costs = [ self._cost_heur(nodes[ki], t) + edge_costs[i] for i,ki in enumerate(k_nearest_idxs) ]
            
            target_proj_points = [ self._extend(nodes[i].point, Point(t)) for i in k_nearest_idxs ]
            costs = [ nodes[ni].cost + self._cost_edge(nodes[ni], target_proj_points[i][0]) for i,ni in enumerate(k_nearest_idxs) ] 

            lowest_cost_idx = np.array(costs).argmin()
            lowest_cost_node = nodes[k_nearest_idxs[lowest_cost_idx]]
            #lowest_cost = est_costs[lowest_cost_idx]

            nearest_neighbours.append(lowest_cost_node)
        return nearest_neighbours

    def _extend(self, nearest, target):
        g = Geod(ellps="WGS84")
        idx_fwd_azimuth = 0
        idx_dist = 2
        
        n_x, n_y = nearest.xy; n_x = list(n_x)[0]; n_y = list(n_y)[0]
        t_x, t_y = target.xy; t_x = list(t_x)[0]; t_y = list(t_y)[0]

        # Extend from nearest towards target but only up to edge_length 
        # If target is within edge_length of nearest point return target
        inv_res = g.inv(n_x, n_y, t_x, t_y, return_back_azimuth=False)
        dist = inv_res[idx_dist]
        
        if(dist >= edge_length):
            fwd_az = inv_res[idx_fwd_azimuth]

            #print(fwd_az)

            res = g.fwd(n_x, n_y, fwd_az, edge_length)
            res_lon = res[0]
            res_lat = res[1]

            new = Point(res_lon, res_lat)
            return new, True
        return target, False
    
    def iterate(self, start_time, batch_size = 10, sim_flight_speed = 15.0, sim_flight_temp_res = 1.0):
        self.nodes.append(Node(self.home, start_time, 0.0, None))

        done = False
        iter = 0
        shortest_dist = float("inf")

        t_x, t_y = self.dest.xy; t_x = list(t_x)[0]; t_y = list(t_y)[0]

        while(not done):
            print(f"Doing batch {iter+1}"); iter += 1
            targets = [ self._choose_target() for _ in range(batch_size) ] 
            nearest_nodes_and_costs = self._nearest_neighbour(targets)

            for i,n in enumerate(nearest_nodes_and_costs):
                nearest_node = n
                #path_cost = n[1]
                #print(f'Target: {targets[i]}')
                #print(f'Nearest: {nearest_node.point}')
                new_point, intermediate_point = self._extend(nearest_node.point, targets[i])
                #print(f'New point: {new_point}')

                new_node_edge_cost = self._cost_edge(nearest_node, new_point)

                # TODO: Partial edge_length should return n.toa + distance / sim_flight_speed
                toa = nearest_node.toa + timedelta(0, edge_length / sim_flight_speed) if intermediate_point else nearest_node.toa
                cost = nearest_node.cost + new_node_edge_cost
                #print(f'Nearest node rank: {nearest_node.rank}')
                #print(f'Nearest node cost: {nearest_node.cost}')
                #print(f'New edge cost: {new_node_edge_cost}')
                #print(f'New node cost {cost}')
                new_node = Node(new_point, toa, cost, nearest_node)
                
                self.nodes.append(new_node)

                n_x, n_y = new_point.xy; n_x = list(n_x)[0]; n_y = list(n_y)[0]
                inv_res = g.inv(n_x, n_y, t_x, t_y)
                dist = inv_res[2]
                if(shortest_dist > dist):
                    print(f'Shortest dist: {dist}')
                    print(f'Node rank: {new_node.rank}')
                    shortest_dist = dist

                done = done or new_point == self.dest
                if(done):
                    self.dest = Node(self.dest, new_node.toa + timedelta(0, sim_flight_temp_res) if intermediate_point else new_node.toa, self._cost_edge(new_node, self.dest), new_node)
                    break
                #print(f'New Node Rank: {new_node.rank}')
                
        node_path = []
        current = self.dest
        while(current.parent != None):
            node_path.append(current)

            current = current.parent
        
        point_path = [ n.point for n in node_path ]
        return point_path
    
def find_path(home, dest, engine = None, meta = None, dbsm = None, path_dat = None, start_time = datetime(2023, 4, 26, 20, 00, 00)):
    wps = [home, dest]

    if(path_dat == None):
        bounds = list(LineString(wps).bounds)
        radius = 0.25
        bounds[0] = bounds[0] - radius
        bounds[1] = bounds[1] - radius
        bounds[2] = bounds[2] + radius
        bounds[3] = bounds[3] + radius
        bbox = box(bounds[0], bounds[1], bounds[2], bounds[3]) 
        path_dat = extract_db_data(tables, engine, meta, dbsm, wps, 0.25, bbox)
    RRT = Tree(path_dat, wps, bounds)
    return RRT.iterate(start_time)

if(__name__ == "__main__"):
    env = environmentVars()
    engine, meta, dbsm = setupDB(env['DB_USER'], env['DB_PASS'])

    path = find_path((10.3245895, 55.4718524), (10.3145895, 55.2518524), engine, meta, dbsm)
    print(to_geojson(LineString(path)))

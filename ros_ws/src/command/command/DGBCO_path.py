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

tables = ["dmi"] # ["df", "dmi", "cell", "zone"]
g = Geod(ellps="WGS84")

class DGBCO_Particle:
    def __init__(self, wps = [], num_ctr_points = 5, bounds = [0.0, 0.0, 0.0, 0.0]):
        #self.wps = wps[1:-1]
        self.home = wps[0]
        self.dest = wps[-1]
        self.bounds = bounds

        self.wps = [ [rnd.uniform(bounds[0], bounds[2]), rnd.uniform(bounds[1], bounds[3])] for i in range(num_ctr_points) ]
        #waypoints = g.inv_intermediate(self.home[0], self.home[1], self.dest[0], self.dest[1], num_ctr_points, initial_idx=0, terminus_idx=0)
        #waypoints = [(lon, lat) for lon, lat in zip(waypoints.lons, waypoints.lats)]
        #self.wps = waypoints

    def cost(self, path_dat, start_time):
        if(not hasattr(self, "cost_analyser")):
            self.cost_analyser = Cost_Analyser(path_dat, [])

        waypoints = [self.home]
        #print(self.wps)
        waypoints.extend(self.wps)
        waypoints.extend([self.dest])
        #waypoints = self.home + self.wps + self.dest
        #print(waypoints)
        self.total_cost, self.cost_per_waypoint = self.cost_analyser.analyse_cost(waypoints, start_time)
        return self.total_cost
    
    def get_path(self):
        waypoints = [self.home]
        waypoints.extend(self.wps)
        waypoints.extend([self.dest])
        return waypoints
    
    def iterate(self, type, best, iter, max_iter):
        vector = np.array(self.wps).flatten()
        best_vector = np.array(best).flatten()
        if(type == "e1"):
            #print("Explore")
            if(rnd.random() >= 0.5):
                # Mutate
                r = rnd.randint(0, len(vector)-1)
                b_min = self.bounds[0] if r % 2 != 1 else self.bounds[1]
                b_max = self.bounds[2] if r % 2 != 1 else self.bounds[3]
                vector[r] = b_min + rnd.random() * (b_max - b_min)
            else:
                # Search around current solution
                r1 = np.random.rand(len(vector)) * 2
                r2 = np.random.rand(len(vector)) * 2

                D = r1 * (vector - 1) # TODO: -1 ?
                vector = vector + D * (r2 - 1)
        else:
            #print("Exploit")
            if(rnd.random() >= 0.5):
                # Move towards best
                r3 = np.random.rand(len(vector)) * 2

                D = r3 * (best_vector - vector)
                #print(f'Exploit Move D: {D}')
                vector = vector + D
            else:
                # Search around best
                r4 = np.random.rand(len(vector))
                r5 = np.random.rand(len(vector)) * 2

                k = 2 - ((2*iter**2)/max_iter**2)
                k_vect = [ k for i in range(len(vector)) ]
                #print(k_vect)

                D = best_vector * (k_vect - r4)
                #print(f'Exploit Search D: {D}')
                vector = vector + D * (r5 - 1)

        # Amend solution to within bounds
        b_min_x = self.bounds[0]
        b_min_y = self.bounds[1]
        b_max_x = self.bounds[2]
        b_max_y = self.bounds[3]
                
        for i,s in enumerate(vector):
            if i % 2 == 0: # Even
                if s < b_min_x:
                    vector[i] = b_min_x
                elif s > b_max_x:
                    vector[i] = b_max_x
            else: # Odd
                if s < b_min_y:
                    vector[i] = b_min_y
                elif s > b_max_y:
                    vector[i] = b_max_y
        
        waypoints = vector.reshape((-1, 2)).tolist()
        #print(waypoints)
        #waypoints = waypoints.tolist()
        #waypoints.insert(0, self.wps[0])
        #waypoints.insert(-1)
        #print(waypoints[0])
        self.wps = waypoints
        return waypoints

                # if(rnd.random() >= 0.5):
                #     # Random waypoint
                #     r = rnd.random(len(self.wps))
                #     self.wps[r] = self.wps[r] + (rnd.random(), rnd.random()) * 0.
                # else:
                #     # Worst waypoint

def _init_path(waypoints):
    #waypoints = g.inv_intermediate(waypoints[0][0], waypoints[0][1], waypoints[1][0], waypoints[1][1], 5, initial_idx=1, terminus_idx=1)
    #waypoints = [(lon, lat) for lon, lat in zip(waypoints.lons, waypoints.lats)]
    return waypoints

def _iterate_path(waypoints, path_dat, start_time, bounds, max_iter = 200, particle_count = 44):
    # DGBCO
    #print(bbox)
    #bbox = bbox.coords
    #bounds = [bbox[0], bbox[2]]
    particles = [ DGBCO_Particle(waypoints, 5, bounds) for i in range(particle_count) ]
    #particles_exploitation = [ DGBCO_Particle(waypoints) for i in range(30) ]

    #particles = particles_exploration + particles_exploitation

    best_cost = float("inf")
    best_path = []
    prev1_cost = float("inf")
    prev2_cost = float("inf")

    perc_exploration = 0.7

    iters_without_improvement = 0

    for iter in range(max_iter):
        #print(f"Doing iteration {iter+1} of {max_iter}")
        costs = np.array([ p.cost(path_dat, start_time) for p in particles ])
        idx_min = costs.argmin()
        min_cost = costs[idx_min]

        # costs_e2 = np.array([ p.cost(path_dat, start_time) for p in particles_exploitation ])
        # idx_min_e2 = costs_e2.argmin()
        # e2_min_cost = costs_e2[idx_min_e2]

        # idx_min = idx_min_e1 if e1_min_cost < e2_min_cost else idx_min_e2
        # min_in_e1 = e1_min_cost < e2_min_cost

        if(best_cost > min_cost):
            #print(f'Found new best: {min_cost}')
            best_path = particles[idx_min].get_path()
            best_cost = min_cost
            iters_without_improvement = 0
        iters_without_improvement += 1

        if(iters_without_improvement > 25):
            print(f'Found no better solution for 25 iterations at iteration {iter+1}. Terminating now!')
            break

        k = 2 - (2*iter)/max_iter

        # This is not well described in the algorithm pseudo-code or article. So I used some "common sense" instead. This might not be the best approach.
        if(k < rnd.random() * 2): # Lean towards exploitation towards the end of search
            #print("Adding more exploitation particles")
            perc_exploration = perc_exploration - (perc_exploration) * rnd.random() * 0.12
            #print(f"perc_exploration is now: {perc_exploration}")

        if(min_cost >= prev1_cost and costs[idx_min] >= prev2_cost):
            #print("Adding more exploration particles")
            perc_exploration = perc_exploration + (1 - perc_exploration) * rnd.random() * 0.06
            #print(f"perc_exploration is now: {perc_exploration}")
            # for i in range(3):
            #     if(len(particles_exploitation) >= 1):
            #         particles_exploration.insert(0, particles_exploitation.pop(0)) # Pop one exploitation into exploration
        prev2_cost = prev1_cost
        prev1_cost = min_cost

        for i,p in enumerate(particles):
            if(i == idx_min): # Elitism
                    continue
            if(i <= len(particles) * perc_exploration):
                # Exploration
                #print("Exploration particle")
                p.iterate("e1", particles[idx_min].wps, iter, max_iter)
            else:
                # Exploitation
                #print("Exploitation particle")
                p.iterate("e2", particles[idx_min].wps, iter, max_iter)
        #print(particles)
        rnd.shuffle(particles)
        #print(particles)
        
    #print(best_path)
    #print(best_cost)

    print(to_geojson(LineString(best_path)))

    return best_path, best_cost

def find_path(home, dest, engine, meta, dbsm, start_time):
    wps = [home, dest]

    bounds = list(LineString(wps).bounds)
    radius = 0.25
    bounds[0] = bounds[0] - radius
    bounds[1] = bounds[1] - radius
    bounds[2] = bounds[2] + radius
    bounds[3] = bounds[3] + radius
    bbox = box(bounds[0], bounds[1], bounds[2], bounds[3]) 
    path_dat = extract_db_data(tables, engine, meta, dbsm, wps, 0.25, bbox)

    #path = [(10.3245895, 55.4718524), (10.3145895, 55.2518524), (10.2945895, 55.1518524)]
    path = _init_path(wps)
    path, cost = _iterate_path(path, path_dat, start_time, bounds)

    return path, cost

if(__name__ == "__main__"):
    env = environmentVars()
    engine, meta, dbsm = setupDB(env['DB_USER'], env['DB_PASS'])

    for i in range(10):
        start = time.time()
        path, cost = find_path((10.3245895, 55.4718524), (10.3145895, 55.2518524), engine, meta, dbsm, datetime(2023, 4, 26, 20, 00, 00))
        end = time.time()
        print(f'Cost: {cost} - Elapsed time: {end-start}')
        print(to_geojson(LineString(path)))

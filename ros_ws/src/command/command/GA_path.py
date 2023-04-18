try:
    from utils import *
    from path_info_extract import *
    from path_sim import *
    from simple_path import _cost_path, Cost_Analyser
except ImportError:
    from .utils import *
    from .path_info_extract import *
    from .path_sim import *
    from .simple_path import _cost_path, Cost_Analyser

from shapely import Point, LineString, STRtree, distance, prepare, contains, intersects, envelope, buffer, box, to_geojson
import pyproj as proj
from datetime import datetime
from functools import reduce
import numpy as np
import time

import random as rnd

tables = ["df", "cell", "zone"] # ["df", "dmi", "cell", "zone"]
g = Geod(ellps="WGS84")

discrete_recombination = False;
mutate_rate = 0.2
allele_mutate_rate = 0.25
perc_small_vs_global_mutation = 0.75
small_perc_change_vs_bound_size = 0.01

# Steady-state-selection
top_perc_recombine = 0.1
bot_perc_drop = 0.3

class GA_Agent:
    def __init__(self, wps = [], num_ctr_points = 5, bounds = [0.0, 0.0, 0.0, 0.0]):
        self.home = wps[0]
        self.dest = wps[-1]
        self.bounds = bounds
        self.small_ranges = [(bounds[2]-bounds[0])*small_perc_change_vs_bound_size, (bounds[3]-bounds[1])*small_perc_change_vs_bound_size]

        self.cost_calculated = False

        if(len(wps) <= 2): # If only home and dest
            self.wps = [ [rnd.uniform(bounds[0], bounds[2]), rnd.uniform(bounds[1], bounds[3])] for i in range(num_ctr_points) ]
        else:
            self.wps = wps

    def get_path(self):
        waypoints = [self.home]
        waypoints.extend(self.wps)
        waypoints.extend([self.dest])
        return waypoints

    def cost(self, path_dat, start_time, force=False):
        if(not self.cost_calculated or force):
            if(not hasattr(self, "cost_analyser")):
                self.cost_analyser = Cost_Analyser(path_dat, [])

            waypoints = self.get_path()

            self.total_cost, self.cost_per_waypoint = self.cost_analyser.analyse_cost(waypoints, start_time)
            self.cost_calculated = True
        return self.total_cost
    
    def get_alleles(self): # Returns waypoints as a flattened list
        return np.array(self.wps).flatten()
    
    def set_alleles(self, new_alleles):
        # Amend solution to within bounds
        b_min_x = self.bounds[0]
        b_min_y = self.bounds[1]
        b_max_x = self.bounds[2]
        b_max_y = self.bounds[3]
                
        for i,s in enumerate(new_alleles):
            if i % 2 == 0: # Even
                if s < b_min_x:
                    new_alleles[i] = b_min_x
                elif s > b_max_x:
                    new_alleles[i] = b_max_x
            else: # Odd
                if s < b_min_y:
                    new_alleles[i] = b_min_y
                elif s > b_max_y:
                    new_alleles[i] = b_max_y

        new_alleles = np.array(new_alleles)         
        waypoints = new_alleles.reshape((-1, 2)).tolist()
        self.wps = waypoints
        return waypoints
    
    def mutate(self):
        # TODO: Allow for insertion / deletion of allele pairs
        # Mutation
        old_alleles = self.get_alleles()
        new_alleles = []
        for i,a in enumerate(old_alleles):
            if(perc_small_vs_global_mutation >= rnd.random()): # Small mutation
                new_alleles.insert(len(new_alleles), a + (rnd.random()*2-1)*self.small_ranges[0 if i % 2 == 0 else 1])
            else:
                #r = rnd.randint(0, len(vector)-1)
                b_min = self.bounds[0] if i % 2 != 1 else self.bounds[1]
                b_max = self.bounds[2] if i % 2 != 1 else self.bounds[3]
                new_alleles.insert(len(new_alleles), b_min + rnd.random() * (b_max - b_min))

        self.set_alleles(new_alleles)
        self.cost_calculated = False
        #new_alleles = [ a if  for i,a in enumerate(self.get_alleles()) ]

def _init_path(waypoints):
    #waypoints = g.inv_intermediate(waypoints[0][0], waypoints[0][1], waypoints[1][0], waypoints[1][1], 5, initial_idx=1, terminus_idx=1)
    #waypoints = [(lon, lat) for lon, lat in zip(waypoints.lons, waypoints.lats)]
    return waypoints

def _random_between_except(a, b, c):
    #print(f'{a};{b};{c}')
    lst = list(range(a, b))
    lst.remove(c)
    return rnd.sample(lst, 1)[0]

def _iterate_path(waypoints, path_dat, start_time, bounds, max_iter = 200, particles = 100):
    agents = [ GA_Agent(waypoints, 5, bounds) for i in range(particles) ]

    best_cost = float("inf")
    best_path = []

    iters_without_improvement = 0

    for iter in range(max_iter):
        #print(f"Doing iteration {iter+1} of {max_iter}")
        costs = np.array([ a.cost(path_dat, start_time) for a in agents ])
        idx_min = costs.argmin()
        min_cost = costs[idx_min]

        if(best_cost > min_cost):
            #print(f'Found new best: {min_cost}')
            best_path = agents[idx_min].get_path()
            best_cost = min_cost
            iters_without_improvement = 0
        iters_without_improvement += 1

        if(iters_without_improvement > 25):
            print(f'Found no better solution for 25 iterations at iteration {iter+1}. Terminating now!')
            break

        idxs_sorted_asc = costs.argsort(kind='heapsort')
        
        sorted_agents = [ agents[i] for i in idxs_sorted_asc ] 
        sorted_agents = sorted_agents[0:round(len(idxs_sorted_asc)*(1-bot_perc_drop))]
        
        # Top % always recombine
        new_agents = [ _recombine_parents(sorted_agents[i], sorted_agents[_random_between_except(0,len(sorted_agents),i)]) for i in range(round(particles*top_perc_recombine)) ] 

        # Others recombine
        while(len(sorted_agents) + len(new_agents) < particles):
            idx_a = rnd.randint(0, len(sorted_agents)-1)
            idx_b = _random_between_except(0, len(sorted_agents), idx_a)
            new_agents.extend( [_recombine_parents(sorted_agents[idx_a], sorted_agents[idx_b])] )

        sorted_agents.extend(new_agents)

        # Mutate at random
        for i in range(1, len(sorted_agents)): # Start at 1 (Elitism)
            if(rnd.random() <= mutate_rate):
                sorted_agents[i].mutate()
        
        agents = sorted_agents

    print(to_geojson(LineString(best_path)))
    return best_path, best_cost

def _recombine_parents(parent_a = None, parent_b = None):
    if(parent_a == None or parent_b == None):
        return None
    alleles_a = parent_a.get_alleles()
    alleles_b = parent_b.get_alleles()

    bounds = parent_a.bounds # Get bounds from either parent

    alleles_c = []
    # Discrete recombination
    if(discrete_recombination):
        alpha = 0.5 # Assume 50% contribution from each parent
        alleles_c = [ a if rnd.random() > alpha else b for a,b in zip(alleles_a, alleles_b) ]
        # Zip shortens to the shorter of the parents
        # TODO: Consider padding with the longer parents remainder alleles

    # Intermediate recombination
    else:
        # a*beta + b*(1-beta) -> beta*(a-b)+b
        alleles_c = [ rnd.random()*(a-b)+b for a,b in zip(alleles_a, alleles_b) ]

    alleles_c = np.array(alleles_c)
    waypoints = alleles_c.reshape((-1, 2)).tolist()

    new_agent = GA_Agent(waypoints, bounds=bounds)
    new_agent.home = parent_a.home; new_agent.dest = parent_a.dest # Keep extremes intact
    return new_agent

def find_path(home, dest, engine, meta, dbsm):
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
    path, cost = _iterate_path(path, path_dat, datetime(2023, 4, 3, 12, 00, 00), bounds)

    #print(path)
    return path, cost

if(__name__ == "__main__"):
    env = environmentVars()
    engine, meta, dbsm = setupDB(env['DB_USER'], env['DB_PASS'])

    for i in range(10):
        start = time.time()
        path, cost = find_path((10.3245895, 55.4718524), (10.3145895, 55.2518524), engine, meta, dbsm)
        end = time.time()
        print(f'Cost: {cost} - Elapsed time: {end-start}')

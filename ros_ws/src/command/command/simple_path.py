try:
    from utils import *
    from path_info_extract import *
    from path_sim import *
except ImportError:
    from .utils import *
    from .path_info_extract import *
    from .path_sim import *

from shapely import Point, STRtree, distance, prepare, contains, intersects
import pyproj as proj
from datetime import datetime
from functools import reduce
import numpy as np

tables = ["df", "dmi", "cell", "zone"]
DMI_TS_FORMAT = "%Y-%m-%dT%H%M%S"

def _init_path(waypoints):
    return waypoints

class Cost_Analyser():
    def __init__(self, path_dat, zone_greenlist):
        self.prepare_dat(path_dat, zone_greenlist)

    def prepare_dat(self, path_dat, zone_greenlist):
        if("dmi" in path_dat):
            dmi_by_dts = {}
            for p in path_dat['dmi']: # Linear pass to sort into buckets by timestamp
                dt = datetime.strftime(p['dt'], DMI_TS_FORMAT)
                if(dt in dmi_by_dts):
                    dmi_by_dts[dt].extend([(Point(p['geom']), p)])
                else:
                    dmi_by_dts[dt] = [(Point(p['geom']), p)]

            self.dmi = []
            for dt in dmi_by_dts: # Build STRTree per timestamp for later lookup
                stree = STRtree([ p[0] for p in dmi_by_dts[dt] ])
                points = [ p[1] for p in dmi_by_dts[dt] ]
                dmi_by_dts[dt] = {'stree': stree, 'points': points} # Keep point together for later cost calculation
                self.dmi.append(dmi_by_dts[dt])
            self.dmi_by_dts = dmi_by_dts
            self.dmi_dts = [ datetime.strptime(dt, DMI_TS_FORMAT) for dt in dmi_by_dts ]

        if("cell" in path_dat):
            self.cell_dat = path_dat['cell']

            # TODO: Filter by technology and radio available on drone
            self.cell_stree = STRtree([ c['geom'] for c in self.cell_dat ])
    
        if("df" in path_dat):
            cities = [ infs for infs in path_dat['df'] if infs['itype'] == "city" ]
            #print(cities)

            self.prepped_cities = [ (c['geom'], c) for c in cities ]
            for pc in self.prepped_cities:
                prepare(pc[0])
        
        if("zone" in path_dat):
            notams = path_dat['notam']

            self.notams_by_zname = {}
            for n in notams:
                self.notams_by_zname[n['zname']] = n
            
            zones = path_dat['zone']
            notam_zones = [ n['zname'] for n in notams ]
            activated_zones = [ z for z in zones if (z['zname'] in notam_zones and not z['zname'] in zone_greenlist) ]

            # Prepare geometry for better performance
            self.prepped_zones = [ (z['geom'], z) for z in activated_zones ]
            for pz in self.prepped_zones:
                prepare(pz[0])

    def analyse_cost(self, waypoints, start_time, sim_dist = 15.0, sim_temp_res = 1.0):
        epsg25832 = proj.CRS.from_epsg(25832)
        epsg4326 = proj.CRS.from_epsg(4326)
        transformer = proj.Transformer.from_crs(epsg4326, epsg25832, True)

        pos, num_points = simpleSimulate(waypoints, start_time, sim_dist, sim_temp_res)
        pos_nt = [ Point(p[0], p[1]) for p in pos ]

        #print(f'Length of pos: {len(pos)}')

        cost_per_pos = np.zeros(len(pos))

        if(hasattr(self, "dmi")):
            pos_map = []
            for p in pos:
                # Find shortest time
                dt_delta = [ abs(p[2] - dt) for dt in self.dmi_dts ]
                min_idx = np.argmin(dt_delta)
                #min_dt = min(self.dmi_dts, key=lambda t: abs(p[2] - t))
                dmi = self.dmi[min_idx]

                # Find shortest point to point
                nearest_idx = dmi['stree'].nearest(Point(p[0], p[1]))
                pos_map.append(dmi['points'][nearest_idx]) # Snap positions to nearest position from DMI

            precips = [ p['precip_meas'] for p in pos_map ]
            windspds = [ p['windspd_meas'] for p in pos_map ]
            windspds = [ (s if s >= 5.0 else 0.0) for s in windspds ]
            cost_per_pos = cost_per_pos + precips + windspds
        #print(f'Post DMI: {reduce(lambda a,b: a+b, cost_per_pos)}')

        if(hasattr(self, "cell_stree")):
            pos_map = []
            for p in pos:
                nearest_idx = self.cell_stree.nearest(Point(p[0], p[1]))
                pos_map.append(self.cell_dat[nearest_idx])
            
            # Convert to EPSG:25832 to measure in meters
            coord_convert = lambda lon,lat : Point(transformer.transform(lon,lat))
            point_convert = lambda point : Point(transformer.transform(list(point['geom'].coords)[0][0], list(point['geom'].coords)[0][1]))
            # Using distance even though it degenerates to Euclidean distance
            dists_to_nearest = [ distance(coord_convert(p[0], p[1]), point_convert(cell)) for p, cell in zip(pos, pos_map) ]

            # Calc cost based on min and max distances
            # TODO: Consider basing this on technology type as well
            min_dist = 100.0
            max_dist = 3000.0
            dist_cost_func = lambda d : 0.0 if d >= min_dist and d <= max_dist else (((d-max_dist)*0.003)**2 if d > max_dist else ((d-min_dist)*0.02)**10)
            dist_cost = [ dist_cost_func(d) for d in dists_to_nearest ]
            cost_per_pos = cost_per_pos + dist_cost
        #print(f'Post Cell: {reduce(lambda a,b: a+b, cost_per_pos)}')

        if(hasattr(self, "prepped_cities")):
            for pc in self.prepped_cities:
                pc_violations = contains(pc[0], pos_nt)
                cost_per_violation = 100.0
                city_cost = [ (cost_per_violation if pc_violations[i] else 0.0) for i, p in enumerate(pos) ]
                cost_per_pos = cost_per_pos + city_cost
        #print(f'Post City: {reduce(lambda a,b: a+b, cost_per_pos)}')
        
        if(hasattr(self, "prepped_zones")):
            for pz in self.prepped_zones:
                zname = pz[1]['zname']
                pz_violations = contains(pz[0], pos_nt)

                cost_per_violation = 1000.0
                zone_cost = [ (cost_per_violation if (pz_violations[i] and (p[2] >= self.notams_by_zname[zname]['act_from'] and p[2] <= self.notams_by_zname[zname]['act_to'])) else 0.0 ) for i, p in enumerate(pos) ]
                
                cost_per_pos = cost_per_pos + zone_cost
        #print(f'Post Zone: {reduce(lambda a,b: a+b, cost_per_pos)}')

        cost_per_pos = cost_per_pos + ((sim_dist*sim_temp_res) / 100)
        #print(len(cost_per_pos))

        running = 1 # Start after first after home
        cost_per_waypoint = np.zeros(len(waypoints))

        for i,n in enumerate(num_points):
            start = running
            stop = running+n
            segment = cost_per_pos[start:stop]
            cost_per_waypoint[i+1] = reduce(lambda a,b: a+b, segment) if len(segment) > 0 else 0
            running += n

        #print(f'cost_per_pos: {cost_per_pos}')
        #print(f'cost_per_waypoint: {cost_per_waypoint}')

        total_cost = reduce(lambda a,b: a+b, cost_per_waypoint)
        #print(cost_per_waypoint)
        #print(total_cost)

        return total_cost, cost_per_waypoint        

#def _iterate_path(waypoints, path_dat, start_time):
    # Normally a loop here
#    cost = _cost_path(waypoints, path_dat, start_time)

#    return waypoints

#def find_path(home, dest, engine, meta, dbsm):
#    wps = [home, dest]
#    path_dat = extract_db_data(tables, engine, meta, dbsm, wps, 0.25)

    #path = [(10.3245895, 55.4718524), (10.3145895, 55.2518524), (10.2945895, 55.1518524)]
#    path = _init_path(wps)
#    path = _iterate_path(path, path_dat, datetime(2023, 4, 3, 12, 00, 00))

#    return path

if(__name__ == "__main__"):
    env = environmentVars()
    engine, meta, dbsm = setupDB(env['DB_USER'], env['DB_PASS'])
    #find_path((10.3245895, 55.4718524), (10.3145895, 55.2518524), engine, meta, dbsm)
    
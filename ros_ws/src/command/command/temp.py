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

from shapely import Point, LineString, STRtree, distance, prepare, contains, intersects, envelope, buffer, box, to_geojson, from_geojson
import pyproj as proj
from datetime import datetime
from functools import reduce
import numpy as np
import time

import random as rnd

tables = ["df", "dmi", "cell", "zone"]
g = Geod(ellps="WGS84")

def to_geojson_feature(geom, css):
    return f'{{"type":"Feature","properties":{{{css}}},"geometry":{to_geojson(geom)}}}'
    

def find_path(home, dest, engine = None, meta = None, dbsm = None, path_dat = None, start_time = datetime(2023, 4, 26, 20, 00, 00)):
    wps = [home, dest]

    bounds = list(LineString(wps).bounds)
    radius = 0.25
    bounds[0] = bounds[0] - radius
    bounds[1] = bounds[1] - radius
    bounds[2] = bounds[2] + radius
    bounds[3] = bounds[3] + radius

    if(path_dat == None):
        bbox = box(bounds[0], bounds[1], bounds[2], bounds[3]) 
        path_dat = extract_db_data(tables, engine, meta, dbsm, wps, 0.25, bbox)
        with open("temp_data.txt", "w") as file:
            file.write((", ").join([ to_geojson_feature(d["geom"], '"fill": "#f9f06b","fill-opacity": 0.5') for d in path_dat["df"]]))
            file.write(", ")
            file.write((", ").join([ to_geojson_feature(d["geom"], '"fill": "#f66151","fill-opacity": 0.5') for d in path_dat["zone"]]))
            file.write(", ")
            file.write((", ").join([ to_geojson_feature(d["geom"], '"marker-color": "#8ff0a4","marker-size": "small"') for d in path_dat["cell"]]))

            file.write(", ")
            dt = datetime.strptime("2023-04-26 20:00:00", "%Y-%m-%d %H:%M:%S")
            filtered_dmi = [d for d in path_dat["dmi"] if (d['dt'] == dt and (d['precip_meas'] > 0.25 or d['windspd_meas'] > 5.0))]
            file.write((", ").join([ to_geojson_feature(d["geom"], '"marker-color": "#dc8add","marker-size": "small"') for d in filtered_dmi ]))

            file.write(", ")
            path = from_geojson('{"type":"LineString","coordinates":[[10.3245895,55.4718524],[10.470225443824578,55.206367486229425],[10.161529769032171,55.43612621862618],[10.568555686993719,55.03444783097374],[10.435765367208703,55.15169847459572],[10.3395696970241,55.099730920520834],[10.3145895,55.2518524]]}')
            file.write(to_geojson_feature(path, '"stroke": "#1a5fb4","stroke-width": 4,"stroke-opacity": 1'))

            sim_points, _ = simpleSimulate([[10.3245895,55.4718524],[10.470225443824578,55.206367486229425]], speed=300.0)
            sim_points = [ Point(s[0], s[1]) for s in sim_points ]
            file.write(", ")
            file.write((", ").join([ to_geojson_feature(d, '"marker-color": "#ffffff","marker-size": "small"') for d in sim_points]))
            
if(__name__ == "__main__"):
    env = environmentVars()
    engine, meta, dbsm = setupDB(env['DB_USER'], env['DB_PASS'])

    find_path((10.3245895, 55.4718524), (10.3145895, 55.2518524), engine, meta, dbsm)
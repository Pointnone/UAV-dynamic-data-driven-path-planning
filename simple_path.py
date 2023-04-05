
from setup import setupDB, environmentVars
import path_info_extract as piex
import path_sim as ps

from shapely import LineString, Polygon, Point, to_wkt, STRtree
from datetime import datetime
from functools import reduce

tables = ["df", "dmi", "cell", "zone"]
DMI_TS_FORMAT = "%Y-%m-%dT%H%M%S"

def _init_path(waypoints):
    return waypoints

def _cost_path(waypoints, path_dat, start_time):
    pos = ps.simpleSimulate(waypoints, start_time, 15.0, 1.0)

    if(path_dat['dmi']):
        dmi_by_dts = {}
        for p in path_dat['dmi']:
            dt = datetime.strftime(p['dt'], DMI_TS_FORMAT)
            if(dt in dmi_by_dts):
                dmi_by_dts[dt].extend([(Point(p['geom']), p)])
            else:
                dmi_by_dts[dt] = [(Point(p['geom']), p)]

        for dt in dmi_by_dts:
            stree = STRtree([ p[0] for p in dmi_by_dts[dt] ])
            points = [ p[1] for p in dmi_by_dts[dt] ]
            dmi_by_dts[dt] = {'stree': stree, 'points': points}

        pos_map = []
        for p in pos:
            # Find shortest time
            min_dt = min(dmi_by_dts, key=lambda t: abs(p[2] - datetime.strptime(t, DMI_TS_FORMAT)))
            dmi = dmi_by_dts[min_dt]

            # Find shortest point to point
            nearest_idx = dmi['stree'].nearest(Point(p[0], p[1]))
            pos_map.append(dmi['points'][nearest_idx]) # Snap positions to nearest position from DMI

        #print([ p['dt'] for p in pos_map ])
        #return
        #pos_times = {}
        #for i in pos_map:
        #    pos_times[path_dat['dmi'][i]['geom']] = 

        precips = [ p['precip_meas'] for p in pos_map ]
        cost_precip = reduce(lambda a,b: a+b, precips)
        #print(cost_precip)

        windspds = [ p['windspd_meas'] for p in pos_map ]
        windspds = [ (s if s >= 2.0 else 0.0) for s in windspds ]
        #print(windspds)
        cost_windspds = reduce(lambda a,b: a+b, windspds)
        print(cost_windspds)

def _iterate_path(waypoints, path_dat, start_time):
    # Normally a loop here
    cost = _cost_path(waypoints, path_dat, start_time)

    return waypoints

def find_path(home, dest, engine, meta, dbsm):
    wps = [home, dest]
    path_dat = piex.extract_db_data(tables, engine, meta, dbsm, wps, 0.5)

    path = _init_path(wps)
    path = _iterate_path(path, path_dat, datetime(2023, 4, 3, 12, 00, 00))

    #print(path_dat)

    #for p in path_dat:
    #    print(p)
    #    print(len(path_dat[p]))

if(__name__ == "__main__"):
    environmentVars()
    engine, meta, dbsm = setupDB()
    find_path((10.3245895, 55.4718524), (10.3145895, 55.0518524), engine, meta, dbsm)
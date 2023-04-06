from sqlalchemy import Table, select, func
from sqlalchemy.orm import sessionmaker
from sqlalchemy.engine import URL

from geoalchemy2 import Geometry, shape
from shapely import LineString, Polygon, Point, to_wkt

import xml.etree.ElementTree as ET

import pyproj as proj
from geojson import Polygon

from utils import setupDB, environmentVars

def waypoint_to_linestring(waypoints):
    return to_wkt(LineString(waypoints))

def extract_db_data(tables, engine, meta, dbsm, waypoints, radius = 0.25):
    ret = {}
    with dbsm() as session:
        for table in tables:
            C_Table = Table(table, meta, autoload_with=engine)
            c = func.ST_Buffer(waypoint_to_linestring(waypoints), radius, 'endcap=round join=round')
            s = select([C_Table], C_Table.c.geom.ST_Intersects(c))
            res = [dict(r) for r in session.execute(s).all()]

            for row in res:
                row['geom'] = shape.to_shape(row['geom'])
            
            ret[table] = res

        if(ret['zone']):
            N_Table = Table('notam', meta, autoload_with=engine)
            s = select([N_Table], N_Table.c.zname.in_([z['zname'] for z in ret['zone']]))
            res = [dict(r) for r in session.execute(s).all()]

            ret['notam'] = res

    return ret
    
if(__name__ == "__main__"):
    env = environmentVars()
    engine, meta, dbsm = setupDB(env['DB_USER'], env['DB_PASS'])
    waypoints = [(9.538999, 55.706185), (9.538999, 54.979835), (12.004298, 54.979835)]
    data = extract_db_data(["df", "dmi", "cell", "zone"], engine, meta, dbsm, waypoints, 0.25)
    print(data)

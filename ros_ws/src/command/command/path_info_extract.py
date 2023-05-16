from sqlalchemy import Table, select, func
from sqlalchemy.dialects.postgresql import insert
from sqlalchemy.orm import sessionmaker
from sqlalchemy.engine import URL

from geoalchemy2 import Geometry, shape
from shapely import LineString, Polygon, Point, to_wkt

import xml.etree.ElementTree as ET

import pyproj as proj
from geojson import Polygon

try:
    from utils import *
except ImportError:
    from .utils import *

def setupDB(db_user, db_pass):
    urlObj = URL("postgresql", db_user, db_pass, "localhost", 5432, "geomdata")
    engine = create_engine(urlObj, echo=False)
    meta = MetaData()
    dbsm = sessionmaker(engine)
    return engine, meta, dbsm

def waypoint_to_linestring(waypoints):
    return to_wkt(LineString(waypoints))

class DB_Handler():
    def __init__(self):
        env = environmentVars()
        self.engine, self.meta, self.dbsm = setupDB(env['DB_USER'], env['DB_PASS'])

    def extract_db_data(self, tables, waypoints, radius = 0.25, geometry = None):
        ret = {}
        with self.dbsm() as session:
            for table in tables:
                C_Table = Table(table, self.meta, autoload_with=self.engine)
                c = None
                if(geometry == None):
                    c = func.ST_Buffer(waypoint_to_linestring(waypoints), radius, 'endcap=round join=round')
                else:
                    c = to_wkt(geometry)
                s = select([C_Table], C_Table.c.geom.ST_Intersects(c))
                res = [dict(r) for r in session.execute(s).all()]

                for row in res:
                    row['geom'] = shape.to_shape(row['geom'])
                
                ret[table] = res

            if("zone" in ret):
                N_Table = Table('notam', self.meta, autoload_with=self.engine)
                znames = [z['zname'] for z in ret['zone']]
                s = select([N_Table]) # .where(N_Table.c.zname.in_(znames))
                res = session.execute(s).fetchall()
                res = [dict(r) for r in res]

                res = [ r for r in res if r['zname'] in znames ]

                ret['notam'] = res
        return ret
    
    def extract_drone_paths(self):
        with self.dbsm() as session:
            D_Table = Table("drone_paths", self.meta, autoload_with=self.engine)
            s = select([D_Table])
            res = [dict(r) for r in session.execute(s).all()]

            for row in res:
                row['path'] = shape.to_shape(row['path'])
            return res

    def insert_drone_into_drone_paths(self, data):
        with self.dbsm() as session:
            C_Table = Table("drone_paths", self.meta, autoload_with=self.engine)
            s = insert(C_Table)
            s = s.on_conflict_do_update(
                index_elements=[C_Table.c.drone], set_=dict(s.excluded.items())
            )
            session.execute(s, data)
            session.commit()

def extract_db_data(tables, engine, meta, dbsm, waypoints, radius = 0.25, geometry = None):
    ret = {}
    with dbsm() as session:
        for table in tables:
            C_Table = Table(table, meta, autoload_with=engine)
            c = None
            if(geometry == None):
                c = func.ST_Buffer(waypoint_to_linestring(waypoints), radius, 'endcap=round join=round')
            else:
                c = to_wkt(geometry)
            s = select([C_Table], C_Table.c.geom.ST_Intersects(c))
            res = [dict(r) for r in session.execute(s).all()]

            for row in res:
                row['geom'] = shape.to_shape(row['geom'])
            
            ret[table] = res

        if("zone" in ret):
            N_Table = Table('notam', meta, autoload_with=engine)
            znames = [z['zname'] for z in ret['zone']]
            s = select([N_Table]) # .where(N_Table.c.zname.in_(znames))
            res = session.execute(s).fetchall()
            res = [dict(r) for r in res]

            res = [ r for r in res if r['zname'] in znames ]

            ret['notam'] = res

    return ret
    
if(__name__ == "__main__"):
    env = environmentVars()
    engine, meta, dbsm = setupDB(env['DB_USER'], env['DB_PASS'])
    #waypoints = [(9.538999, 55.706185), (9.538999, 54.979835), (12.004298, 54.979835)]
    #data = extract_db_data(["df", "dmi", "cell", "zone"], engine, meta, dbsm, waypoints, 0.25)
    #print(data)
    print(extract_drone_paths(engine, meta, dbsm))

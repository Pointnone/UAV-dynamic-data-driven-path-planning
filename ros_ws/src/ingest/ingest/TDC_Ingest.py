#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import csv

from sqlalchemy import Table 
from sqlalchemy.dialects.postgresql import insert
from sqlalchemy.orm import sessionmaker
from sqlalchemy.engine import URL

from geoalchemy2 import Geometry, shape

def buildTowerDBRecords(entries, is_sim = False, sim_layer = 0):
    entries_set = {f'{e[3]},{e[2]}' for e in entries} # Set using string compression for comprehension (id, long, lat)
    entries_lst = [e.split(",") for e in list(entries_set)]

    entries_radio = [ {f'{e[4]}' for e in entries if e[3] == coord[0] and e[2] == coord[1]} for coord in entries_lst]

    records = [{"geom": f'POINT({entries_lst[i][0]} {entries_lst[i][1]})',
                "technology": ','.join(entries_radio[i]),
                "sim": is_sim,
                "sim_layer": sim_layer} for i in range(len(entries_lst))]

    return records

def updateTDCData(engine, meta, dbsm):
    with open('outputny.csv', newline='') as csvfile:
        csvreader = csv.reader(csvfile, delimiter=';')
        entries = [e for e in csvreader]

        with dbsm() as session:
            Cell_Table = Table("cell", meta, autoload_with=engine) # Table Reflection
            
            # Upsert
            s = insert(Cell_Table)
            s = s.on_conflict_do_update(
                index_elements=[Cell_Table.c.geom], set_=dict(s.excluded.items())
            )

            session.execute(s, buildTowerDBRecords(entries))
            session.commit()
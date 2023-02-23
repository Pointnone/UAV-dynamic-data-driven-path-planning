#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import csv

from sqlalchemy import Table, Column, Float, Integer, String, MetaData, create_engine, insert
from sqlalchemy.orm import sessionmaker
from sqlalchemy.engine import URL

def buildTowerDBRecords(entries, is_sim = False, sim_layer = 0):
    entries_set = {f'{e[0]},{e[3]},{e[2]}' for e in entries} # Set using string compression for comprehension (id, long, lat)
    entries_lst = [e.split(",") for e in list(entries_set)]

    records = [{"geom": f'POINT({entries_lst[i][1]} {entries_lst[i][2]})',
                "technology": "Test",
                "sim": is_sim,
                "sim_layer": sim_layer} for i in range(len(entries_lst))]

    return records

def updateTDCData(engine, meta, dbsm):
    with open('outputny.csv', newline='') as csvfile:
        csvreader = csv.reader(csvfile, delimiter=';')
        entries = [e for e in csvreader]

        with dbsm() as session:
            Cell_Table = Table("cell", meta, autoload_with=engine) # Table Reflection
            session.execute(insert(Cell_Table), buildTowerDBRecords(entries))
            session.commit()
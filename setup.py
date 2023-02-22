#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 15 11:29:43 2023

@author: point
"""
from dotenv import load_dotenv
from pathlib import Path
import os

import pygrib
import numpy as np
#import cfgrib

from sqlalchemy import Table, Column, Float, Integer, String, MetaData, create_engine, insert
from sqlalchemy.orm import sessionmaker
from sqlalchemy.engine import URL

from datetime import datetime

from DF_Ingest import *
from DMI_Ingest import *

def environmentVars():
    env_path = Path('./.env')
    load_dotenv(dotenv_path=env_path)

    global DMI_API_KEY; DMI_API_KEY = os.getenv('DMI_API_KEY')
    global DF_USER; DF_USER = os.getenv('DF_USER')
    global DF_PASS; DF_PASS = os.getenv('DF_PASS')
    global DB_PASSWORD; DB_PASSWORD = os.getenv('DB_PASSWORD')

def setupDB():
    urlObj = URL("postgresql", "postgres", DB_PASSWORD, "localhost", 5432, "geomdata")
    global engine; engine = create_engine(urlObj, echo=False)
    global meta; meta = MetaData()
    global dbsm; dbsm = sessionmaker(engine)

global DMI_idxs
DMI_idxs = [
    {'idx': 42, 'mtype': 'precip'},
    {'idx': 87, 'mtype': 'winddir'},
    {'idx': 88, 'mtype': 'windspd'}
]

def updateDMIData():
    # TODO: Get all datapoint from last modelrun
    last = findLatest(DMI_API_KEY)
    grib_file = getGribFile(last, DMI_API_KEY)

    grbs = pygrib.open(f"./{grib_file}")

    time_str = (" ").join(grib_file.split("_")[-1].split(".")[0][:-1].split("T"))
    time_str = time_str[:-4] + ":" + time_str[-4:-2] + ":" + time_str[-2:]
    
    setupDB()
    values = {}
    
    for DMI_idx in DMI_idxs:
        grb = grbs[DMI_idx['idx']]

        values[DMI_idx['mtype']] = np.array(grb['values']).reshape(-1)
        
        if("(accum)" in grb.__str__()):
            before = findBefore(DMI_API_KEY, last)
            before_grib_file = getGribFile(before, DMI_API_KEY)

            before_grbs = pygrib.open(f"./{before_grib_file}")
            before_grb = before_grbs[DMI_idx['idx']]

            before_values = np.array(before_grb['values']).reshape(-1)

            values[DMI_idx['mtype']] -= before_values # Element-wise list subtraction
    
    lats, lons = grbs[42].latlons()
    lats = np.array(lats).reshape(-1)
    lons = np.array(lons).reshape(-1)

    with dbsm() as session:
        DMI_Table = Table("dmi", meta, autoload_with=engine) # Table Reflection
        session.execute(insert(DMI_Table), buildDBRecords(lats, lons, values, time_str))
        session.commit()

environmentVars()
updateDMIData()
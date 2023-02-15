#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 15 11:29:43 2023

@author: point
"""
from dotenv import load_dotenv
from pathlib import Path
import os

import numpy as np

from sqlalchemy import Table, Column, Float, Integer, String, MetaData, create_engine, insert
from sqlalchemy.orm import sessionmaker
from sqlalchemy.engine import URL

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
    urlObj = URL("postgresql", "postgres", DB_PASSWORD, "localhost", 5432, "postgres")
    global engine; engine = create_engine(urlObj, echo=False)
    global meta; meta = MetaData()
    global dbsm; dbsm = sessionmaker(engine)

def updateDMIData():
    last = findLatest(DMI_API_KEY)
    grib_file = getGribFile(last, DMI_API_KEY)

    grbs = pygrib.open(f"./{grib_file}")
    grbs.seek(0)

    lats, lons = grbs[42].latlons()
    lats = np.array(lats).reshape(-1)
    lons = np.array(lons).reshape(-1)

    precipitation = np.array(grbs[42]['values']).reshape(-1)

    print(precipitation)
    print(lats, lons)

    setupDB()
    with dbsm() as session:
        DMI_Table = Table("dmi", meta, autoload_with=engine) # Table Reflection
        session.execute(insert(DMI_Table), buildDBRecords("precip", lats, lons, precipitation))
        session.commit()

environmentVars()
updateDMIData()
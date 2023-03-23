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

from sqlalchemy import MetaData, create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.engine import URL

from datetime import datetime

from DF_Ingest import *
from DMI_Ingest import *
from TDC_Ingest import *
from Naviair_Ingest import *

def environmentVars():
    env_path = Path('./.env')
    load_dotenv(dotenv_path=env_path)

    global DMI_API_KEY; DMI_API_KEY = os.getenv('DMI_API_KEY')
    global DF_USER; DF_USER = os.getenv('DF_USER')
    global DF_PASS; DF_PASS = os.getenv('DF_PASS')
    global DB_PASSWORD; DB_PASSWORD = os.getenv('DB_PASSWORD')
    global NAVIAIR_API_KEY; NAVIAIR_API_KEY = os.getenv('NAVIAIR_API_KEY')

def setupDB():
    urlObj = URL("postgresql", "postgres", DB_PASSWORD, "localhost", 5432, "geomdata")
    global engine; engine = create_engine(urlObj, echo=False)
    global meta; meta = MetaData()
    global dbsm; dbsm = sessionmaker(engine)
    return engine, meta, dbsm

if(__name__ == "__main__"):
    environmentVars()
    setupDB()
    updateTDCData(engine, meta, dbsm)
    updateNaviairData(NAVIAIR_API_KEY, engine, meta, dbsm)
    updateDFData(DF_USER, DF_PASS, engine, meta, dbsm)
    updateDMIData(DMI_API_KEY, engine, meta, dbsm)
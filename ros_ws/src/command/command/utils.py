#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 15 11:29:43 2023

@author: point
"""
from dotenv import load_dotenv
from pathlib import Path
import os

from sqlalchemy import MetaData, create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.engine import URL

envKeys = ['DMI_API_KEY', 'DF_USER', 'DF_PASS', 'DB_USER', 'DB_PASS', 'NAVIAIR_API_KEY']

def environmentVars():
    env_path = Path('./.env')
    load_dotenv(dotenv_path=env_path)

    env = {}
    for key in envKeys:
        val = os.getenv(key); env[key] = val
    return env

def setupDB(db_user, db_pass):
    urlObj = URL("postgresql", db_user, db_pass, "localhost", 5432, "geomdata")
    engine = create_engine(urlObj, echo=False)
    meta = MetaData()
    dbsm = sessionmaker(engine)
    return engine, meta, dbsm
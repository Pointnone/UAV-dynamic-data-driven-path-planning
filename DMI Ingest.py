#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb  2 16:21:13 2023

@author: point
"""
from dotenv import load_dotenv
from pathlib import Path
import os

import requests
import pygrib

env_path = Path('./.env')
load_dotenv(dotenv_path=env_path)

DMI_API_KEY = os.getenv('DMI_API_KEY')
#print(DMI_API_KEY)

def findLatest():
    url = 'https://dmigw.govcloud.dk/v1/forecastdata/collections/harmonie_nea_sf/items'
    headers = {'X-Gravitee-Api-Key': DMI_API_KEY}
    r = requests.get(url, headers=headers)
    
    resp = r.json()
    
    # TODO: Consider iterating through to find the actual latest. Assumption: Last in the list is the latest
    latest_url = resp['features'][-1]['asset']['data']['href']
    return latest_url
    
def getGribFile(url):
    filename = url.split('/')[-1]
    headers = {'X-Gravitee-Api-Key': DMI_API_KEY}
    
    with requests.get(url, headers=headers, stream=True) as req:
        req.raise_for_status()
        with open(filename, 'wb') as fd:
            for chunk in req.iter_content(chunk_size=8192):
                fd.write(chunk)
    return filename

last = findLatest()
grib_file = getGribFile(last)

grbs = pygrib.open("./" + grib_file)
grbs.seek(0)

#grb = grbs[42]
#print(grb)

#lats, lons = grb.latlons()

#print(len(lats[0]))
#print("-----")
#print(len(lons))
#print(len(grb.values[0]))
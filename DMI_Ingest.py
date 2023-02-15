#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb  2 16:21:13 2023

@author: point
"""
import os

import requests
import pygrib

import numpy as np

def findLatest(DMI_API_KEY):
    url = 'https://dmigw.govcloud.dk/v1/forecastdata/collections/harmonie_nea_sf/items'
    headers = {'X-Gravitee-Api-Key': DMI_API_KEY}
    r = requests.get(url, headers=headers)
    
    resp = r.json()
    
    # TODO: Consider iterating through to find the actual latest. Assumption: Last in the list is the latest
    latest_url = resp['features'][-1]['asset']['data']['href']
    return latest_url
    
def getGribFile(url, DMI_API_KEY):
    filename = url.split('/')[-1]
    headers = {'X-Gravitee-Api-Key': DMI_API_KEY}

    if(os.path.exists(filename)):
        return filename # Terminate early if file already exists
    
    with requests.get(url, headers=headers, stream=True) as req:
        req.raise_for_status()
        with open(filename, 'wb') as fd:
            for chunk in req.iter_content(chunk_size=8192):
                fd.write(chunk)
    return filename

def buildDBRecords(type, lats, lons, measures, is_sim = False, sim_layer = 0):
    if(len(lats) != len(lons) or len(lons) != len(measures)):
        print("WTF") # TODO: Throw error here
    
    records = [{"mtype": type, 
                "lat": lats[i], 
                "long": lons[i], 
                "meas": measures[i],
                "sim": is_sim,
                "sim_layer": sim_layer} for i in range(len(lats))]

    return records

#if(__name__ == "__main__"):
#    last = findLatest()
#    grib_file = getGribFile(last)
#    grbs = pygrib.open("./" + grib_file)
#    grbs.seek(0)
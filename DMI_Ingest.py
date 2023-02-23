#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb  2 16:21:13 2023

@author: point
"""
import os

import requests

import pygrib

from sqlalchemy import Table, Column, Float, Integer, String, MetaData, create_engine, insert
from sqlalchemy.orm import sessionmaker
from sqlalchemy.engine import URL

import numpy as np

def findLatest(DMI_API_KEY):
    url = 'https://dmigw.govcloud.dk/v1/forecastdata/collections/harmonie_nea_sf/items'
    headers = {'X-Gravitee-Api-Key': DMI_API_KEY}
    r = requests.get(url, headers=headers)
    
    resp = r.json()
    
    # TODO: Consider iterating through to find the actual latest. Assumption: Last in the list is the latest
    latest_url = resp['features'][-1]['asset']['data']['href']
    return latest_url

def findBefore(DMI_API_KEY, last_url):
    url = 'https://dmigw.govcloud.dk/v1/forecastdata/collections/harmonie_nea_sf/items'
    headers = {'X-Gravitee-Api-Key': DMI_API_KEY}
    r = requests.get(url, headers=headers)

    resp = r.json()   
    feat_list = resp['features']

    hrefs = [x['asset']['data']['href'] for x in feat_list]
    idx = hrefs.index(last_url)
    
    before_url = feat_list[idx-1]['asset']['data']['href']

    if(before_url.split("_")[-2] == last_url.split("_")[-2]): # If model run is the same
        return before_url
    return None
    
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

def buildDBRecords(lats, lons, measures, dts, is_sim = False, sim_layer = 0):
    records = [{"geom": f'POINT({lons[i]} {lats[i]})',
                "precip_meas": measures['precip'][i],
                "winddir_meas": measures['winddir'][i],
                "windspd_meas": measures['windspd'][i],
                "dt": dts,
                "sim": is_sim,
                "sim_layer": sim_layer} for i in range(len(lats))]

    return records

global DMI_idxs
DMI_idxs = [
    {'idx': 42, 'mtype': 'precip'},
    {'idx': 87, 'mtype': 'winddir'},
    {'idx': 88, 'mtype': 'windspd'}
]

def updateDMIData(DMI_API_KEY, engine, meta, dbsm):
    # TODO: Get all datapoint from last modelrun
    last = findLatest(DMI_API_KEY)
    grib_file = getGribFile(last, DMI_API_KEY)

    grbs = pygrib.open(f"./{grib_file}")

    time_str = (" ").join(grib_file.split("_")[-1].split(".")[0][:-1].split("T"))
    time_str = time_str[:-4] + ":" + time_str[-4:-2] + ":" + time_str[-2:]
    
    values = {}
    
    for DMI_idx in DMI_idxs:
        grb = grbs[DMI_idx['idx']]

        values[DMI_idx['mtype']] = np.array(grb['values']).reshape(-1)
        
        if("(accum)" in grb.__str__()):
            before = findBefore(DMI_API_KEY, last)

            if(before):
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

#if(__name__ == "__main__"):
#    last = findLatest()
#    grib_file = getGribFile(last)
#    grbs = pygrib.open("./" + grib_file)
#    grbs.seek(0)
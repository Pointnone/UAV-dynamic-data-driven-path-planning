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

from sqlalchemy import Table, Column, Float, Integer, String, MetaData, create_engine, insert
from sqlalchemy.orm import sessionmaker
from sqlalchemy.engine import URL

from datetime import datetime

import numpy as np

DMI_TS_FORMAT = "%Y-%m-%dT%H%M%S"

def findLatest(DMI_API_KEY):
    url = 'https://dmigw.govcloud.dk/v1/forecastdata/collections/harmonie_nea_sf/items'
    headers = {'X-Gravitee-Api-Key': DMI_API_KEY}
    r = requests.get(url, headers=headers)
    
    resp = r.json()
    
    # TODO: Consider iterating through to find the actual latest. Assumption: Last in the list is the latest
    feat_list = resp['features']
    hrefs = [x['asset']['data']['href'] for x in feat_list]

    model_runs = np.asarray([datetime.strptime(mr[:-1], DMI_TS_FORMAT) for mr in [u.split("_")[-2] for u in hrefs]])
    latest_mr = model_runs.max(); latest_mr = datetime.strftime(latest_mr, DMI_TS_FORMAT)

    idxs = [i for i, href in enumerate(hrefs) if latest_mr == href.split("_")[-2][:-1]]
    ret_hrefs = [ href for i, href in enumerate(hrefs) if i in idxs ]
    last_idx = idxs[-1]

    latest_url = resp['features'][last_idx]['asset']['data']['href']
    return latest_url, ret_hrefs

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

def buildDBRecords(data, dts, is_sim = False, sim_layer = 0):
    find_idx = lambda mtype : [idx for idx,v in enumerate(DMI_idxs) if v['mtype'] == mtype][0] + 2 # +2 lat / lon offset
    records = [{"geom": f'POINT({data[i][1]} {data[i][0]})',
                "precip_meas": data[i][find_idx('precip')],
                "winddir_meas": data[i][find_idx('winddir')],
                "windspd_meas": data[i][find_idx('windspd')],
                "dt": dts,
                "sim": is_sim,
                "sim_layer": sim_layer} for i in range(len(data))]

    return records

global DMI_idxs
DMI_idxs = [
    {'idx': 42, 'mtype': 'precip'},
    {'idx': 87, 'mtype': 'winddir'},
    {'idx': 88, 'mtype': 'windspd'}
]

def updateDMIData(DMI_API_KEY, engine, meta, dbsm):
    last, all_of_last_mr = findLatest(DMI_API_KEY)
    print(f'Getting {len(all_of_last_mr)} records')
    #all_of_last_mr = ["https://dmigw.govcloud.dk/v1/forecastdata/download/HARMONIE_NEA_SF_2023-03-27T060000Z_2023-03-27T060000Z.grib"]
    for href in all_of_last_mr:
        #print(href)
        grib_file = getGribFile(href, DMI_API_KEY)

        grbs = pygrib.open(f"./{grib_file}")

        time_str = (" ").join(grib_file.split("_")[-1].split(".")[0][:-1].split("T"))
        time_str = time_str[:-4] + ":" + time_str[-4:-2] + ":" + time_str[-2:]
        
        values = {}
        before = findBefore(DMI_API_KEY, href)
        
        if(not before): # First measurements seem broken
            continue
        
        for DMI_idx in DMI_idxs:
            grb = grbs[DMI_idx['idx']]

            values[DMI_idx['mtype']] = np.array(grb['values']).reshape(-1)
            
            if("(accum)" in grb.__str__()):
                before_grib_file = getGribFile(before, DMI_API_KEY)
                before_grbs = pygrib.open(f"./{before_grib_file}")
                before_grb = before_grbs[DMI_idx['idx']]
                before_values = np.array(before_grb['values']).reshape(-1)
                values[DMI_idx['mtype']] -= before_values # Element-wise list subtraction
        
        lats, lons = grbs[42].latlons()
        lats = np.array(lats).reshape(-1)
        lons = np.array(lons).reshape(-1)

        data = np.dstack((lats, lons))
        for DMI_idx in DMI_idxs:
            data = np.dstack((data, values[DMI_idx['mtype']]))
        data = np.array(data[0])

        # Filter to Denmark only data
        data = np.array([ d for d in data if (d[0] >= 54.0 and d[0] <= 58.0 and d[1] >= 7.0 and d[1] <= 16.0) ])

        with dbsm() as session:
            DMI_Table = Table("dmi", meta, autoload_with=engine) # Table Reflection
            session.execute(insert(DMI_Table), buildDBRecords(data, time_str))
            session.commit()

if(__name__ == "__main__"):
    env_path = Path('./.env')
    load_dotenv(dotenv_path=env_path)

    global DMI_API_KEY; DMI_API_KEY = os.getenv('DMI_API_KEY')
    last = findLatest(DMI_API_KEY)
    grib_file = getGribFile(last)
    grbs = pygrib.open("./" + grib_file)
    grbs.seek(0)
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb  2 16:21:13 2023

@author: point
"""
import os

import requests

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
    # TODO: Take into account that it must be from the same model run
    idx = hrefs.index(last_url)
    #print(hrefs)
    #idx = [i for i, x in enumerate(feat_list) if x['asset']['data']['href'] == url][0]

    before_url = feat_list[idx-1]['asset']['data']['href']
    return before_url
    
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
    #if(len(lats) != len(lons) or len(lons) != len(measures)):
    #    print("WTF") # TODO: Throw error here
    
    records = [{"geom": f'POINT({lons[i]} {lats[i]})',
                "precip_meas": measures['precip'][i],
                "winddir_meas": measures['winddir'][i],
                "windspd_meas": measures['windspd'][i],
                "dt": dts,
                "sim": is_sim,
                "sim_layer": sim_layer} for i in range(len(lats))]

    return records

#if(__name__ == "__main__"):
#    last = findLatest()
#    grib_file = getGribFile(last)
#    grbs = pygrib.open("./" + grib_file)
#    grbs.seek(0)
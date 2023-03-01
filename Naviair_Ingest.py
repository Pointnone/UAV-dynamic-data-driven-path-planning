#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 27 14:33:43 2023

@author: point
"""
import os

import requests

from sqlalchemy import Table, Column, Float, Integer, String, MetaData, create_engine, insert
from sqlalchemy.orm import sessionmaker
from sqlalchemy.engine import URL

import xml.etree.ElementTree as ET

import pyproj as proj
from geojson import Polygon

def buildPostGISSimplePoly(poly):
    coord_wrap = lambda c : (" ").join([str(f) for f in c])
    poly_wrap = lambda p : f'({(", ").join([coord_wrap(c) for c in p])})'

    if(poly["type"] == "MultiPolygon"):
        poly_str = f'POLYGON({(",").join([poly_wrap(polygon) for polygon in poly["coordinates"][0]])})'
    elif(poly["type"] == "Polygon"):
        poly_str = f'POLYGON({(",").join([poly_wrap(polygon) for polygon in poly["coordinates"]])})'

    print(poly_str)

    return poly_str

def buildZoneDBRecords(zones, is_sim = False, sim_layer = 0):
    zrecords = [{"geom": buildPostGISSimplePoly(zones[i]['geometry']),
                "zname": zones[i]["properties"]["name"],
                "ztype": zones[i]["properties"]["type"] if "type" in zones[i]["properties"] else "None",
                "sim": is_sim,
                "sim_layer": sim_layer} for i in range(len(zones))]

    return zrecords

def buildNotamDBRecords(zones, is_sim = False, sim_layer = 0):
    nrecords = [{"zname": z["properties"]["name"],
                "act_from": n["parsedMessage"]["schedule"]["activityStart"],
                "act_to": n["parsedMessage"]["schedule"]["validityEnd"],
                "sim": is_sim,
                "sim_layer": sim_layer} for z in zones for n in z["properties"]["notams"]]

    return nrecords

def json_extract(obj, key):
    # Recursively fetch values from nested JSON. 
    arr = []

    def extract(obj, arr, key):
        if isinstance(obj, dict):
            for k, v in obj.items():
                if k == key:
                    arr.append(v)
                elif isinstance(v, (dict, list)):
                    extract(v, arr, key)
        elif isinstance(obj, list):
            for item in obj:
                extract(item, arr, key)
        return arr

    values = extract(obj, arr, key)
    return values

def updateNaviairData(NAVIAIR_API_KEY, engine, meta, dbsm):
    url = 'https://apim-api-001-utm-prd.azure-api.net/aftn/messages'
    headers = {'Ocp-Apim-Subscription-Key': NAVIAIR_API_KEY}
    r = requests.get(url, headers=headers)

    resp = r.json()
    feat_lists = json_extract(resp, 'features')
    feat_list = [f for fl in feat_lists for f in fl] # Flatten

    #notams_lists = json_extract(resp, 'notams')
    #notams_list = [n for nl in notams_lists for n in nl] # Flatten

    with dbsm() as session:
            Zone_Table = Table("zone", meta, autoload_with=engine) # Table Reflection
            session.execute(insert(Zone_Table), buildZoneDBRecords(feat_list))
            Notam_Table = Table("notam", meta, autoload_with=engine) # Table Reflection
            session.execute(insert(Notam_Table), buildNotamDBRecords(feat_list))
            session.commit()
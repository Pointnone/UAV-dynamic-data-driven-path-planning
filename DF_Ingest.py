#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb  6 14:57:45 2023

@author: point
"""
from pathlib import Path
import os

import ftplib

from zipfile import ZipFile

import xml.etree.ElementTree as ET

import pyproj as proj
from geojson import Polygon

from sqlalchemy import Table, Column, Float, Integer, String, MetaData, create_engine, insert
from sqlalchemy.orm import sessionmaker
from sqlalchemy.engine import URL

def downloadDF(DF_USER, DF_PASS):
    host = "ftp3.datafordeler.dk"
    ftp = ftplib.FTP(host, DF_USER, DF_PASS)
    
    if(len(ftp.nlst()) > 0):
        filename = [f for f in ftp.nlst() if "GML" in f][0]
        
        if(os.path.exists(filename)):
            return filename # Terminate early if file already exists
        
        with open(filename, "wb") as fd:
            ftp.retrbinary(f"RETR {filename}", fd.write)
        
        ftp.quit()
        
        return filename
    return None

def unzip(filename, fileToExtract):
    with ZipFile(Path(f'./{filename}'), 'r') as zipF:
        zipF.extract(fileToExtract)
    zipF.close()

def extractCityShapes(filename):
    t = ET.parse(filename)
    r = t.getroot()
    
    epsg25832 = proj.CRS.from_epsg(25832)
    epsg4326 = proj.CRS.from_epsg(4326)
    transformer = proj.Transformer.from_crs(epsg25832, epsg4326, True)
    
    cities = []
    namespaces = {'fme': 'http://www.safe.com/gml/fme',
                  'gml': 'http://www.opengis.net/gml/3.2'}
    
    for cfm in r.findall("gml:featureMember", namespaces):
        cbb = cfm.find("fme:bebyggelse", namespaces)
        placetype = cbb.find("fme:bebyggelsestype", namespaces).text
        
        if(placetype == "by" and cbb.find("fme:indbyggertal", namespaces).text):
            population = int(cbb.find("fme:indbyggertal", namespaces).text)
            placename = cbb.find("fme:navn_1_skrivemaade", namespaces).text
            shape = [(coord.text)[0:-2].split(" 0 ") for coord in (cbb.iterfind(".//gml:posList", namespaces))][0] # TODO: Fix assumption about it being only one surface
            shape = [c.split(" ") for c in shape]
            
            if(population >= 1000):
                shape = [transformer.transform(c[0], c[1]) for c in shape]            
                cities.append({'name': placename, 'pop': population, 'shape': Polygon([shape])})
    return cities

def buildPostGISSimplePoly(poly):
    coord_wrap = lambda c : (" ").join([str(f) for f in c])
    poly_wrap = lambda p : f'({(", ").join([coord_wrap(c) for c in p])})'
    poly_str = f'POLYGON({(",").join([poly_wrap(polygon) for polygon in poly["coordinates"]])})'

    return poly_str

def buildCityDBRecords(cities, is_sim = False, sim_layer = 0):
    # POLYGON((0 0, 1 0, 1 1, 0 1, 0 0))

    records = [{"geom": buildPostGISSimplePoly(cities[i]['shape']),
                "itype": "city",
                "sim": is_sim,
                "sim_layer": sim_layer} for i in range(len(cities))]

    return records

def updateDFData(DF_USER, DF_PASS, engine, meta, dbsm):
    filename = downloadDF(DF_USER, DF_PASS)
    
    if(filename):
        # TODO: Add other sources?
        unzip(filename, "bebyggelse.gml")
        cities = extractCityShapes("bebyggelse.gml")

        with dbsm() as session:
            DF_Table = Table("df", meta, autoload_with=engine) # Table Reflection
            session.execute(insert(DF_Table), buildCityDBRecords(cities))
            session.commit()
    
#if(__name__ == "__main__"):
#    filename = downloadDF()
#    unzip(filename, "bebyggelse.gml")
#    cities = extractCityShapes("bebyggelse.gml")
#    print(len(cities))
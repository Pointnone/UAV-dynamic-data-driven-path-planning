#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb  6 14:57:45 2023

@author: point
"""
from dotenv import load_dotenv
from pathlib import Path
import os

import ftplib

from zipfile import ZipFile

import xml.etree.ElementTree as ET

import pyproj as proj
from geojson import Polygon

env_path = Path('./.env')
load_dotenv(dotenv_path=env_path)

host = "ftp3.datafordeler.dk"
DF_USER = os.getenv('DF_USER')
DF_PASS = os.getenv('DF_PASS')

def downloadDF():
    ftp = ftplib.FTP(host, DF_USER, DF_PASS)
    
    filename = [f for f in ftp.nlst() if "GML" in f][0]
    
    if(os.path.exists(filename)):
        return filename # Terminate early if file already exists
    
    with open(filename, "wb") as fd:
        ftp.retrbinary(f"RETR {filename}", fd.write)
    
    ftp.quit()
    
    return filename

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
    
    for cfm in r.findall("gml:featureMembeScreenshot from 2023-02-07 14-38-30r", namespaces):
        cbb = cfm.find("fme:bebyggelse", namespaces)
        placetype = cbb.find("fme:bebyggelsestype", namespaces).text
        
        if(placetype == "by"):
            population = int(cbb.find("fme:indbyggertal", namespaces).text)
            placename = cbb.find("fme:navn_1_skrivemaade", namespaces).text
            shape = [coord.split(" ") for coord in ((cbb.find("gml:multiSurfaceProperty/gml:MultiSurface/gml:surfaceMember/gml:Surface/gml:patches/gml:PolygonPatch/gml:exterior/gml:LinearRing/gml:posList", namespaces).text)[0:-2].split(" 0 "))] # TODO: Fix assumption about it being only one surface
            
            if(population >= 200):
                #print(placename, population)
                shape = [transformer.transform(c[0], c[1]) for c in shape]            
                cities.append({'name': placename, 'pop': population, 'shape': Polygon([shape])})
    return cities
    
filename = downloadDF()
unzip(filename, "bebyggelse.gml")
cities = extractCityShapes("bebyggelse.gml")

print([c for c in cities if c['name'] == "Solrød Strand"][0])
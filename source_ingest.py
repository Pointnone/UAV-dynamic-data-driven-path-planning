from DF_Ingest import *
from DMI_Ingest import *
from TDC_Ingest import *
from Naviair_Ingest import *

from ros_ws.src.command.command.utils import environmentVars, setupDB

if(__name__ == "__main__"):
    env = environmentVars()
    engine, meta, dbsm = setupDB(env['DB_USER'], env['DB_PASS'])
    updateTDCData(engine, meta, dbsm)
    updateNaviairData(env['NAVIAIR_API_KEY'], engine, meta, dbsm)
    updateDFData(env['DF_USER'], env['DF_PASS'], engine, meta, dbsm)
    updateDMIData(env['DMI_API_KEY'], engine, meta, dbsm)
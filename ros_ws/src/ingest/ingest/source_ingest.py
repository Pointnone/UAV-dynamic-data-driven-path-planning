import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from .DF_Ingest import *
from .DMI_Ingest import *
from .TDC_Ingest import *
from .Naviair_Ingest import *

from ....src.command.command.utils import environmentVars, setupDB

class IngestService(Node):
    def __init__(self):
        super().__init__('ingest_service')
        
        self.publisher_ = self.create_publisher(String, 'ingest/update', 10)
        timer_period = 900  # seconds
        self.timer = self.create_timer(timer_period, self.ingest)

        self.env = environmentVars()
        self.engine, self.meta, self.dbsm = setupDB(self.env['DB_USER'], self.env['DB_PASS'])

        self.ingest()

    def ingest(self):
        print("Doing ingest")
        new_inf = True # TODO: Return bool from each update if new information is available
        try:
            new_inf_TDC = updateTDCData(self.engine, self.meta, self.dbsm)
            new_inf_Naviair = updateNaviairData(self.env['NAVIAIR_API_KEY'], self.engine, self.meta, self.dbsm)
            new_inf_DF = updateDFData(self.env['DF_USER'], self.env['DF_PASS'], self.engine, self.meta, self.dbsm)
            #new_inf_DMI = updateDMIData(self.env['DMI_API_KEY'], self.engine, self.meta, self.dbsm)
            #new_inf = new_inf_TDC or new_inf_Naviair or new_inf_DF or new_inf_DMI
        except:
            print("Caught exception")
            # noop = True # TODO: Consider cleaning up in case something fails and catching each individually to localize clean-up efforts
        
        #print("Checking new_inf")
        if(new_inf):
            #print("Got in new_inf check")
            msg = String()
            msg.data = "New information available"
            self.publisher_.publish(msg)

if(__name__ == "__main__"):
    env = environmentVars()
    engine, meta, dbsm = setupDB(env['DB_USER'], env['DB_PASS'])
    #updateTDCData(engine, meta, dbsm)
    #updateNaviairData(env['NAVIAIR_API_KEY'], engine, meta, dbsm)
    #updateDFData(env['DF_USER'], env['DF_PASS'], engine, meta, dbsm)
    #updateDMIData(env['DMI_API_KEY'], engine, meta, dbsm)

    rclpy.init()
    ingest_service = IngestService()
    rclpy.spin(ingest_service)
    ingest_service.destroy_node()
    rclpy.shutdown()

    #init()
    #updateNaviairData(env['NAVIAIR_API_KEY'], engine, meta, dbsm)
    #updateDFData(env['DF_USER'], env['DF_PASS'], engine, meta, dbsm)
    #updateDMIData(env['DMI_API_KEY'], engine, meta, dbsm)
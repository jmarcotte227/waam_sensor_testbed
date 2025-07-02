import sys
import os
import time
import numpy as np
import datetime
import traceback

from RobotRaconteur.Client import *

sys.path.append("utils/")
from welder import WeldTrigger
from stepcraft_api import UC100Controller
from utils import estop_all
from xiris_api import XirisInterface
from SensorSuite import SensorSuite
# import SensorSuite

if __name__ == "__main__":
    ###### FILEPATHS ######
    stepcraft_dll_path = "C:/UCCNC/API/DLL/UC100.dll"
    config_file = "utils/Stepcraft2_Model420.pro"
    data_dir = "../wst_data/"


    ###### LOAD DATA ######
    pathplan = np.loadtxt("path_generation/paths/wall.csv", delimiter = ',')

    ###### EXPERIMENT DIR ######
    desc = input("Enter experiment ID: ")
    filename = desc+datetime.datetime.now().strftime("%Y%m%d-%H%M%S/")
    os.mkdir(data_dir+filename)

    device_list = []
    try:
        ###### INITIALIZE HARDWARE DEVICES ######
        # weld trigger
        print("Setting trigger")
        welder = WeldTrigger()
        device_list.append(welder)

        # stepcraft
        print("connecting to stepcraft")
        uc = UC100Controller(stepcraft_dll_path)
        num_devices = uc.list_devices()
        print(f"Found {num_devices} device(s)")
        uc.open_device(1)
        uc.load_config(config_file)
        device_list.append(uc)

        # xiris
        xiris = XirisInterface()
        xiris.set_filename(filename)
        xiris.set_overlay(filename)
        device_list.append(xiris)

        # Microphone
        # mic_ser = RRN.ConnectService('rr+tcp://localhost:60828?service=microphone')
        mic_ser = None
        # Spectrometer
        spec_ser = RRN.ConnectService('rr+tcp://localhost:60825?service=spectrometer')
        # Sensor Suite for RR sensors
        rr_sensors = SensorSuite(microphone_service = mic_ser, spec_service = spec_ser, spec_freq = 5)

        ###### INITIALIZE PROCESS ######
        uc.jog_to_zero()
        uc.init_pathplan(pathplan)

        ###### RUN PROCESS ######
        input("Enter to weld")
        # starting weld
        xiris.start_recording()
        rr_sensors.start_all_sensors()
        welder.weld_on()
        #run path with blocking
        uc.run_pathplan()

        # stopping weld
        welder.weld_off()
        xiris.stop_recording()
        rr_sensors.stop_all_sensors()
        rr_sensors.save_all_sensors(data_dir+filename)

    except (Exception, KeyboardInterrupt) as e:
        estop_all(device_list)
        # print(traceback.format_exc())
        raise e
        

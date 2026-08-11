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

if __name__ == "__main__":
    ###### FILEPATHS ######
    stepcraft_dll_path = "C:/UCCNC/API/DLL/UC100.dll"
    config_file = "utils/Stepcraft2_Model420.pro"
    data_dir = "../wst_data/"

    ###### LOAD DATA ######
    pathplan = np.loadtxt("path_generation/paths/wall.csv", delimiter = ',')

    ###### PROCESS PATHPLAN ###### 
    jump_vel = pathplan[0,-1]

    jumps = np.squeeze(np.argwhere(pathplan[:,-1]==100))
    pathplan_split =[]

    # append the first line
    pathplan_split.append(pathplan[:jumps[1],:])

    for idx, _ in enumerate(jumps[1:]):
        try:
            pathplan_split.append(pathplan[jumps[idx+1]:jumps[idx+2], :])
        except IndexError:
            pathplan_split.append(pathplan[jumps[idx+1]:, :])

    ###### EXPERIMENT DIR ######
    device_list = []
    print(pathplan_split)
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

        ###### Jogging to zero
        uc.jog_to_zero()
        input("Enter to weld")

        for path in pathplan_split:
            ###### INITIALIZE PROCESS ######
            uc.init_pathplan(path)

            ###### RUN PROCESS ######
            welder.weld_on()
            #run path with blocking
            uc.run_pathplan()
            welder.weld_off()
            time.sleep(5)

    except (Exception, KeyboardInterrupt) as e:
        estop_all(device_list)
        print(traceback.format_exc())

    finally:
        # stopping weld
        welder.weld_off()

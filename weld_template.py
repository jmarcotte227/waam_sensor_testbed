import sys
import time
import numpy as np

sys.path.append("utils/")
from welder import WeldTrigger
from stepcraft_api import UC100Controller
from utils import estop_all
# import SensorSuite

if __name__ == "__main__":
    ###### FILEPATHS ######
    stepcraft_dll_path = "C:/UCCNC/API/DLL/UC100.dll"
    config_file = "utils/Stepcraft2_Model420.pro"


    ###### LOAD DATA ######
    pathplan = np.loadtxt("path_generation/paths/wall.csv", delimiter = ',')

    device_list = []
    try:
        ###### INITIALIZE HARDWARE DEVICES ######
        # weld trigger
        welder = WeldTrigger()
        device_list.append(welder)

        # stepcraft
        uc = UC100Controller(stepcraft_dll_path)
        num_devices = uc.list_devices()
        print(f"Found {num_devices} device(s)")
        uc.open_device(1)
        uc.load_config(config_file)
        device_list.append(uc)

        ###### INITIALIZE PROCESS ######
        uc.jog_to_zero()
        uc.init_pathplan(pathplan)

        ###### RUN PROCESS ######
        input("Enter to weld")
        welder.weld_on()
        uc.run_pathplan()
        time.sleep(1)
        x = 1/0
        welder.weld_off()

    except (Exception, KeyboardInterrupt) as e:
        estop_all(device_list)
        raise e
        

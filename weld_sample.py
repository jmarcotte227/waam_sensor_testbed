import sys
import glob
import yaml
import numpy as np

sys.path.append("utils/")
sys.path.append("")
from RobotRaconteur.Client import *
from run_testbed import *
import matplotlib.pyplot as plt
from datetime import datetime

# directories
path_plan = 'path_generation/wall.csv'
save_dir = 'spec_testing/'

# setup sensor services
spec_ser = RRN.ConnectService("rr+tcp://localhost:60825?service=spectrometer")
sensors = SensorSuite(spec_service = spec_ser, spec_freq = 10)

sensors.start_all_sensors()
time.sleep(30)
sensors.stop_all_sensors()
sensors.save_all_sensors(save_dir)


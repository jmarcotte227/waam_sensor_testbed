import ctypes
import configparser
import keyboard
import time
import numpy as np

class Stat(ctypes.Structure):
    _fields_ = [
            ("Idle", ctypes.c_bool),
            ("Jog", ctypes.c_bool),
            ("Dwell", ctypes.c_bool),
            ("Backlash", ctypes.c_bool),
            ("Home", ctypes.c_bool),
            ("Probe", ctypes.c_bool),
            ("Estop", ctypes.c_bool),
            ("SoftLimit", ctypes.c_bool),
            ("HardLimit", ctypes.c_bool),
            ("Puffer", ctypes.c_int),
            ("Feed", ctypes.c_double),
            ("SpindleRPM", ctypes.c_double),
            ("CurrentID", ctypes.c_int),
            ("LimitOverride", ctypes.c_bool),
            ("ProgrammedFeed", ctypes.c_double),
            ("MPG1JogOn", ctypes.c_bool),
            ("MPG2JogOn", ctypes.c_bool),
            ("THCOnWaiting", ctypes.c_bool),
            ("SyncThread", ctypes.c_bool),
            ("SpindleOn", ctypes.c_bool),
            ("SpindleDir", ctypes.c_bool),
            ("LaserRunning", ctypes.c_bool),
            ("LDataValid", ctypes.c_bool),
            ("THCOn", ctypes.c_bool),
            ("THCAntiDive", ctypes.c_bool),
            ("THCAntiDiveEnable", ctypes.c_bool),
            ("THCDelayEnable", ctypes.c_bool),
            ("THCAntiDownEnable", ctypes.c_bool),
            ("ProbeActive", ctypes.c_bool),
            ]

class AxisSetting(ctypes.Structure):
    _fields_ = [
            ("Axis",ctypes.c_int),			#Axis number (X=0,Y=1,Z=2,A=3,B=4,C=5).
            ("Enable",ctypes.c_bool),			#Enables the axis.
            ("StepPin",ctypes.c_int),		#Step output pin.
            ("DirPin",ctypes.c_int),			#Direction output pin.
            ("StepNeg",ctypes.c_bool),		#Inverts the step pin.
            ("DirNeg",ctypes.c_bool),			#Inverts the dir pin.
            ("MaxAccel",ctypes.c_double),		#Acceleration parameter of the axis in Units/sec^2.
            ("MaxVel",ctypes.c_double),			#Velocity parameter of the axis in Units/sec.
            ("StepPer",ctypes.c_double),		#Steps per Units parameter.
            ("HomePin",ctypes.c_int),		#Home input pin of the axis.
            ("HomeNeg",ctypes.c_bool),		#Inverts the home input.
            ("LimitPPin",ctypes.c_int),		#Positive side end-limit input pin.
            ("LimitPNeg",ctypes.c_bool),		#Inverts the positive side end-limit input.
            ("LimitNPin",ctypes.c_int),		#Negative side end-limit input pin.
            ("LimitNNeg",ctypes.c_bool),		#Inverts the negative side end-limit input.
            ("SoftLimitP",ctypes.c_double),		#Positive side software end-limit.
            ("SoftLimitN",ctypes.c_double),		#Negative side software end-limit.
            #Slave axis, only for the XYZ axes. (0=No slave, 3=A slave, 4=B slave, 5=C slave).
            ("SlaveAxis",ctypes.c_int),
            ("BacklashOn",ctypes.c_bool),		#Enables the backlash compensation for the axis.
            ("BacklashDist",ctypes.c_double),	#Backlash distance in Units.
            #Compensation acceleration for backlash and thread cutting in Units/sec^2.
            ("CompAccel",ctypes.c_double),
            ("EnablePin",ctypes.c_int),		#Axis enable output pin.
            ("EnablePinNeg",ctypes.c_bool),	#Inverts the axis enable output.
            ("EnableDelay",ctypes.c_int),	#Delays the enable output 0-255 value (x10msec).
            ("CurrentHiLowPin",ctypes.c_int),	#Current hi/low output pin.
            ("CurrentHiLowPinNeg",ctypes.c_bool),	#Inverts the current hi/low output.
            ("HomeBackOff",ctypes.c_double),	#Home back off distance in Units.
            #Enables the rotary axis function for the axis. Works for the A,B,C axes only.
            ("RotaryAxis",ctypes.c_bool),
            #Enables the rollover for rotary axis on 360 degrees. 
            #Works for the A,B,C axes only and if the rotary axis function is enabled.
            ("RotaryRollover",ctypes.c_bool),
            ]

class UC100Controller:
    '''
    UC100 Controller Class to interface with the Stepcraft CNC.
    '''

    DEVICE_TYPE_MAP = {
        -19: "Demo_AXBB",
        -18: "Demo_UC300ETH_UB1",
        -17: "Demo_UC300ETH_M45",
        -16: "Demo_UC300ETH_5441",
        -15: "Demo_UC300_5441",
        -14: "Demo_UC300ETH_STEPPER",
        -13: "Demo_UC300ETH_ISOBOB",
        -12: "Demo_UC300ETH_M44",
        -11: "Demo_UC300ETH_5LPT",
        -10: "Demo_UC300ETH_Low",
        -9: "Demo_UC300ETH_Hi",
        -8: "Demo_UC400ETH",
        -7: "Demo_UC300_STEPPER",
        -6: "Demo_UC300_ISOBOB",
        -5: "Demo_UC300_M44",
        -4: "Demo_UC300_5LPT",
        -3: "Demo_UC300_Low",
        -2: "Demo_UC300_Hi",
        -1: "Demo_UC100",
         0: "Demo_mode",
         1: "UC100",
         2: "UC300_Hi",
         3: "UC300_Low",
         4: "UC300_5LPT",
         5: "UC300_M44",
         6: "UC300_ISOBOB",
         7: "UC300_STEPPER",
         8: "UC400ETH",
         9: "UC300ETH_Hi",
        10: "UC300ETH_Low",
        11: "UC300ETH_5LPT",
        12: "UC300ETH_M44",
        13: "UC300ETH_ISOBOB",
        14: "UC300ETH_STEPPER",
        15: "UC300_5441",
        16: "UC300ETH_5441",
        17: "UC300ETH_M45",
        18: "UC300ETH_UB1",
        19: "AXBB"
    }

    AXIS_MAP = {
        'X': 0,
        'Y': 1,
        'Z': 2,
        'A': 3,
        'B': 4,
        'C': 5
    }
    SLAVE_AXIS_MAP = {
        'None': 0,
        'A': 1,
        'B': 2,
        'C': 3
    }

    def __init__(self, dll_path):
        self.dll = ctypes.WinDLL(dll_path)

        # Function prototypes
        self.dll.ListDevices.argtypes = [ctypes.POINTER(ctypes.c_int)]
        self.dll.ListDevices.restype = ctypes.c_int

        self.dll.DeviceInfo.argtypes = [ctypes.c_int,
                                        ctypes.POINTER(ctypes.c_int),
                                        ctypes.POINTER(ctypes.c_int)]
        self.dll.DeviceInfo.restype = ctypes.c_int

        self.dll.Open.argtypes = [ctypes.c_int]
        self.dll.Open.restype = ctypes.c_int

        self.dll.Close.argtypes = []
        self.dll.Close.restype = ctypes.c_int

        self.dll.SetStepRate.argtypes = [ctypes.c_int]
        self.dll.SetStepRate.restype = ctypes.c_int

        self.dll.GetStepRate.argtypes = [ctypes.POINTER(ctypes.c_int)]
        self.dll.GetStepRate.restype = ctypes.c_int

        self.dll.GetInput.argtypes = [ctypes.POINTER(ctypes.c_int)]
        self.dll.GetStepRate.restype = ctypes.c_int

        self.dll.GetStatus.argtypes = [ctypes.POINTER(Stat)]
        self.dll.GetStatus.restype = ctypes.c_int

        self.dll.SetAxisSetting.argtypes = [ctypes.POINTER(AxisSetting)]
        self.dll.SetAxisSetting.restype = ctypes.c_int

        self.dll.GetAxisSetting.argtypes = [ctypes.POINTER(AxisSetting)]
        self.dll.GetAxisSetting.restype = ctypes.c_int

        self.dll.SetEstopSetting.argtypes = [ctypes.c_int, ctypes.c_bool]
        self.dll.SetEstopSetting.restype = ctypes.c_int

        self.dll.SpindleOn.argtypes = [ctypes.c_bool]
        self.dll.SpindleOn.restype = ctypes.c_int

        self.dll.SpindleOff.argtypes = []
        self.dll.SpindleOff.restype = ctypes.c_int

        self.dll.AddLinearMoveRel.argtypes = [ctypes.c_int,
                                              ctypes.c_double,
                                              ctypes.c_int,
                                              ctypes.c_double,
                                              ctypes.c_bool]
        self.dll.AddLinearMoveRel.restype = ctypes.c_int

        self.dll.AddLinearMove.argtypes = [ctypes.c_double,
                                           ctypes.c_double,
                                           ctypes.c_double,
                                           ctypes.c_double,
                                           ctypes.c_double,
                                           ctypes.c_double,
                                           ctypes.c_double,
                                           ctypes.c_int]
        self.dll.AddLinearMove.restype = ctypes.c_int

        self.dll.JogOnSpeed.argtypes = [ctypes.c_int,
                                        ctypes.c_bool,
                                        ctypes.c_double]
        self.dll.JogOnSpeed.restype = ctypes.c_int

        self.dll.SetAxisPosition.argtypes = [ctypes.c_double,
                                             ctypes.c_double,
                                             ctypes.c_double,
                                             ctypes.c_double,
                                             ctypes.c_double,
                                             ctypes.c_double]
        self.dll.SetAxisPosition.restype = ctypes.c_int

        self.dll.SetFeedHold.argtypes = [ctypes.c_bool]
        self.dll.SetFeedHold.restype = ctypes.c_int

        self.dll.Stop.argtypes = []
        self.dll.Stop.restype = ctypes.c_int

        self.dll.GetEstopCause.argtypes = [ctypes.POINTER(ctypes.c_int)]
        self.dll.GetEstopCause.restype = ctypes.c_int

        self.dll.SetEstopState.argtypes = []
        self.dll.SetEstopState.restype = ctypes.c_int

        self.connected = False
        self.device_id = None
        self.jog_speed = 1


    def list_devices(self):
        count = ctypes.c_int()
        result = self.dll.ListDevices(ctypes.byref(count))
        if result != 0:
            raise RuntimeError("ListDevices failed with code {}".format(result))
        return count.value

    def get_device_info(self, board_id):
        device_type = ctypes.c_int()
        serial = ctypes.c_int()
        result = self.dll.DeviceInfo(board_id, ctypes.byref(device_type), ctypes.byref(serial))
        if result != 0:
            raise RuntimeError(f"DeviceInfo failed for board {board_id} with code {result}")
        return {
            "board_id": board_id,
            "type": device_type.value,
            "serial": serial.value
        }

    def open_device(self, board_id=0):
        result = self.dll.Open(board_id)
        if result != 0:
            raise RuntimeError(f"Failed to open device {board_id}, error code {result}")
        self.connected = True
        self.device_id = board_id

        # use stop to clear current state
        # result = self.dll.Stop()
        # if result != 0:
        #     if result == 7: 
        #         cause = ctypes.c_int()
        #         _ = self.dll.GetEstopCause(ctypes.byref(cause))
        #         print("Estop Cause: ", cause)
        #     raise RuntimeError("Stop failed with code {}".format(result))

    def load_config(self, config_file):
        # parse config
        if config_file is not None:
            config = configparser.ConfigParser()
            config.optionxform = str  # Preserve key case
            config.read(config_file)

            # load estop config
            estop_pin = int(config['IOsetupsettings']['Estoppinnumber'])
            estop_negate = eval(config['IOsetupsettings']['Estoppinnegate'])

            # set estop pin
            # result = self.dll.SetEstopSetting(estop_pin, estop_negate)
            # if result != 0:
            #     if result == 7: 
            #         cause = ctypes.c_int()
            #         _ = self.dll.GetEstopCause(ctypes.byref(cause))
            #         print("Estop Cause: ", cause)
            #     raise RuntimeError(f"Setting Estop pin failed with code {result}")


            self.axis_settings = {}
            for section in config.sections():
                if section.startswith("axessettingscontrol"):
                    axis = section[len("axessettingscontrol"):]  # e.g., "X"
                    self.axis_settings[axis] = dict(config[section])

        # Load Config into struct and upload to machine
        for key in self.axis_settings.keys():
            setting = self._parse_axis_settings(key)
            # set the axis setting
            # self._get_axis_setting()
            result = self.dll.SetAxisSetting(ctypes.byref(setting))
            if result != 0:
                raise RuntimeError("Setting axis setting failed with code{}".format(result))

    def close_device(self):
        result = self.dll.Close()
        if result != 0:
            raise RuntimeError("Failed to close device, error code {}".format(result))
        self.connected = False
        self.device_id = None

    def get_status(self):
        _status = Stat()
        result = self.dll.GetStatus(ctypes.byref(_status))
        if result != 0:
            raise RuntimeError(f"GetStatus failed with error code {result}")

        return _status

    def spindle_on(self):
        result = self.dll.SpindleOn(True)
        if result != 0:
            raise RuntimeError(f"SpindleOn failed with error code {result}")

    def spindle_off(self):
        result = self.dll.SpindleOff()
        if result != 0:
            raise RuntimeError(f"SpindleOff failed with error code {result}")

    def linear_move_relative(self,
                             axis,
                             step,
                             step_count,
                             speed,
                             direction):
        axis = int(axis)
        step = float(step)
        step_count = int(step_count)
        speed = float(speed)
        direction = bool(direction)

        result = self.dll.AddLinearMoveRel(axis,
                                           step,
                                           step_count,
                                           speed,
                                           direction)
        if result != 0:
            raise RuntimeError(f"AddLinearMoveRel failed with error code {result}")

    def linear_move(self,
                    x,
                    y,
                    z,
                    feed,
                    ID,
                    a=0.0,
                    b=0.0,
                    c=0.0):
        result = self.dll.AddLinearMove(x,
                                        y,
                                        z,
                                        a,
                                        b,
                                        c,
                                        feed,
                                        ID)
        if result != 0:
            raise RuntimeError(f"AddLinearMove failed with error code {result}")

    def set_axis_pos(self,
                     x_pos,
                     y_pos,
                     z_pos,
                     a_pos=0.0,
                     b_pos=0.0,
                     c_pos=0.0):
        result = self.dll.SetAxisPosition(x_pos,
                                          y_pos,
                                          z_pos,
                                          a_pos,
                                          b_pos,
                                          c_pos)
        if result != 0:
            raise RuntimeError(f"SetAxisPosition failed with error code {result}")

    def set_feedhold(self,state):
        result = self.dll.SetFeedHold(state)
        if result != 0:
            raise RuntimeError(f"SetFeedhold failed with error code {result}")

    def zero_axes(self):
        # zeros all axes
        self.set_axis_pos(0.0,0.0,0.0)

    def init_pathplan(self, plan):
        '''
        Function to convert generated pathplan into motion commands. Uses absolute move
        commands after zeroing at current position.

        plan = [[x, y, z, vel],
                [x, y, z, vel],
                 ...
                [x, y, z, vel]]
        '''
        self.zero_axes()
        mot_id = 0
        # send to start
        self.linear_move(plan[0,0],
                         plan[0,1],
                         plan[0,2],
                         plan[0,3],
                         mot_id)

        # increment motion id
        mot_id+=1

        input("Press Enter when motion stops")

        # hold motion
        self.set_feedhold(True)

        # buffer all commands
        for i in range(1,plan.shape[0]):
            self.linear_move(plan[i,0],
                             plan[i,1],
                             plan[i,2],
                             plan[i,3],
                             mot_id)
            mot_id += 1

    def run_pathplan(self, blocking=True):
        # undo the feedhold to start motion
        self.set_feedhold(False)

        if blocking:
            while True:
                if getattr(self.get_status(), "Idle"):
                    break
            
    def jog_axis(self, axis, direction):
        result = self.dll.JogOnSpeed(axis, direction, self.jog_speed)
        if result != 0:
            raise RuntimeError(f"SetFeedhold failed with error code {result}")
        # self.linear_move_relative(axis,
        #                           1,
        #                           1, # stepcount always 1
        #                           self.jog_speed,
        #                           direction)
    def stop_jog(self, axis):
        result = self.dll.JogOnSpeed(axis, False, 0)
        if result != 0:
            raise RuntimeError(f"SetFeedhold failed with error code {result}")

    def jog_to_zero(self):
        ''' 
        enables jogging with keyboard until at zero
        '''
        key_map = {
                0: ('a', 'd'),
                1: ('w', 's'),
                2: ('q', 'e')
                }

        try:
            print("Jogging active. Press Enter to set zero.")
            while True:
                for axis, (pos_key, neg_key) in key_map.items():
                    pos_pressed = keyboard.is_pressed(pos_key)
                    neg_pressed = keyboard.is_pressed(neg_key)

                    if pos_pressed and not neg_pressed:
                        self.jog_axis(axis, True)
                    elif neg_pressed and not pos_pressed:
                        self.jog_axis(axis, False)
                    else:
                        self.stop_jog(axis)

                if keyboard.is_pressed('enter'):
                    print("Exiting jog control.")
                    break
                if keyboard.is_pressed('z'):
                    self.jog_speed -= 1
                    if self.jog_speed<=0:
                        self.jog_speed = 1
                if keyboard.is_pressed('c'):
                    self.jog_speed += 1

                time.sleep(0.05)

            # set zero
            self.zero_axes()
            print("Zero Set")


        except KeyboardInterrupt:
            print("Jog loop interrupted.")

    def e_stop(self):
        result = self.dll.Stop()
        if result != 0:
            raise RuntimeError(f"Stop failed with error code {result}")

    def _parse_axis_settings(self, axis):
        axis_opts = self.axis_settings[axis]
        setting_struct = AxisSetting(
                Axis = int(self.AXIS_MAP[axis]),
                Enable = eval(axis_opts["Axisenabled"]),
                StepPin = int(axis_opts["Steppinnumber"]),
                DirPin = int(axis_opts["Dirpinnumber"]),
                StepNeg = eval(axis_opts["Steppinnegate"]),
                DirNeg = eval(axis_opts["Dirpinnegate"]),
                MaxAccel = float(axis_opts["Acceleration"]),
                MaxVel = float(axis_opts["Velocity"]),
                StepPer = float(axis_opts["Stepsperunit"]),
                HomePin = int(axis_opts["Homepinnumber"]),
                HomeNeg = eval(axis_opts["Homepinnegate"]),
                LimitPPin = int(axis_opts["Limitpluspinnumber"]),
                LimitPNeg = eval(axis_opts["Limitpluspinnegate"]),
                LimitNPin = int(axis_opts["Limitminuspinnumber"]),
                LimitNNeg = eval(axis_opts["Limitminuspinnegate"]),
                SoftLimitP = float(axis_opts["Softlimitpositive"]),
                SoftLimitN = float(axis_opts["Softlimitnegative"]),
                SlaveAxis = 0, # no slave axes are in use
                BacklashOn = eval(axis_opts["Backlashenable"]),
                BacklashDist = float(axis_opts["Backlashdistance"]),
                CompAccel = float(axis_opts["Compaccel"]),
                EnablePin = int(axis_opts["Enapinnumber"]),
                EnablePinNeg = eval(axis_opts["Enapinnegate"]),
                EnableDelay = 0, #May need to modify this, not in file
                CurrentHiLowPin = 0,
                CurrentHiLowPinNeg = False,
                HomeBackOff = 0,
                RotaryAxis = False, # no rotary axes
                RotaryRollover = False
                )

        return setting_struct

    def _get_axis_setting(self):
        setting = AxisSetting()
        result = self.dll.GetAxisSetting(ctypes.byref(setting))
        if result != 0:
            raise RuntimeError(f"GetStatus failed with error code {result}")

        print(setting)

    def __del__(self):
        if self.connected:
            self.close_device()

if __name__=='__main__':

    # TODO: Figure out if I can copy these to a known filepath
    dll_path = "C:/UCCNC/API/DLL/UC100.dll"
    config_file = "Stepcraft2_Model420.pro"

    uc = UC100Controller(dll_path)

    num_devices = uc.list_devices()
    print(f"Found {num_devices} device(s)")

    # getting device 0 simply returns demo mode. Need to access device 1 to get
    for i in range(num_devices+1):
        info = uc.get_device_info(i)
        print(f"Device {i}: Type={info['type']}, Serial={hex(info['serial'])}")

    uc.open_device(1)  # Or use demo mode with 0
    print("Device opened.")

    # load config file
    uc.load_config(config_file)
    print("Configuration Loaded")

    input("Jog To Zero")
    uc.jog_to_zero()

    path_plan = np.loadtxt("../path_generation/paths/wall.csv", delimiter = ',')
    uc.init_pathplan(path_plan)
    input("Enter to execute pathplan")
    uc.run_pathplan()
    uc.close_device()
    print("Device closed.")


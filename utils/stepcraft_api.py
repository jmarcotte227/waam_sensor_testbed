import ctypes

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

        self.dll.SpindleOn.argtypes = [ctypes.c_bool]
        self.dll.SpindleOn.restype = ctypes.c_int

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

        self.dll.SetAxisPosition.argtypes = [ctypes.c_double,
                                             ctypes.c_double,
                                             ctypes.c_double,
                                             ctypes.c_double,
                                             ctypes.c_double,
                                             ctypes.c_double]
        self.dll.SetAxisPosition.restype = ctypes.c_int

        self.dll.SetFeedhold.argtypes = [ctypes.c_bool]
        self.dll.SetFeedhold.restype = ctypes.c_int

        self.connected = False
        self.device_id = None

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

    def linear_move_relative(self,
                             axis,
                             step,
                             step_count,
                             speed,
                             direction):

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
        result = self.dll.SetFeedhold(state)
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

    def start_pathplan(self):
        # undo the feedhold to start motion
        self.set_feedhold(False)

    def __del__(self):
        if self.connected:
            self.close_device()

if __name__=='__main__':

    # TODO: Figure out if I can copy these to a known filepath
    dll_path = "C:/UCCNC/API/DLL/UC100.dll"

    uc = UC100Controller(dll_path)

    num_devices = uc.list_devices()
    print(f"Found {num_devices} device(s)")

    # getting device 0 simply returns demo mode. Need to access device 1 to get
    for i in range(num_devices+1):
        info = uc.get_device_info(i)
        print(f"Device {i}: Type={info['type']}, Serial={hex(info['serial'])}")

    uc.open_device(0)  # Or use demo mode with 0
    print("Device opened.")

    status = uc.get_status()
    print(f"Idle: {status.Idle}")
    print(f"Estop: {status.Estop}")
    print(f"SpindleOn: {status.SpindleOn}")


    # Do other operations...
    input()
    print("Turning on Spindle")
    uc.spindle_on()
    status = uc.get_status()
    print(f"Idle: {status.Idle}")
    print(f"Estop: {status.Estop}")
    print(f"SpindleOn: {status.SpindleOn}")
    input()
    uc.close_device()
    print("Device closed.")


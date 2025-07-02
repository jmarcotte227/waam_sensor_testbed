'''
To be used with an arduino hooked up to the stepcraft through the external I/O port.
Necessary connections TBA. Starting with ability to trigger cycle start.

Use modified standardFirmata file in this repo, as this makes the default out state high
'''
import time
from threading import Thread
import pyfirmata2 as pf
import signal   

class WeldTrigger:
    '''
    Interface to control the stepcraft CNC with an arduino. 
    Allows gcode to be started and stopped.
    '''
    def __init__(self, port=None):
        # connect to arduino
        # None port autodetects arduino
        arduino = pf.Arduino(port)
        self.out_pin = arduino.get_pin('d:8:o')

        # initialize pin states
        self.out_pin.write(0)

    def weld_on(self):
        self.out_pin.write(1)

    def weld_off(self):
        self.out_pin.write(0)

    def e_stop(self):
        self.weld_off()


if __name__=='__main__':
    # can be used to test cycle start and cycle stop
    interface = WeldTrigger()
    input("Enter to weld")
    interface.weld_on()
    time.sleep(3)
    interface.weld_off()

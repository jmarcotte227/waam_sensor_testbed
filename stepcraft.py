'''
To be used with an arduino hooked up to the stepcraft through the external I/O port.
Necessary connections TBA. Starting with ability to trigger cycle start.

Use modified standardFirmata file in this repo, as this makes the default out state high
'''
import time
from threading import Thread
import pyfirmata2 as pf

class StepcraftInterface:
    def __init__(self, port=None):
        # connect to arduino
        # None port autodetects arduino
        arduino = pf.Arduino(port)
        self.start_pin = arduino.get_pin('d:8:o')
        self.stop_pin = arduino.get_pin('d:11:o')
        # initialize pin states
        self.start_pin.write(1)
        self.stop_pin.write(1)


    def cycle_start(self):
        '''
        Triggers the cycle start pin on the arudino
        Opens thread to not block main loop execution
        '''
        t = Thread(target=self._toggle_input, args=(self.start_pin,))
        t.start()


    def cycle_stop(self):
        '''
        Triggers the cycle stop pin on the arduino 
        '''
        t = Thread(target=self._toggle_input, args=(self.stop_pin,))
        t.start()

    def _toggle_input(self,pin):
        pin.write(0)
        time.sleep(1)
        pin.write(1)
if __name__=='__main__':
    # can be used to test cycle start and cycle stop
    interface = StepcraftInterface()
    interface.cycle_start()
    input()
    interface.cycle_stop()

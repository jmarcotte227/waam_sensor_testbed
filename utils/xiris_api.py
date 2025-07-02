import numpy as np
import json
import http.client
import requests
import time

class XirisInterface():
    def __init__(self, address = 'http://localhost:8000/', timeout = 10, cam_key = '00111C05DA8B'):
        '''
        Interface Class for the XIR-1800 in the WAAM cell at RPI. 
        '''
        self.address = address
        self.cam_key = cam_key
        self.timeout = timeout

        # test for connection to weld studio
        # r = requests.post(self.address+'rpc/App/Test', timeout = self.timeout)
        # print(r)

    def list_cameras(self):
        list_cmd = 'api/Cameras'
        r = requests.get(self.address+list_cmd, timeout=self.timeout)
        return r.json()

    def set_filename(self, filename):
        # sets the file name of the recording
        set_overlay_cmd = 'rpc/Camera/SetRecordingLabel'
        data = {
                'camera': self.cam_key,
                'label':filename,
                }
        r = requests.post(self.address+set_overlay_cmd, json=data, timeout=self.timeout)
        print(r.content)

    def set_overlay(self, text):
        # sets the file name of the recording
        set_overlay_cmd = 'rpc/Camera/SetRecordingLabel'
        data = {
                'camera': self.cam_key,
                'label':text,
                }
        r = requests.post(self.address+set_overlay_cmd, json=data, timeout=self.timeout)
        print(r.content)

    def start_recording(self):
        # starts a recording session
        record_cmd = 'rpc/Camera/StartRecording'
        data = {
                'camera': self.cam_key
                }
        r = requests.post(self.address+record_cmd, json=data, timeout=self.timeout)
        print(r.text)

    def stop_recording(self):
        stop_cmd = 'rpc/Camera/StopRecording'
        data = {
                'camera': self.cam_key
                }
        r = requests.post(self.address+stop_cmd, json=data, timeout=self.timeout)

    def e_stop(self):
        self.stop_recording()

if __name__=='__main__':
    xiris = XirisInterface()
    xiris.set_filename('Experiment_xxx_3')
    xiris.set_overlay('Experiment_xxx_3')
    time.sleep(2)
    xiris.start_recording()
    time.sleep(3)
    xiris.stop_recording()
    # xiris.start_recording()

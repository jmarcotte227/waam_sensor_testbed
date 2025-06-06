from RobotRaconteur.Client import *
import RobotRaconteur
import time, copy, pickle, wave
import threading
import numpy as np
from matplotlib import pyplot as plt

class SensorSuite(object):
    def __init__(
            self,
            spec_service=None,
            spec_freq = 2, # hz
            flir_service=None,
            microphone_service=None,
            current_service=None
            ) -> None:	
        ## Spectrometer Service
        self.spec_ser= spec_service
        self.spec_per = 1/spec_freq

        if spec_service:
            self.spec_ser.setf_param("electric_dark", RR.VarValue(True, "bool"))
            self.spec_ser.setf_param("integration_time", RR.VarValue(6000,"int32"))
            self.spec_ser.setf_param("scans_to_average", RR.VarValue(5, "int32"))

            self.start_spec_cb = False

            # initialize thread that takes measurements
            self.spec_thread = threading.Thread(target=self.meas_spec, daemon=True)
            self.spec_thread.start()

        ## IR Camera Service

        self.flir_ser=flir_service
        if flir_service:
            self.ir_image_consts = RRN.GetConstants('com.robotraconteur.image', self.flir_ser)

            self.flir_ser.setf_param("focus_pos", RR.VarValue(int(1900),"int32"))
            self.flir_ser.setf_param("object_distance", RR.VarValue(0.4,"double"))
            self.flir_ser.setf_param("reflected_temperature", RR.VarValue(291.15,"double"))
            self.flir_ser.setf_param("atmospheric_temperature", RR.VarValue(293.15,"double"))
            self.flir_ser.setf_param("relative_humidity", RR.VarValue(50,"double"))
            self.flir_ser.setf_param("ext_optics_temperature", RR.VarValue(293.15,"double"))
            self.flir_ser.setf_param("ext_optics_transmission", RR.VarValue(0.99,"double"))
            self.flir_ser.setf_param("current_case", RR.VarValue(2,"int32"))
            self.flir_ser.setf_param("ir_format", RR.VarValue("radiometric","string"))
            self.flir_ser.setf_param("object_emissivity", RR.VarValue(0.13,"double"))
            self.flir_ser.setf_param("scale_limit_low", RR.VarValue(293.15,"double"))
            self.flir_ser.setf_param("scale_limit_upper", RR.VarValue(5000,"double"))

            self.cam_pipe=self.flir_ser.frame_stream.Connect(-1)
            #Set the callback for new pipe packets
            self.start_ir_cb = False
            self.cam_pipe.PacketReceivedEvent+=self.ir_cb
            try:
                self.flir_ser.start_streaming()
            except:
                pass
            self.clean_ir_record()

        ## microphone service
        self.mic_service=microphone_service
        if microphone_service:
            self.mic_samplerate = 44100
            self.mic_channels = 1
            self.mic_service=microphone_service
            self.mic_pipe = self.mic_service.microphone_stream.Connect(-1)
            self.clean_mic_record()
            self.start_mic_cb=False
            self.mic_pipe.PacketReceivedEvent+=self.microphone_cb

        self.current_service=current_service
        if current_service:
            self.current_state_sub = self.current_service.SubscribeWire("current")
            self.start_current_cb = False
            self.clean_current_record()
            self.current_state_sub.WireValueChanged += self.current_cb


    def start_all_sensors(self):

        if self.flir_ser:
            self.clean_ir_record()
            self.start_ir_cb=True
        if self.mic_service:
            self.clean_mic_record()
            self.start_mic_cb=True
        if self.current_service:
            self.clean_current_record()
            self.start_current_cb=True
        if self.spec_ser:
            self.clean_spec_record()
            self.start_spec_cb=True

    def clear_all_sensors(self):
        if self.weld_service:
            self.clean_weld_record()
        if self.flir_ser:
            self.clean_ir_record()
        if self.mic_service:
            self.clean_mic_record()
        if self.current_service:
            self.clean_current_record()

    def stop_all_sensors(self):

        self.start_weld_cb=False
        self.start_ir_cb=False
        self.start_mic_cb=False
        self.start_current_cb=False

    def save_all_sensors(self,filedir):

        if self.spec_ser:
            self.save_spec_file(filedir)
        if self.flir_ser:
            self.save_ir_file(filedir)
        if self.mic_service:
            self.save_mic_file(filedir)
        if self.current_service:
            self.save_current_file(filedir)


    def test_all_sensors(self,t=3):

        self.start_all_sensors()
        time.sleep(t)
        self.stop_all_sensors()

        if self.flir_ser:
            fig = plt.figure(1)
            sleep_t=float(3./len(self.ir_recording))
            for r in self.ir_recording:
                plt.imshow(r, cmap='inferno', aspect='auto')
                plt.colorbar(format='%.2f')
                plt.pause(sleep_t)
                plt.clf()
        if self.mic_service:
            first_channel = np.concatenate(self.audio_recording)
            first_channel_int16=(first_channel*32767).astype(np.int16)
            plt.plot(first_channel_int16)
            plt.title("Microphone data")
            plt.show()
        if self.current_service:
            print("Current data length:",len(self.current))
            plt.plot(self.current_timestamp,self.current)
            plt.title("Current data")
            plt.show()

    def clean_weld_record(self):

        self.weld_timestamp=[]
        self.weld_voltage=[]
        self.weld_current=[]
        self.weld_feedrate=[]
        self.weld_energy=[]

    def clean_current_record(self):
        self.current=[]
        self.current_timestamp=[]

    def weld_cb(self, sub, value, ts):

        if self.start_weld_cb:
            # self.weld_timestamp.append(value.ts['microseconds'][0])
            self.weld_timestamp.append(time.perf_counter())
            self.weld_voltage.append(value.welding_voltage)
            self.weld_current.append(value.welding_current)
            self.weld_feedrate.append(value.wire_speed)
            self.weld_energy.append(value.welding_energy)

    def current_cb(self, sub, value, ts):

        if self.start_current_cb:
            # self.current_timestamp.append(ts.seconds+ts.nanoseconds*1e-9)
            self.current_timestamp.append(time.perf_counter())
            self.current.append(value)


    def save_weld_file(self,filedir):
        np.savetxt(filedir + 'welding.csv',
                   np.array([(np.array(self.weld_timestamp)), 
                             self.weld_voltage,
                             self.weld_current,
                             self.weld_feedrate,
                             self.weld_energy]).T,
                   delimiter=',',
                   header='timestamp,voltage,current,feedrate,energy',
                   comments=''
                   )

    def save_current_file(self,filedir):
        np.savetxt(filedir + 'current.csv',
                   np.array([(np.array(self.current_timestamp)),
                             self.current]).T,
                   delimiter=',',
                   header='timestamp,current',
                   comments=''
                   )

    def clean_ir_record(self):
        self.ir_timestamp=[]
        self.ir_recording=[]

    def ir_cb(self,pipe_ep):

        # Loop to get the newest frame
        while (pipe_ep.Available > 0):
            # Receive the packet
            rr_img = pipe_ep.ReceivePacket()
            if self.start_ir_cb:
                if rr_img.image_info.encoding == self.ir_image_consts["ImageEncoding"]["mono8"]:
                    # Simple uint8 image
                    mat = rr_img.data.reshape([rr_img.image_info.height, rr_img.image_info.width], order='C')
                elif rr_img.image_info.encoding == self.ir_image_consts["ImageEncoding"]["mono16"]:
                    data_u16 = np.array(rr_img.data.view(np.uint16))
                    mat = data_u16.reshape([rr_img.image_info.height, rr_img.image_info.width], order='C')

                ir_format = rr_img.image_info.extended["ir_format"].data

                if ir_format == "temperature_linear_10mK":
                    display_mat = (mat * 0.01) - 273.15
                elif ir_format == "temperature_linear_100mK":
                    display_mat = (mat * 0.1) - 273.15
                else:
                    display_mat = mat

                # Convert the packet to an image and set the global variable
                self.ir_recording.append(copy.deepcopy(display_mat))
                # self.ir_timestamp.append(rr_img.image_info.data_header.ts['seconds']+rr_img.image_info.data_header.ts['nanoseconds']*1e-9)
                self.ir_timestamp.append(time.perf_counter())

    def save_ir_file(self,filedir):

        with open(filedir+'ir_recording.pickle','wb') as file:
            pickle.dump(np.array(self.ir_recording),file)
        np.savetxt(filedir + "ir_stamps.csv",self.ir_timestamp,delimiter=',')

    def clean_mic_record(self):

        self.audio_recording=[]

    def microphone_cb(self,pipe_ep):

        #Loop to get the newest frame
        while (pipe_ep.Available > 0):
            audio = pipe_ep.ReceivePacket().audio_data
            if self.start_mic_cb:
                #Receive the packet
                self.audio_recording.extend(audio)

    def save_mic_file(self,filedir):

        print("Mic length:",len(self.audio_recording))

        try:
            first_channel = np.concatenate(self.audio_recording)

            first_channel_int16=(first_channel*32767).astype(np.int16)
            with wave.open(filedir+'mic_recording.wav', 'wb') as wav_file:
                # Set the WAV file parameters
                wav_file.setnchannels(self.mic_channels)
                wav_file.setsampwidth(2)  # 2 bytes per sample (16-bit)
                wav_file.setframerate(self.mic_samplerate)

                # Write the audio data to the WAV file
                wav_file.writeframes(first_channel_int16.tobytes())
        except:
            print("Mic has no recording!!!")

    def clean_spec_record(self):
        self.spec_timestamp=[]
        self.spec_counts=[]
        self.spec_wavelengths=[]

    def meas_spec(self):
        next_call = time.time()
        while True:
            if self.start_spec_cb:
                # measure waveform
                try:
                    spectrum = self.spec_ser.capture_spectrum()
                except (RobotRaconteur.RobotRaconteurPythonError.InvalidOperationException,
                        RobotRaconteur.RobotRaconteurPythonError.InvalidEndpointException):
                    print("Spectrometer no longer available, terminating measurements")
                    self.start_spec_cb=False

                self.spec_timestamp.append(time.perf_counter())
                self.spec_counts.append(spectrum.spectrum_counts)
                self.spec_wavelengths.append(spectrum.wavelengths)

                next_call = next_call+self.spec_per
                time.sleep(next_call - time.time())

    def save_spec_file(self,filedir):
        np.savetxt(filedir + 'spec_wavelengths.csv',
                   np.hstack((np.array([self.spec_timestamp]).T,
                             np.array(self.spec_wavelengths))),
                   delimiter=',',
                   header='timestamp,wavelengths',
                   comments=''
                   )
        np.savetxt(filedir + 'spec_counts.csv',
                   np.hstack((np.array([self.spec_timestamp]).T,
                             np.array(self.spec_counts))),
                   delimiter=',',
                   header='timestamp,counts',
                   comments=''
                   )

    def save_data_streaming(self,
                            recorded_dir,
                            current_data,
                            welding_data,
                            audio_recording,
                            robot_data,
                            flir_logging,
                            flir_ts,
                            slice_num,
                            section_num=0
                            ):
        ###MAKING DIR
        layer_data_dir=recorded_dir+'layer_'+str(slice_num)+'_'+str(section_num)+'/'
        Path(layer_data_dir).mkdir(exist_ok=True)

        ####AUDIO SAVING
        first_channel = np.concatenate(audio_recording)
        first_channel_int16=(first_channel*32767).astype(np.int16)
        with wave.open(layer_data_dir+'mic_recording.wav', 'wb') as wav_file:
            # Set the WAV file parameters
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)  # 2 bytes per sample (16-bit)
            wav_file.setframerate(44100)
            # Write the audio data to the WAV file
            wav_file.writeframes(first_channel_int16.tobytes())

        ####CURRENT SAVING
        np.savetxt(layer_data_dir + 'current.csv',current_data, delimiter=',',header='timestamp,current', comments='')

        ####FRONIUS SAVING
        np.savetxt(layer_data_dir + 'welding.csv',welding_data, delimiter=',',header='timestamp,voltage,current,feedrate,energy', comments='')


        ####ROBOT JOINT SAVING
        np.savetxt(layer_data_dir+'joint_recording.csv',robot_data,delimiter=',')

        ###FLIR SAVING
        flir_ts=np.array(flir_ts)
        with open(layer_data_dir+'ir_recording.pickle','wb') as file:
            pickle.dump(np.array(flir_logging),file)
        np.savetxt(layer_data_dir + "ir_stamps.csv",flir_ts,delimiter=',')

        return


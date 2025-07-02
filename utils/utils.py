def estop_all(device_list):
    for device in device_list:
        try:
            device.e_stop()
        except RuntimeError as e:
            print(e)

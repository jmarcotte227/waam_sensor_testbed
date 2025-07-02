def estop_all(device_list):
    for device in device_list:
        device.e_stop()

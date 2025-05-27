import pygetwindow as gw
import keyboard
import time

'''
Generated using ChatGPT. 
'''

# Focus the window (use part of the title or full name)
window = gw.getWindowsWithTitle('UCCNC software for STEPCRAFT')[0]
window.activate()
time.sleep(2)

# Send the hotkey
keyboard.press_and_release('k')
print("acutated")


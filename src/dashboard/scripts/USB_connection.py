import win32com.client


def Connected_Device():

    wmi = win32com.client.GetObject ("winmgmts:")
    
    for usb in wmi.InstancesOf ("Win32_USBHub"):
        print(usb.DeviceID)
        # if usb.DeviceID == "USB\VID_8086&PID_0B3A\944223050481":
        #     return True
    
    return False

Connected_Device()
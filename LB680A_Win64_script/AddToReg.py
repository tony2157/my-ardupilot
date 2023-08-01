import winreg as reg

def addToReg():
    key = reg.OpenKey(reg.HKEY_CURRENT_USER, "Software\Microsoft\Windows\CurrentVersion\Run", 0, reg.KEY_ALL_ACCESS)
    reg.SetValueEx(key, "ARRC_LB680A_reg", 0, reg.REG_SZ, "C:\\Users\\PC_NAME\\AppData\\Roaming\\Microsoft\\Windows\\Start Menu\\Programs\\Startup\\LB680A_ArduPilot.pyw")

addToReg()

print("\a")
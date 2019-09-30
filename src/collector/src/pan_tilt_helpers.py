import serial
import time

class PanTilt:
    PR = 92.571429
    TR = 46.285714

    def __init__(self, port="/dev/ptu"):
        self.ser = serial.Serial(port, 9600, timeout=5)
        if not self.ser.is_open:
            raise("Failed to open Pan/Tilt serial port.")
        self.setPanTilt(0, 0)

    def setPanTilt(self, pan_deg, tilt_deg, delay=5.0):
        self.setPan(pan_deg, 0.0)
        self.setTilt(tilt_deg, 0.0)
        time.sleep(delay)

    def setPan(self, pan_deg, delay=5.0):
        pan_ticks = self.deg2PanTicks(pan_deg)
        pan_str = 'PP'+str(int(pan_ticks))+' \n'
        b = bytearray()
        b.extend(pan_str)
        print(b)
        self.ser.write(b)
        self.ser.write(b)
        self.ser.write(b)
        self.ser.write(b)
        time.sleep(delay)

    def setTilt(self, tilt_deg, delay=5.0):
        tilt_ticks = self.deg2TiltTicks(tilt_deg)
        tilt_str = 'TP'+str(int(tilt_ticks))+' \n'
        b = bytearray()
        b.extend(tilt_str)
        print(b)
        self.ser.write(b)
        self.ser.write(b)
        self.ser.write(b)
        self.ser.write(b)
        time.sleep(delay)

    def deg2PanTicks(self, pan_deg):
        return pan_deg / (PanTilt.PR / 3600)

    def deg2TiltTicks(self, tilt_deg):
        return tilt_deg / (PanTilt.TR / 3600)

if __name__=="__main__":
    ptu = PanTilt()
    ptu.setPanTilt(-20, 30)
    ptu.setPanTilt(0, 0)
    ptu.setPanTilt(20, -30)
    ptu.setPanTilt(0, 0)

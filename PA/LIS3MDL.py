"""
CODE TO INTERFACE WITH THE COMPASS ONBOARD THE BERRY-GPS-IMU-v4

"""
import smbus
from math import atan2,pi

LIS3MDL_ADDRESS = 0x1C

LIS3MDL_WHO_AM_I = 0x0F

LIS3MDL_CTRL_REG1 = 0x20

LIS3MDL_CTRL_REG2 = 0x21
LIS3MDL_CTRL_REG3 = 0x22
LIS3MDL_CTRL_REG4 = 0x23
LIS3MDL_CTRL_REG5 = 0x24

LIS3MDL_STATUS_REG = 0x27

LIS3MDL_OUT_X_L = 0x28
LIS3MDL_OUT_X_H = 0x29
LIS3MDL_OUT_Y_L = 0x2A
LIS3MDL_OUT_Y_H = 0x2B
LIS3MDL_OUT_Z_L = 0x2C
LIS3MDL_OUT_Z_H = 0x2D

LIS3MDL_TEMP_OUT_L = 0x2E
LIS3MDL_TEMP_OUT_H = 0x2F

LIS3MDL_INT_CFG = 0x30
LIS3MDL_INT_SRC = 0x31
LIS3MDL_INT_THS_L = 0x32
LIS3MDL_INT_THS_H = 0x33

MAGX_MAX = 555.00
MAGY_MAX = 3270.00
MAGZ_MAX = 974.00

MAGX_MIN = -2420.00
MAGY_MIN = 0.00
MAGZ_MIN = -3654.00


class LIS3MDL(object):
    """docstring for LIS3MDL"""

    def __init__(self, bus, address=LIS3MDL_ADDRESS):
        self._address = address
        self._bus = bus

        # initialise the magnetometer
        self._writeByte( LIS3MDL_CTRL_REG1,0b11011100)  # Temp sesnor enabled, High performance, ODR 80 Hz, FAST ODR disabled and Selft test disabled.
        self._writeByte( LIS3MDL_CTRL_REG2, 0b00100000)  # +/- 8 gauss
        self._writeByte( LIS3MDL_CTRL_REG3, 0b00000000)  # Continuous-conversion mode

    def _writeByte(self, register, value):
        self._bus.write_byte_data(self._address, register, value)

    def readMAGx(self):
        mag_l = self._bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_OUT_X_L)
        mag_h = self._bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_OUT_X_H)

        mag_combined = (mag_l | mag_h << 8)
        return mag_combined if mag_combined < 32768 else mag_combined - 65536

    def readMAGy(self):
        mag_l = self._bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_OUT_Y_L)
        mag_h = self._bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_OUT_Y_H)

        mag_combined = (mag_l | mag_h << 8)
        return mag_combined if mag_combined < 32768 else mag_combined - 65536

    def readMAGz(self):
        mag_l = self._bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_OUT_Z_L)
        mag_h = self._bus.read_byte_data(LIS3MDL_ADDRESS, LIS3MDL_OUT_Z_H)

        mag_combined = (mag_l | mag_h << 8)
        return mag_combined if mag_combined < 32768 else mag_combined - 65536



if __name__ == '__main__':

    import time
    ## TILT COMPENSATION REQUIRED AND CALIBRATION
    print("LIS3MDL Test Program ...\n")
    LIS3MDL = LIS3MDL(smbus.SMBus(0x01))
    magMax = [0, 0, 0]
    magMin = [0, 0, 0]
    while True:
        time.sleep(0.5)
        magX = LIS3MDL.readMAGx()
        magY = -LIS3MDL.readMAGy()
        magZ = LIS3MDL.readMAGz()

        magX_comp = magX - (MAGX_MIN + MAGX_MAX) / 2
        magY_comp = magY - (MAGY_MIN + MAGY_MAX) / 2
        magZ_comp = magZ - (MAGZ_MIN + MAGZ_MAX) / 2

        if magX > magMax[0]:
            magMax[0] = magX
        if magY > magMax[1]:
            magMax[1] = magY
        if magZ > magMax[2]:
            magMax[2] = magZ

        if magX < magMin[0]:
            magMin[0] = magX
        if magY < magMin[1]:
            magMin[1] = magY
        if magZ < magMin[2]:
            magMin[2] = magZ

        print('===================================================')
        print('Raw:')
        heading = atan2(magY,magX) * 180 / pi
        if heading < 0:
            heading += 360
        print(' magX = %.2f magY = %.2f  magZ =%.2f ' % (magX, magY, magZ))
        print(' Heading = %.2f\n' % (heading))

        print('Compensated:')
        heading = atan2(magY_comp, magX_comp) * 180 / pi
        if heading < 0 :
            heading += 360
        print(' magX = %.2f magY = %.2f  magZ =%.2f ' % (magX_comp, magY_comp, magZ_comp))
        print(' Heading = %.2f\n' % (heading))

        print('MAGX_MAX = %.2f\nMAGY_MAX = %.2f\nMAGZ_MAX =%.2f\n' % (magMax[0], magMax[1], magMax[2]))
        print('MAGX_MIN = %.2f\nMAGY_MIN = %.2f\nMAGZ_MIN =%.2f\n' % (magMin[0], magMin[1], magMin[2]))

        print('===================================================\n\n')
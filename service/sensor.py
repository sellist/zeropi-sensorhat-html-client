#!/usr/bin/python
# -*- coding:utf-8 -*-
import smbus

__all__ = ['get_pressure', 'get_temperature', 'get_humidity', 'get_light', 'get_uv', 'get_gas', 'get_roll', 'get_pitch',
           'get_yaw', 'get_acceleration', 'get_gyroscope', 'get_magnetic']

from drivers import *

MPU_VAL_WIA = 0x71
MPU_ADD_WIA = 0x75
ICM_VAL_WIA = 0xEA
ICM_ADD_WIA = 0x00
ICM_SLAVE_ADDRESS = 0x68
bus = smbus.SMBus(1)
bme280: BME280 = BME280.BME280()
bme280.get_calib_param()
light = TSL2591.TSL2591()
uv = LTR390.LTR390()
sgp = SGP40.SGP40()

device_id1 = bus.read_byte_data(int(ICM_SLAVE_ADDRESS), int(ICM_ADD_WIA))
device_id2 = bus.read_byte_data(int(ICM_SLAVE_ADDRESS), int(MPU_ADD_WIA))
if device_id1 == ICM_VAL_WIA:
    mpu = ICM20948.ICM20948()
elif device_id2 == MPU_VAL_WIA:
    mpu = MPU925x.MPU925x()

# time.sleep(1)
bme = []
bme = bme280.readData()
pressure = round(bme[0], 2)
temp = round(bme[1], 2)
hum = round(bme[2], 2)

lux = round(light.Lux(), 2)

UVS = uv.UVS()

gas = round(sgp.raw(), 2)

icm = []
icm = mpu.getdata()


def get_pressure() -> float:
    return round(bme280.readData()[0], 2)


def get_temperature() -> float:
    return round(bme280.readData()[1], 2)


def get_humidity() -> float:
    return round(bme280.readData()[2], 2)


def get_light() -> int:
    return round(light.Lux(), 2)


def get_uv() -> int:
    return uv.UVS()


def get_gas() -> float:
    return round(sgp.raw(), 2)


def get_roll() -> float:
    return icm[0]


def get_pitch() -> float:
    return icm[1]


def get_yaw() -> float:
    return icm[2]


def get_acceleration() -> tuple:
    return icm[3], icm[4], icm[5]


def get_gyroscope() -> tuple:
    return icm[6], icm[7], icm[8]


def get_magnetic() -> tuple:
    return icm[9], icm[10], icm[11]

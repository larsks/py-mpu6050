import gc
import network
import machine
from machine import Pin

gc.collect()

ap_if = network.WLAN(network.AP_IF)
ap_if.config(essid='MPU', password='drones.in.space')
ap_if.active()

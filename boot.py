import gc
import webrepl
import network
import machine
from machine import Pin

#webrepl.start()
gc.collect()

sta_if = network.WLAN(network.STA_IF)
sta_if.active()
sta_if.connect("vankelsted", "")
while not sta_if.isconnected():
    pass
print("network config:", sta_if.ifconfig())



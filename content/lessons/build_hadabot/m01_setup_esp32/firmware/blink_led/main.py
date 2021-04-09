from machine import Pin
import utime

led = Pin(2, Pin.OUT)
led.off()
for i in range(5):
    led.on()
    utime.sleep_ms(500)
    led.off()
    utime.sleep_ms(500)

led.on()

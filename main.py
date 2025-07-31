# HV Supply Board Code
# importing modules
import time
import sys
from machine import Pin, I2C, Timer, UART
import ads1x15
from array import array
from time import sleep_ms, ticks_ms, ticks_us
import uasyncio as asyncio

external = False


# DAC configuration
class AD5693R:
    def __init__(self, i2c, address=0x4C):
        self.i2c = i2c
        self.address = address

    def reset(self):
        buf = bytes([0x70, 0x00, 0x00])
        self.i2c.writeto(self.address, buf)

    def write_dac(self, value):
        # value: 0 - 4095 (12-bit)
        if not 0 <= value <= 4095:
            raise ValueError("DAC value out of range")

        # Command 0x30: Write DAC register and update output
        command = 0x30 | 0x20
        msb = (value >> 4) & 0xFF
        lsb = (value << 4) & 0xFF
        buf = bytes([command, msb, lsb])
        self.i2c.writeto(self.address, buf)

    def write_control_register(self, reset=0, pd1=0, pd0=0, ref=0, gain=1):
        # Build 16-bit control register value
        control = (reset << 15) | (pd1 << 14) | (pd0 << 13) | (ref << 12) | (gain << 11)

        # Command byte: Write Control Register command is 0x1 << 4 = 0x10
        command = 0x40

        msb = (control >> 8) & 0xFF
        lsb = control & 0xFF

        buf = bytes([command, msb, lsb])
        self.i2c.writeto(self.address, buf)

    def set_gain(self, gain):
        self.write_control_register(gain=(1 if gain == 2 else 0))


# pin number configuration
i2c_sda = 0
i2c_scl = 1
adc_alert_pin = 13
dac_reset_pin = 14
dac_ldac_pin = 15
op_enable_pin = 26
inverter_enable_pin = 27
converter_enable_pin = 28
op_status_pin = 29

# address config
dac_addr = 0x4c
adc_addr = 0x48

# pin configuration
inv_enable = Pin(inverter_enable_pin, Pin.OUT)
conv_enable = Pin(converter_enable_pin, Pin.OUT)
dac_reset = Pin(dac_reset_pin, Pin.OUT)
op_enable = Pin(op_enable_pin, Pin.OUT)
op_status = Pin(op_status_pin, Pin.IN)

# constants
buffersize = const(512)
adc_rate = const(5)
adc_gain = 1
dac_gain = 1
max_out = 2.5

if (dac_gain == 2):
    max_out = 5

# array
data = array("h", (0 for _ in range(buffersize)))
timestamps = array("L", (0 for _ in range(buffersize)))

# state variables
index_put = 0
irq_busy = False

# i2c configuration
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100000)
uart = UART(1, baudrate=115200, tx=Pin(8), rx=Pin(5))
time.sleep(0.1)

# ic configuration
dac_reset.value(1)
dac = AD5693R(i2c)
dac.set_gain(1)
adc = ads1x15.ADS1115(i2c, adc_addr, 1)
adc.gain = 1  # +/-4.096V

inv_enable.value(0)
conv_enable.value(0)
op_enable.value(0)


# some basic functions

def power_on():
    inv_enable.value(1)
    conv_enable.value(1)
    op_enable.value(1)
    print("board is now on")


def power_off():
    set_voltage(0.5)
    inv_enable.value(0)
    conv_enable.value(0)
    op_enable.value(0)
    print("board is now off")


def set_voltage(set_voltage):
    global current_voltage
    step = 0.01
    delay = 0.01

    try:
        current_voltage
    except NameError:
        current_voltage = 0.0

    if (set_voltage > current_voltage):
        direction = 1
    else:
        direction = -1

    adj_voltage = set_voltage / 17

    voltage = current_voltage
    while abs(voltage - adj_voltage) > step:
        voltage += direction * step
        voltage = max(0, min(2.5, voltage))

        dac_value = round((voltage * 4095) / max_out)
        dac.write_dac(dac_value)
        time.sleep(delay)


def set_raw(raw):
    dac.write_dac(raw)
    time.sleep(0.01)


def read_voltage():
    value = adc.read(0, 1)
    voltage_adc = (value / 32768.0) * 4.096
    high_voltage = voltage_adc * 41
    return high_voltage


def read_current():
    raw = adc.read(0, 0)
    vmon = (raw / 32768.0) * 4.096
    current = vmon / 10000 * 5
    return current


def check_status():
    if conv_enable.value() == 1:
        return ("Board is On.")
    else:
        return ("Board is Off")





irq_busy = False
index_put = 0
ADC_RATE = 5


def stream_samples():
    adc.set_conv(7, 1)
    samplesize = 600
    current_index = 0
    while current_index < samplesize:
        raw = adc.read_rev()
        voltage = (raw / 32768.0) * 4.096 * 41

        print(f"{voltage:.3f}")
        time.sleep_ms(10)
        current_index += 1
    print("END")
    time.sleep(0.1)


def stream_samples_current():
    adc.set_conv(7, 0)
    samplesize = 600
    current_index = 0
    while current_index < samplesize:
        raw = adc.read_rev()
        current = (raw / 32768.0) * 4.096 / 10000 * 5

        print(f"{current:.6f}")
        time.sleep_ms(10)
        current_index += 1
    print("END")
    time.sleep(0.1)


def stream_both():
    sample_size = 600
    current_index = 0
    while current_index < sample_size:
        raw = adc.read(7, 1)
        voltage = (raw / 32768.0) * 4.096 * 41
        raw = adc.read(7, 0)
        current = (raw / 32768.0) * 4.096 / 10000 * 5
        print(f"{voltage:.3f},{current:.9f}")
        current_index += 1
        time.sleep_ms(2)
    print("END")



def execute_command(cmd):
    cmd = cmd.strip()
    if cmd == "*IDN?":
        return "HV Supply Board"
    elif cmd == "ON":
        power_on()
        return "Board is ON"
    elif cmd == "OFF":
        power_off()
        return "Board is OFF"
    elif cmd.startswith("VSET"):
        voltage_val = float(cmd[4:])
        set_voltage(voltage_val)
        return f"Voltage set to {voltage_val} V"
    elif cmd == "VREAD":
        response = read_voltage()
        # function to read voltage from adc
        return f"{response:.3f}"
    elif cmd == "IREAD":
        return read_current()
        # function to read current from adc
    elif cmd == "STATUS?":
        return check_status()
        # function to check ON/OFF status and voltage
    elif cmd == "TRACKV":
        stream_samples()
    elif cmd == "TRACKI":
        stream_samples_current()
    elif cmd == "TRACK":
        stream_both()


# main loop
def loop_external():
    while True:
        try:
            line = uart.read()

            if line:
                line = line.decode().strip()
                print("Received: ", line)
                result = execute_command(line)
                if result is not None:
                    uart.write(result)
                    print("Sent: ", result)
        #
        except KeyboardInterrupt:
            print("\nLoop Terminated")
            break


def loop():
    while True:
        try:
            line = sys.stdin.readline()
            if line:
                result = execute_command(line)
                if result is not None:
                    print(result)
        #
        except KeyboardInterrupt:
            print("\nLoop Terminated")
            break


if __name__ == "__main__":
    if (external):
        loop_external()
    else:
        loop()


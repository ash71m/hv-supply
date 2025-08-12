# HV Supply Board Code
# importing modules
import time
import sys
import select
from machine import Pin, I2C, Timer, UART, PWM, SPI
from array import array
from ad5693r import AD5693R
from ads131m02 import ADS131M02
from time import sleep_ms, ticks_ms, ticks_us
import uasyncio as asyncio

external = False

# pin number configuration
i2c_sda = 0
i2c_scl = 1
uart_tx = 4
uart_rx = 5
synq = 7
adc_rdy = 8
spi_cs0 = 9
spi_sclk = 10
spi_mosi = 11
spi_miso = 12
adc_clk = 13

dac_reset_pin = 14
dac_ldac_pin = 15
inverter_enable_pin = 26
op_status_pin = 27
op_enable_pin = 28
converter_enable_pin = 29

# pwm
pwm = PWM(Pin(adc_clk))
pwm.freq(8192000)
pwm.duty_u16(32768)

# address config
dac_addr = 0x4c

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
# uart = UART(1, baudrate=115200, tx=Pin(8), rx=Pin(5)) #uncomment this if using external mcu through UART

#spi config
spi = SPI(1, baudrate=100000, polarity=0, phase=1, bits=8, sck=Pin(spi_sclk), mosi=Pin(spi_mosi), miso=Pin(spi_miso))




# ic configuration
adc = ADS131M02(1, cs=Pin(spi_cs0, Pin.OUT), drdy=Pin(adc_rdy, Pin.IN))
print(adc.reset())  #prints true if reset worked
dac_reset.value(1)
dac = AD5693R(i2c)


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

    adj_voltage = set_voltage / 32.645

    voltage = current_voltage
    while abs(voltage - adj_voltage) > step:
        voltage += direction * step
        voltage = max(0, min(2.5, voltage))

        dac_value = round((voltage * 4095) / max_out)
        dac.write_dac(dac_value)
        print(dac_value)
        time.sleep(delay)


def set_raw(raw):
    dac.write_dac(raw)
    time.sleep(0.01)


def check_status():
    if conv_enable.value() == 1:
        return ("Board is On.")
    else:
        return ("Board is Off")

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

#Test Code that runs on startup
resp = adc.xfer_16(0x0000)
print(f"NOP response: {hex(resp)}")

adc.set_channel_enable(0, 1)
adc.set_channel_enable(1, 1)

vref = 1.2
fs_code = (1 << 23)


def code_to_volts(code):
    return (code / fs_code) * vref


time.sleep(1)
chip_id = adc.read_reg(adc.reg_id)
status = adc.read_reg(adc.reg_status)
print("Chip ID:", hex(chip_id))
print("Status:", hex(status))

power_on()
time.sleep_us(5)
set_voltage(10)
time.sleep_us(5)

timeout = 5000
start = time.ticks_ms()
while True:
    ready = adc.is_data_ready()
    if ready:
        print("ready!")
        data = adc.read_adc()
        print("raw data", data)
        ch0_v = code_to_volts(data[1])
        ch1_v = code_to_volts(data[2])
        print("data from adc:", ch0_v * 72.42, ch1_v * 72.42)
        break
    if time.ticks_ms() - start > timeout:
        print("timed out waiting for adc ready")
        break

power_off()





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
    if (external == True):
        loop_external()
    else:
        loop()



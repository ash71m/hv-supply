from machine import Pin, SPI
import time
"""
unfinished MicroPython driver for ADS131M02. read_adc is still in progress; reading and writing to registers is functional
"""
class ADS131M02:
    # commands
    null_cmd = 0x0000
    reset_cmd = 0x0011
    standby_cmd = 0x0022
    wakeup_cmd = 0x0033
    lock_cmd = 0x0555
    unlock_cmd = 0x0655
    read_reg_cmd = 0xA000
    write_reg_cmd = 0x6000

    # responses
    reset_ok = 0xFF22
    reset_nok = 0x0011

    # read only registers
    reg_id = 0x00
    reg_status = 0x01

    # global settings registers
    reg_mode = 0x02
    reg_clock = 0x03
    reg_gain = 0x04
    reg_cfg = 0x06

    # channel 0 registers
    reg_ch0_cfg = 0x09
    reg_ch0_ocal_msb = 0x0A
    reg_ch0_ocal_lsb = 0x0B
    reg_ch0_gcal_mcb = 0x0C
    reg_ch0_gcal_lsb = 0x0D

    # channel 1 registers
    reg_ch1_cfg = 0x0E
    reg_ch1_ocal_msb = 0x0F
    reg_ch1_ocal_lsb = 0x10
    reg_ch1_gcal_msb = 0x11
    reg_ch1_gcal_lsb = 0x12

    reg_map_crc = 0x3E

    # register masks
    regmask_read_addr = 0x1F80
    regmask_read_bytes = 0x007F
    regmask_status_lock = 0x8000
    regmask_status_resync = 0x4000
    regmask_status_regmap = 0x2000
    regmask_status_crc_err = 0x1000
    regmask_status_reset = 0x0200

    # conversion modes
    conversion_cont = 0
    conversion_single = 1

    # data format
    data_binary = 1

    # pga gain
    pga_gain_1 = 0
    pga_gain_2 = 1

    # mask register clock
    regmask_clock_ch1_en = 0x0200
    regmask_clock_ch0_en = 0x0100
    regmask_clock_osr = 0x001C
    regmask_clock_pwr = 0x0003

    def __init__(self, spi=1, sck=None, mosi=None, miso=None, cs=None, drdy=None, baudrate=1000000):
        self.spi = SPI(spi, baudrate=1_000_000, polarity=0, phase=1, sck=Pin(spi_sclk), mosi=Pin(spi_mosi),
                       miso=Pin(spi_miso))
        self.cs = Pin(spi_cs0, Pin.OUT)
        self.drdy = Pin(adc_rdy, Pin.IN)
        self.cs.value(1)

    def cs_low(self):
        self.cs.value(0)

    def cs_high(self):
        self.cs.value(1)

    def xfer_16(self, val):
        """
        parameters:
            val: 16-bit data to transfer to adc
        returns:
            16-bit response as an integer
        reading and writing done simultaneously
        """
        buf = val.to_bytes(2, 'big')
        rx = bytearray(2)
        self.spi.write_readinto(buf, rx)
        return int.from_bytes(rx, 'big')


    def xfer_8(self, val):
        """
                parameters:
                    val: 8-bit data to transfer to adc
                returns:
                    8-bit response as an integer
                reading and writing done simultaneously
         """
        buf = bytes([val])
        resp = bytearray(1)
        self.spi.write_readinto(buf, resp)
        return resp[0]

    def write_reg(self, addr, value):
        self.cs_low()
        time.sleep_us(1)
        cmd = self.write_reg_cmd | (addr << 7)
        self.xfer_16(cmd)
        self.xfer_8(0x00)
        self.xfer_16(value)
        self.xfer_8(0x00)
        self.cs_high()

    def read_reg(self, address):
        self.cs_low()
        time.sleep_us(1)
        cmd = self.read_reg_cmd | (address << 7)
        self.xfer_16(cmd)
        self.xfer_8(0x00)
        self.xfer_16(0x0000)
        self.xfer_8(0x00)
        resp = self.xfer_16(0x0000)
        self.cs_high()
        return resp

    def write_reg_masked(self, addr, value, mask):
        current = self.read_reg(addr)
        current &= ~mask
        current |= value
        self.write_reg(addr, current)

    def reset(self):
        self.cs_low()
        time.sleep_us(1)
        #         msb = self.xfer_8(0x00)
        #         lsb = self.xfer_8(0x11)
        #         self.xfer_8(0x00)
        resp = self.xfer_16(self.reset_cmd)
        #         print("resp:", hex(msb), hex(lsb))
        self.xfer_8(0x00)
        time.sleep_us(5)
        #         resp = (msb << 8) | lsb
        self.cs_high()
        return resp == self.reset_ok

    def unlock(self): #only needed when bit 15 of status reg is high
        self.cs_low()
        time.sleep_us(1)
        self.xfer_16(self.unlock_cmd)
        self.cs_high()

    def is_data_ready(self):
        # print("drdy is:", self.drdy.value() == 0)
        return self.drdy.value() == 0

    def is_reset(self):
        return bool(self.read_reg(self.reg_status) & self.regmask_status_reset)

    def is_lock(self):
        return bool(self.read_reg(self.reg_status) & self.regmask_status_lock)

    def set_channel_enable(self, channel, enable):
        if channel == 0:
            self.write_reg_masked(self.reg_clock, enable << 8, self.regmask_clock_ch0_en)
        elif channel == 1:
            self.write_reg_masked(self.reg_clock, enable << 9, self.regmask_clock_ch1_en)
        else:
            return False
        return True

    def read_adc(self):
        self.cs_low()
        time.sleep_us(1)
        status_msb = self.xfer_8(0x00)
        status_lsb = self.xfer_8(0x00)
        self.xfer_8(0x00)
        status = (status_msb << 8) | status_lsb

        # reading ch0 (voltage)
        b1 = self.xfer_8(0x00)
        b2 = self.xfer_8(0x00)
        b3 = self.xfer_8(0x00)

        raw0 = ((b1 << 16) | (b2 << 8) | b3) & 0x00FFFFFF
        ch0 = -((~raw0 & 0x00FFFFFF) + 1) if raw0 > 0x7FFFFF else raw0

        # discard values
        self.xfer_8(0x00)
        self.xfer_8(0x00)
        self.xfer_8(0x00)

        # reading ch1 (current)
        b1 = self.xfer_8(0x00)
        b2 = self.xfer_8(0x00)
        b3 = self.xfer_8(0x00)
        raw1 = ((b1 << 16) | (b2 << 8) | b3) & 0x00FFFFFF
        ch1 = -((~raw1 & 0x00FFFFFF) + 1) if raw1 > 0x7FFFFF else raw1

        self.cs_high()
        return status, ch0, ch1
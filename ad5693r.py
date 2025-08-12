class AD5693R:
    """
    Minimal Micropython Driver for AD5693R
    """
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


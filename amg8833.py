from smbus2 import SMBus

# Operating Modes
AMG8833_NORMAL_MODE = 0x00
AMG8833_SLEEP_MODE = 0x10
AMG8833_STAND_BY_60 = 0x20
AMG8833_STAND_BY_10 = 0x21

# sw resets
AMG8833_FLAG_RESET = 0x30
AMG8833_INITIAL_RESET = 0x3F

# frame rates
AMG8833_FPS_10 = 0x00
AMG8833_FPS_1 = 0x01

# int enables
AMG8833_INT_DISABLED = 0x00
AMG8833_INT_ENABLED = 0x01

# int modes
AMG8833_DIFFERENCE = 0x00
AMG8833_ABSOLUTE_VALUE = 0x01

AMG8833_INT_OFFSET = 0x010
AMG8833_PIXEL_OFFSET = 0x80

AMG8833_PIXEL_ARRAY_WIDTH = 8
AMG8833_PIXEL_ARRAY_HEIGHT = 8
AMG8833_PIXEL_TEMP_CONVERSION = 0.25
AMG8833_THERMISTOR_CONVERSION = 0.0625


def _signed_12bit_to_float(val):
    # take first 11 bits as absolute val
    abs_val = val & 0x7FF
    if val & 0x800:
        return 0 - float(abs_val)
    return float(abs_val)


def _twos_comp_to_float(val):
    val &= 0xFFF
    if val & 0x800:
        val -= 0x1000
    return float(val)


class AMG88XX:
    """Driver for the AMG88xx GRID-Eye IR 8x8 thermal camera."""

    # Set up the registers
    _pctl = i2c_bits.RWBits(8, 0x00, 0)
    _rst = i2c_bits.RWBits(8, 0x01, 0)
    _fps = i2c_bit.RWBit(0x02, 0)
    _inten = i2c_bit.RWBit(0x03, 0)
    _intmod = i2c_bit.RWBit(0x03, 1)

    _intf = i2c_bit.RWBit(0x04, 1)
    _ovf_irs = i2c_bit.RWBit(0x04, 2)
    _ovf_ths = i2c_bit.RWBit(0x04, 3)

    _intclr = i2c_bit.RWBit(0x05, 1)
    _ovs_clr = i2c_bit.RWBit(0x05, 2)
    _ovt_clr = i2c_bit.RWBit(0x05, 3)

    _mamod = i2c_bit.RWBit(0x07, 5)

    _inthl = i2c_bits.RWBits(8, 0x08, 0)
    _inthh = i2c_bits.RWBits(4, 0x09, 0)
    _intll = i2c_bits.RWBits(8, 0x0A, 0)
    _intlh = i2c_bits.RWBits(4, 0x0B, 0)
    _ihysl = i2c_bits.RWBits(8, 0x0C, 0)
    _ihysh = i2c_bits.RWBits(4, 0x0D, 0)

    _tthl = i2c_bits.RWBits(8, 0x0E, 0)

    _tthh = i2c_bits.RWBits(4, 0x0F, 0)

    def __init__(self, bus=1, address=0x69):
        self.address = address
        self.bus = SMBus(bus)

        # enter normal mode
        self._pctl = AMG8833_NORMAL_MODE

        # software reset
        self._rst = AMG8833_INITIAL_RESET

        # disable interrupts by default
        self._inten = False

        # set to 10 FPS
        self._fps = AMG8833_FPS_10

    @property
    def temperature(self):
        """Temperature of the sensor in Celsius"""
        raw = (self._tthh << 8) | self._tthl
        return _signed_12bit_to_float(raw) * AMG8833_THERMISTOR_CONVERSION

    @property
    def pixels(self):
        """Temperature of each pixel across the sensor in Celsius.
           Temperatures are stored in a two dimensional list where the first index is the row and
           the second is the column. The first row is on the side closest to the writing on the
           sensor."""
        retbuf = [[0] * AMG8833_PIXEL_ARRAY_WIDTH for _ in range(AMG8833_PIXEL_ARRAY_HEIGHT)]
        buf = bytearray(3)

        with self.i2c_device as i2c:
            for row in range(0, AMG8833_PIXEL_ARRAY_HEIGHT):
                for col in range(0, AMG8833_PIXEL_ARRAY_WIDTH):
                    i = row * AMG8833_PIXEL_ARRAY_HEIGHT + col
                    buf[0] = AMG8833_PIXEL_OFFSET + (i << 1)
                    i2c.write_then_readinto(buf, buf, out_end=1, in_start=1)

                    raw = (buf[2] << 8) | buf[1]
                    retbuf[row][col] = _twos_comp_to_float(raw) * AMG8833_PIXEL_TEMP_CONVERSION

        return retbuf

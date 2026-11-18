"""
AD5593 ADC/DAC/GPIO adapter
"""

import logging
import struct

from .device import Device
from .i2c import I2CDevice

log = logging.getLogger('DEV')

class AD5593Device(I2CDevice):
    """AD5593 ADC adapter"""
    MODEL_NAME = 'AD5593'
    TEMP_CHAN  = 8 # temperature channel index
    NCHANNELS  = TEMP_CHAN + 1 # 8 ADc inputs + temperature sensor
    I2C_ADDR   = 0x11 # for A0 pin pulled up

    BUF_EN    = 1 # ADC input buffered
    INT_REF   = 1 # Use internal reference
    RANGE     = 0 # ADC range (0: VREF = 2.5V, 1: 2xVREF = 5V)
    DAC_RANGE = 0 # DAC range (0: VREF = 2.5V, 1: 2xVREF = 5V)

    INP = {0: 0, 1: TEMP_CHAN} # main / alternative readout channels

    def adc_buf_enable(self, en: int) -> bool:
        """Enable/disable ADC input buffer"""
        if 0 <= en <= 1:
            self.BUF_EN = en
            return True
        return False

    def adc_int_ref_enable(self, en: int) -> bool:
        """Enable/disable ADC internal reference"""
        if 0 <= en <= 1:
            self.INT_REF = en
            return True
        return False

    def set_adc_range(self, range: int) -> bool:
        """Set ADC range (0: VREF = 2.5V, 1: 2xVREF = 5V)"""
        if 0 <= range <= 1:
            self.RANGE = range
            return True
        return False

    def set_dac_range(self, range: int) -> bool:
        """Set DAC range (0: VREF = 2.5V, 1: 2xVREF = 5V)"""
        if 0 <= range <= 1:
            self.DAC_RANGE = range
            return True
        return False

    def set_adc_inp(self, inp: int, chan: int) -> bool:
        """Set ADC input for the given readout channel"""
        if 0 <= inp < self.NCHANNELS and 0 <= chan < self.NCHANNELS:
            self.INP = self.INP.copy()
            self.INP[chan] = inp
            return True
        return False

    def set_param(self, key: str, val: str|int|float) -> bool:
        """
        Set device specific parameter.
        Returns True if key / val pair is recognized, False otherwise.
        """
        try:
            lkey = key.lower()
            if lkey == 'buf_en':
                return self.adc_buf_enable(int(val))
            if lkey == 'int_ref':
                return self.adc_int_ref_enable(int(val))
            if lkey == 'range':
                return self.set_adc_range(int(val))
            if lkey == 'inp':
                return self.set_adc_inp(int(val), 0)
            if lkey == 'ainp':
                return self.set_adc_inp(int(val), 1)
        except Exception as e:
            log.debug(e, exc_info=True)
            return False
        return super().set_param(key, val)

    def init(self, nchannels: int = NCHANNELS):
        """
        Initialize device setting the number of channels we are going to read.
        Should be called before the first query_raw call.
        """
        assert 0 < nchannels <= self.NCHANNELS
        super().init(nchannels)
        try:
            if not self.adapter.i2c_write(self.I2C_ADDR, [3, self.BUF_EN, (self.RANGE << 5) + (self.DAC_RANGE << 4)]) or \
               not self.adapter.i2c_write(self.I2C_ADDR, [4, 0, 0xff]) or \
               not self.adapter.i2c_write(self.I2C_ADDR, [2, 3, 0xff]) or \
               not self.adapter.i2c_write(self.I2C_ADDR, [11, self.INT_REF << 1, 0]):
                self.disconnected = True
        except Exception as e:
            self.disconnected = True
            log.debug(e, exc_info=True)

    def query_raw(self, idle_sleep=None) -> list[int]|None:
        """Query data from device"""
        try:
            # Read all channels 3 times and apply median filter to them
            data = self.adapter.i2c_write_read(self.I2C_ADDR, [0x40], 6*self.NCHANNELS)
            if data is None:
                return None
            for i in range(self.NCHANNELS):
                for j in range(3):
                    if data[2*(j*self.NCHANNELS + i)] >> 4 != i:
                        log.error('AD5593 returned invalid ADC data')
                        self.disconnected = True
                        return None
            res = [struct.unpack('>H', bytes(data[2*i:2*i+2]))[0] & 0xfff for i in range(3*self.NCHANNELS)]
            # Apply median filter to channel data. Note that even after filtering the data is rather noisy.
            return [sorted(res[i+self.NCHANNELS*j] for j in range(3))[1] for i in range(self.NCHANNELS)]
        except Exception as e:
            self.disconnected = True
            log.debug(e, exc_info=True)
            return None

    def get_value(self, data, channel: int = 0) -> float:
        """Retrieve floating point value for the given channel"""
        if not data:
            return Device.INVALID_VALUE
        inp = self.INP.get(channel, channel)
        assert 0 <= inp < self.NCHANNELS
        val = data[inp] * (1 + self.RANGE)
        if inp < self.TEMP_CHAN:
            return 2.5 * val / (1 << 12)
        return 25 + (val - 820) / 2.654

    def get_mode(self, data, channel: int = 0) -> str:
        """Returns measurement mode and units description string"""
        if not data:
            return ''
        inp = self.INP.get(channel, channel)
        return 'V' if inp < self.TEMP_CHAN else '°C'

    def setup_dac_pins(self, pin_mask: int) -> bool:
        """Set DAC pins given a 8-bit map"""
        return self.adapter.i2c_write(self.I2C_ADDR, [5, 0, pin_mask])

    def write_dac_pin(self, pin: int, value: int) -> bool:
        """Set DAC output level given the pin index"""
        return self.adapter.i2c_write(self.I2C_ADDR, [0x10 + pin, value >> 8, value & 0xff])

    def write_dac_pins(self, pin_values: list[tuple[int,int]]) -> bool:
        """Set DAC output level for several pins at once"""
        if not self.adapter.i2c_write(self.I2C_ADDR, [7, 0, 1]):
            return False
        for pin, value in pin_values:
            if not self.write_dac_pin(pin, value):
                return False
        return self.adapter.i2c_write(self.I2C_ADDR, [7, 0, 0])


if __name__ == '__main__':
    import sys
    import time
    from .i2c import FT260Adapter, CP2112Adapter, CH347TAdapter
    logging.basicConfig(level=logging.DEBUG) 
    if dev := AD5593Device.set_i2c_adapter(
            CP2112Adapter if '--2112' in sys.argv[1:]
                else CH347TAdapter if '--347' in sys.argv[1:]
                else FT260Adapter
      ).open():
        print(dev)
        with dev:
            i = 0
            dev.init()
            assert dev.setup_dac_pins(0xff)
            while dev.is_connected():
                assert dev.write_dac_pins([(j, 1024 if i % 8 == j % 8 else 0) for j in range(8)])
                assert dev.write_dac_pin((i + 1) % 8, 2048)
                data = dev.query_raw()
                print(data)
                print(dev.get_value(data, 0), dev.get_mode(data, 0), dev.get_value(data, 1), dev.get_mode(data, 1))
                time.sleep(.5)
                i += 1

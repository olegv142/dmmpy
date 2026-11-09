"""
ADS1220 24-bit ADC adapter
"""

import os
import sys
import time
import struct
import logging
from typing import Any

if __package__: sys.path.append(os.path.realpath(os.path.dirname(__file__)))

from spi import SPIDevice

log = logging.getLogger('DEV')

class ADS1220Adapter(SPIDevice):
    """ADS1220 ADC adapter"""
    MODEL_NAME = 'ADS1220'
    NCHANNELS  = 2
    TOUT       = .5
    IDLE_DELAY = .05
    # ADC configuration
    SPI_CS = 0 # CS line index
    RDY_IN = 6 # DRDY GPIO pin
    BYPASS = 0 # bypass PGA for unit PGA gain (gain < 8)
    VREF   = 0 # 2.048V internal
    FIR    = 2 # 50Hz reject
    IDAC   = 0 # IDAC disabled
    I1MUX  = 0 # disable IDAC channel 1
    I2MUX  = 0 # disable IDAC channel 2
    # Mux configuration values
    MUX_0_1 = 0  # AINP = AIN0, AINN = AIN1
    MUX_0_2 = 1  # AINP = AIN0, AINN = AIN2
    MUX_0_3 = 2  # AINP = AIN0, AINN = AIN3
    MUX_1_2 = 3  # AINP = AIN1, AINN = AIN2
    MUX_1_3 = 4  # AINP = AIN1, AINN = AIN3
    MUX_2_3 = 5  # AINP = AIN2, AINN = AIN3
    MUX_1_0 = 6  # AINP = AIN1, AINN = AIN0
    MUX_3_2 = 7  # AINP = AIN3, AINN = AIN2
    MUX_0_G = 8  # AINP = AIN0, AINN = AVSS
    MUX_1_G = 9  # AINP = AIN1, AINN = AVSS
    MUX_2_G = 10 # AINP = AIN2, AINN = AVSS
    MUX_3_G = 11 # AINP = AIN3, AINN = AVSS
    # system monitoring modes
    MUX_VREF_4 = 12 # (VREFPx – VREFNx) / 4 monitor (PGA bypassed)
    MUX_AVDD_4 = 13 # (AVDD – AVSS) / 4 monitor (PGA bypassed)
    MUX_SHORT  = 14 # AINP and AINN shorted to (AVDD + AVSS) / 2
    # The gain, mux and TS mode flag for two possible readout channels
    GAIN = [1, 1]
    MUX  = [MUX_0_1, MUX_3_G]
    TS   = [False, False]

    def _tx_cmd(self, cmd: int, data: list[int] = []) -> list[int]|None:
        """Transmits command padded with data and returns response or None on failure"""
        resp = self.adapter.spi_tx([cmd] + data)
        if resp is None:
            log.error('ADS1220 command %x failed', cmd)
            return None
        return resp

    def _send_cmd(self, cmd: int, data: list[int] = []) -> list[int]|None:
        """Same as _tx_cmd but does not return response to the command itself"""
        if resp := self._tx_cmd(cmd, data):
            return resp[1:]
        return None

    def _reset(self) -> bool:
        """Reset chip"""
        return self._send_cmd(6) is not None

    def _start(self) -> bool:
        """Start conversion"""
        return self._send_cmd(8) is not None

    def _wreg(self, reg: int, data: list[int]) -> bool:
        """Write one or more registers starting from the given reg index"""
        return self._send_cmd(0x40 + (reg << 2) + len(data) - 1, data) is not None

    def _rreg(self, reg: int, nregs: int) -> list[int]|None:
        """Read one or more registers starting from the given reg index"""
        return self._send_cmd(0x20 + (reg << 2) + nregs - 1, [0] * nregs)

    def _rdata(self) -> list[int]|None:
        """Read conversion data"""
        return self._send_cmd(0x10, [0]*3)

    def _cs(self, active):
        """Make CS signal active/inactive if it was not explicitly disabled"""
        if self.SPI_CS >= 0:
            self.adapter.spi_set_cs(self.SPI_CS, active)

    def exec_conversion_raw(self, mux: int, gain: int, ts: bool = False, idle_sleep=time.sleep) -> bytes|None:
        """Executes single conversion given MUX, gain value and TS mode flag. Returns raw bytes result or None on error."""
        if not self.is_connected():
            return None
        G = {1: 0, 2: 1, 4: 2, 8: 3, 16: 4, 32: 5, 64: 6, 128: 7}.get(gain, 0)
        PGA_BYPASS = 1 if self.BYPASS and gain < 8 else 0
        cfg = [
                (mux << 4) + (G << 1) + PGA_BYPASS, (ts << 1),
                (self.VREF << 6) + (self.FIR << 4) + self.IDAC,
                (self.I1MUX << 5) + (self.I2MUX << 2)
            ]
        try:
            self._cs(True)
            res = self._wreg(0, cfg) and self._rreg(0, len(cfg)) == cfg and self._start()
            self._cs(False)
            if not res:
                log.error('ADS1220 failed to start conversion')
                self.disconnected = True
                return None
            wait = self.TOUT
            while True:
                idle_sleep(self.IDLE_DELAY)
                nrdy = self.adapter.gpio_get_pin(self.RDY_IN)
                if nrdy is None:
                    log.error('ADS1220 failed to query conversion status')
                    self.disconnected = True
                    return None
                if not nrdy:
                    self._cs(True)
                    data = self._rdata()
                    self._cs(False)
                    if data is None:
                        log.error('ADS1220 failed to query conversion result')
                        self.disconnected = True
                    return data
                wait -= self.IDLE_DELAY
                if wait <= 0:
                    log.error('ADS1220 conversion timeout')
                    return None                
        except Exception as e:
            self.disconnected = True
            log.debug(e, exc_info=True)
            return None

    def exec_conversion(self, mux: int, gain: int, ts: bool = False, idle_sleep=time.sleep) -> float|None:
        """Executes single conversion given MUX, gain value and TS mode flag. Returns floating point value or None on error."""
        data = self.exec_conversion_raw(mux, gain, ts, idle_sleep)
        if not data:
            return None
        if not ts:
            val = struct.unpack('>i', bytes(data + [0]))[0]
            FSR = 2.048 / gain
            return val * FSR / (1 << 31)
        else:
            val = struct.unpack('>h', bytes(data[:2]))[0]
            return val * 0.03125 / 4

    def init(self, nchannels: int = 1):
        """
        Initialize device setting the number of channels we are going to read.
        Should be called before the first query_raw call.
        """
        assert 0 < nchannels <= self.NCHANNELS
        super().init(nchannels)
        try:
            if not self._reset() or not self.adapter.gpio_set_pin(self.RDY_IN, out_dir=False):
                log.error('ADS1220 initialization failed')
                self.disconnected = True
        except Exception as e:
            self.disconnected = True
            log.debug(e, exc_info=True)

    def query_raw(self, idle_sleep=time.sleep) -> Any|None:
        """Query data from device"""
        data = []
        for c in range(self.channels):
            val = self.exec_conversion(self.MUX[c], self.GAIN[c], self.TS[c], idle_sleep)
            if val is None:
                return None
            data.append(val)
        return data

    def get_value(self, data, channel: int = 0) -> float:
        """Retrieve floating point value for the given channel"""
        assert 0 <= channel < self.channels
        if not data:
            return Device.INVALID_VALUE
        return data[channel]

    def get_mode(self, data, channel: int = 0) -> str:
        """Returns measurement mode and units description string"""
        if not data:
            return ''
        return 'V'

if __name__ == '__main__':
    import time
    logging.basicConfig(level=logging.DEBUG)
    if dev := ADS1220Adapter.open():
        print(dev)
        with dev:
            dev.init()
            while True:
                print(
                    dev.exec_conversion(dev.MUX_AVDD_4, 1),
                    dev.exec_conversion(dev.MUX_SHORT, 4),
                    dev.exec_conversion(0, 0, ts=True)
                )

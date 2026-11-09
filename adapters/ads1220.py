"""
ADS1220 24-bit ADC adapter
"""

import os
import sys
import time
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
    RDY_IN = 7 # DRDY GPIO pin
    BYPASS = 0 # bypass PGA for unit PGA gain (gain < 8)
    VREF   = 0 # 2.048V internal
    FIR    = 2 # 50Hz reject
    IDAC   = 0 # IDAC disabled
    I1MUX  = 0 # disable IDAC channel 1
    I2MUX  = 0 # disable IDAC channel 2

    def _tx_cmd(self, cmd: int, data: list[int] = []) -> list[int]|None:
        """Transmits command padded with data and returns response or None on failure"""
        resp = self.adapter.spi_tx([cmd] + data)
        if resp is None:
            log.debug('ADS1220 command %x failed', cmd)
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
        if self.SPI_CS >= 0:
            self.adapter.spi_set_cs(self.SPI_CS, active)

    def exec_conversion_raw(self, mux: int, gain: int, ts: bool = False, idle_sleep=time.sleep) -> bytes|None:
        """Executes single conversion given MUX and GAIN and TS mode flag. Returns raw bytes result or None on error."""
        if not self.is_connected():
            return None
        PGA_BYPASS = 1 if self.BYPASS and gain < 3 else 0
        cfg = [
                (mux << 4) + (gain << 1) + PGA_BYPASS, (ts << 1),
                (self.VREF << 6) + (self.FIR << 4) + self.IDAC,
                (self.I1MUX << 5) + (self.I2MUX << 2)
            ]
        try:
            self._cs(True)
            res = self._wreg(0, cfg) and self._rreg(0, len(cfg)) == cfg and self._start()
            self._cs(False)
            if not res:
                log.debug('ADS1220 failed to start conversion')
                self.disconnected = True
                return None
            wait = self.TOUT
            while True:
                idle_sleep(self.IDLE_DELAY)
                nrdy = self.adapter.gpio_get_pin(self.RDY_IN)
                if nrdy is None:
                    log.debug('ADS1220 failed to query conversion status')
                    self.disconnected = True
                    return None
                if not nrdy:
                    self._cs(True)
                    data = self._rdata()
                    self._cs(False)
                    if data is None:
                        log.debug('ADS1220 failed to query conversion result')
                        self.disconnected = True
                    return data
                wait -= self.IDLE_DELAY
                if wait <= 0:
                    log.debug('ADS1220 conversion timeout')
                    return None                
        except Exception as e:
            self.disconnected = True
            log.debug(e, exc_info=True)
            return None

    def init(self, nchannels: int = 1):
        """
        Initialize device setting the number of channels we are going to read.
        Should be called before the first query_raw call.
        """
        assert 0 < nchannels <= self.NCHANNELS
        super().init(nchannels)
        try:
            if not self._reset() or not self.adapter.gpio_set_pin(self.RDY_IN, out_dir=False):
                log.debug('ADS1220 initialization failed')
                self.disconnected = True
        except Exception as e:
            self.disconnected = True
            log.debug(e, exc_info=True)

if __name__ == '__main__':
    import time
    logging.basicConfig(level=logging.DEBUG) 
    if dev := ADS1220Adapter.open():
        print(dev)
        with dev:
            dev.init()
            while True:
                print(dev.exec_conversion_raw(0, 0, True))

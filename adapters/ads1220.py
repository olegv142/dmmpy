"""
ADS1220 24-bit ADC adapter
See https://www.ti.com/lit/ds/symlink/ads1220.pdf
"""

import time
import struct
import logging
from typing import Any

from .spi import SPIDevice

log = logging.getLogger('DEV')

class ADS1220Device(SPIDevice):
    """ADS1220 ADC adapter"""
    MODEL_NAME  = 'ADS1220'
    MAX_CLK_MHZ = 6.6 # max clock frequency in MHz
    NCHANNELS   = 2
    TOUT        = .5
    IDLE_DELAY  = .05 # 20 sps

    # Reference voltage source
    VREF_INT  = 0 # Internal 2.048V reference selected
    VREF_EXT0 = 1 # External reference selected using dedicated REFP0 and REFN0 inputs
    VREF_EXT1 = 2 # External reference selected using AIN0/REFP1 and AIN3/REFN1 inputs
    VREF_AVDD = 3 # Analog supply (AVDD – AVSS) used as reference
    # FIR filter settings
    FIR_PLAIN   = 0 # No 50Hz or 60Hz rejection
    FIR_50_60Hz = 1 # Simultaneous 50Hz and 60Hz rejection
    FIR_50Hz    = 2 # 50Hz rejection only
    FIR_60Hz    = 3 # 60Hz rejection only
    # IDAC settings
    IDAC_Off    = 0 # Off
    IDAC_10uA   = 1 # 10µA
    IDAC_50uA   = 2 # 50µA
    IDAC_100uA  = 3 # 100µA
    IDAC_250uA  = 4 # 250µA
    IDAC_500uA  = 5 # 500µA
    IDAC_1000uA = 6 # 1000µA
    IDAC_1500uA = 7 # 1500µA
    # IMUX settings
    IMUX_Off  = 0 # disabled
    IMUX_IN0  = 1 # connected to AIN0/REFP1
    IMUX_IN1  = 2 # connected to AIN1
    IMUX_IN2  = 3 # connected to AIN2
    IMUX_IN3  = 4 # connected to AIN3/REFN1
    IMUX_REFP = 5 # connected to REFP0
    IMUX_REFN = 6 # connected to REFN0
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
    # special mux values
    MUX_TS = MUX_RESERVED  = 15 # Activates TS mode
    MUX_0_1_CHOP           = 16 # Switching polarity of IN0 and IN1 to compensate for offset
    MUX_2_3_CHOP = MUX_MAX = 17 # Switching polarity of IN2 and IN3 to compensate for offset
    # ADC configuration
    SPI_CS = 0 # CS line index
    RDY_IN = 6 # DRDY GPIO pin
    VREF   = VREF_INT # 2.048V internal
    FIR    = FIR_50Hz # 50Hz reject
    IDAC   = IDAC_Off # IDAC disabled
    IMUX   = [IMUX_Off, IMUX_Off]
    BYPASS = 1 # bypass PGA if gain < 8
    TURBO  = 0 # enables turbo mode (2x rate)
    BCS    = 0 # enables 10µA, burn-out current sources

    # The gain and mux for two possible readout channels
    GAIN = [1, 1]
    MUX  = [MUX_0_1, MUX_3_G]

    # Special modes
    OC = 0 # offset compensation
    OS = 0 # oversampling

    def set_gain(self, gain: int, chan: int = 0) -> bool:
        """Set PGA gain for the given readout channel"""
        assert 0 <= chan < self.NCHANNELS
        if gain in (1, 2, 4, 8, 16, 32, 64, 128):
            self.GAIN[chan] = gain
            return True
        return False

    def set_mux(self, mux: int, chan: int = 0) -> bool:
        """Set input MUX configuration for the given readout channel"""
        assert 0 <= chan < self.NCHANNELS
        if 0 <= mux <= self.MUX_MAX:
            self.MUX[chan] = mux
            return True
        return False

    def set_cs_pin(self, cs_idx: int) -> bool:
        """Set index of CS adapter output. The -1 disables using CS signal."""
        if -1 <= cs_idx < 2:
            self.SPI_CS = cs_idx
            return True
        return False

    def set_rdy_pin(self, rdy_pin: int) -> bool:
        """Set data ready GPIO pin"""
        if 0 <= rdy_pin < 8:
            self.RDY_IN = rdy_pin
            return True
        return False

    def set_vref(self, vref: int) -> bool:
        """Set reference voltage source"""
        if 0 <= vref < 4:
            self.VREF = vref
            return True
        return False

    def set_fir(self, fir: int) -> bool:
        """Set FIR mode"""
        if 0 <= fir < 4:
            self.FIR = fir
            return True
        return False

    def set_idac(self, idac: int) -> bool:
        """Set IDAC configuration"""
        if 0 <= idac < 8:
            self.IDAC = idac
            return True
        return False

    def set_imux(self, ch: int, imux: int) -> bool:
        """Set I1MUX / I2MUX configuration"""
        assert 0 <= ch < 2
        if 0 <= imux < 7:
            self.IMUX[ch] = imux
            return True
        return False

    def set_turbo(self, en: int) -> bool:
        """Set turbo mode"""
        if 0 <= en <= 1:
            self.TURBO = en
            return True
        return False

    def set_bcs(self, en: int) -> bool:
        """Enable burn-out current sources"""
        if 0 <= en <= 1:
            self.BCS = en
            return True
        return False

    def set_offset_compensation(self, en: int) -> bool:
        """Set offset compensation mode"""
        if 0 <= en <= 1:
            self.OC = en
            return True
        return False

    def set_oversampling(self, cnt: int) -> bool:
        """Set oversampling mode"""
        if cnt >= 0:
            self.OS = cnt
            return True
        return False

    def set_param(self, key: str, val: str|float) -> bool:
        """
        Set device specific parameter.
        Returns True if key / val pair is recognized, False otherwise.
        """
        lkey = key.lower()
        try:
            if lkey == 'gain':
                return self.set_gain(int(val), 0)
            if lkey == 'again':
                return self.set_gain(int(val), 1)
            if lkey == 'mux':
                return self.set_mux(int(val), 0)
            if lkey == 'amux':
                return self.set_mux(int(val), 1)
            if lkey == 'cs':
                return self.set_cs_pin(int(val))
            if lkey == 'rdy':
                return self.set_rdy_pin(int(val))
            if lkey == 'vref':
                return self.set_vref(int(val))
            if lkey == 'fir':
                return self.set_fir(int(val))
            if lkey == 'idac':
                return self.set_idac(int(val))
            if lkey == 'i1mux':
                return self.set_imux(0, int(val))
            if lkey == 'i2mux':
                return self.set_imux(1, int(val))
            if lkey == 'turbo':
                return self.set_turbo(int(val))
            if lkey == 'bcs':
                return self.set_bcs(int(val))
            if lkey == 'oc':
                return self.set_offset_compensation(int(val))
            if lkey == 'os':
                return self.set_oversampling(int(val))
        except Exception as e:
            log.debug(e, exc_info=True)
            return False
        return super().set_param(key, val)

    def _tx_cmd(self, cmd_data: list[int] = []) -> list[int]|None:
        """Transmits command padded with data and returns response or None on failure"""
        resp = self.adapter.spi_tx(cmd_data)
        if resp is None:
            log.error('ADS1220 command %x failed', cmd_data[0])
            return None
        return resp

    def _send_cmd(self, cmd: int, data: list[int] = []) -> list[int]|None:
        """Same as _tx_cmd but does not return response to the command itself"""
        if resp := self._tx_cmd([cmd] + data):
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

    def _write_verify_reg(self, reg: int, data: list[int]) -> bool:
        nregs = len(data)
        res = self._tx_cmd(
                [0x40 + (reg << 2) + nregs - 1] + data +
                [0x20 + (reg << 2) + nregs - 1] + [0] * nregs
            )
        if res is None:
            return False
        return res[2+nregs:] == data

    def _rdata(self) -> list[int]|None:
        """Read conversion data"""
        return self._send_cmd(0x10, [0]*3)

    def _cs(self, active):
        """Make CS signal active/inactive if it was not explicitly disabled"""
        if self.SPI_CS >= 0:
            self.adapter.spi_set_cs(self.SPI_CS, active)

    def conversion_delay(self):
        return self.IDLE_DELAY / (self.TURBO + 1)

    def exec_conversion_raw(self, mux: int, gain: int, idle_sleep=time.sleep) -> bytes|None:
        """Executes single conversion given MUX and gain value. Returns raw bytes result or None on error."""
        if not self.is_connected():
            return None
        G = {1: 0, 2: 1, 4: 2, 8: 3, 16: 4, 32: 5, 64: 6, 128: 7}.get(gain, 0)
        cfg = [
                (mux << 4) + (G << 1) + (self.BYPASS if gain else 0),
                (self.TURBO << 4) + ((mux == self.MUX_TS) << 1) + self.BCS,
                (self.VREF << 6) + (self.FIR << 4) + self.IDAC,
                (self.IMUX[0] << 5) + (self.IMUX[1] << 2)
            ]
        try:
            self._cs(True)
            res = self._write_verify_reg(0, cfg) and self._start()
            self._cs(False)
            if not res:
                log.error('ADS1220 failed to start conversion')
                self.disconnected = True
                return None
            wait = self.TOUT
            while True:
                delay = self.conversion_delay()
                idle_sleep(delay)
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
                wait -= delay
                if wait <= 0:
                    log.error('ADS1220 conversion timeout')
                    return None                
        except Exception as e:
            self.disconnected = True
            log.debug(e, exc_info=True)
            return None

    def exec_conversion_once(self, mux: int, gain: int, idle_sleep=time.sleep) -> float|None:
        """Executes single conversion given MUX and gain value. Returns floating point value or None on error."""
        data = self.exec_conversion_raw(mux, gain, idle_sleep)
        if not data:
            return None
        if mux != self.MUX_TS:
            val = struct.unpack('>i', bytes(data + [0]))[0]
            real_gain = 1 if (mux == self.MUX_VREF_4 or mux == self.MUX_AVDD_4) else max(gain, 1)
            FSR = 2.048 / real_gain
            return val * FSR / (1 << 31)
        else:
            val = struct.unpack('>h', bytes(data[:2]))[0]
            return val * 0.03125 / 4

    def exec_conversion_compensate_offset(self, mux: int, gain: int = 0, idle_sleep=time.sleep) -> float|None:
        """Execute conversion with offset compensation. Returns floating point value or None on error."""
        val = self.exec_conversion_once(mux, gain, idle_sleep)
        off = self.exec_conversion_once(self.MUX_SHORT,
                    0 if (mux == self.MUX_VREF_4 or mux == self.MUX_AVDD_4) else gain,
                    idle_sleep
                )
        if val is None or off is None:
            return None
        return val - off

    def exec_conversion_chopped(self, mux: int, gain: int = 0, idle_sleep=time.sleep) -> float|None:
        """Execute conversion with chopping (swapping input polarity)"""
        if mux == self.MUX_0_1_CHOP:
            inputs = [self.MUX_0_1, self.MUX_1_0]
        elif mux == self.MUX_2_3_CHOP:
            inputs = [self.MUX_2_3, self.MUX_3_2]
        else:
            log.error('invalid MUX=%d', mux)
            return None
        vals = [self.exec_conversion_once(m, gain, idle_sleep) for m in inputs]
        if any(val is None for val in vals):
            return None
        return (vals[0] - vals[1]) / 2

    def exec_conversion(self, mux: int, gain: int = 0, idle_sleep=time.sleep) -> float|None:
        """Execute conversion(s) with optional oversampling. Returns floating point value or None on error."""
        total, cnt = 0, max(1, self.OS)
        for _ in range(cnt): # oversampling loop
            if mux > self.MUX_TS:
                val = self.exec_conversion_chopped(mux, gain, idle_sleep)
            elif not self.OC or mux == self.MUX_TS:
                val = self.exec_conversion_once(mux, gain, idle_sleep)
            else:
                val = self.exec_conversion_compensate_offset(mux, gain, idle_sleep)
            if val is None:
                return None
            total += val
        return total / cnt

    def init(self, nchannels: int = 1):
        """
        Initialize device setting the number of channels we are going to read.
        Should be called before the first query_raw call.
        """
        assert 0 < nchannels <= self.NCHANNELS
        super().init(nchannels)
        try:
            if not self.adapter.gpio_set_pin(self.RDY_IN, out_dir=False):
                log.error('SPI adapter initialization failed')
                self.disconnected = True
                return
            self._cs(True)
            res = self._reset()
            self._cs(False)
            if not res:
                log.error('ADS1220 initialization failed')
                self.disconnected = True
                return
            if self.SPI_CS < 0:
                time.sleep(.1) # no ideas why
        except Exception as e:
            self.disconnected = True
            log.debug(e, exc_info=True)

    def query_raw(self, idle_sleep=time.sleep) -> Any|None:
        """Query data from device"""
        data = []
        for c in range(self.channels):
            val = self.exec_conversion(self.MUX[c], self.GAIN[c], idle_sleep)
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
        return 'V' if self.MUX[channel] != self.MUX_TS else '°C'

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    if dev := ADS1220Device.open():
        print(dev)
        with dev:
            dev.init()
            while dev.is_connected():
                print(
                    dev.exec_conversion(dev.MUX_AVDD_4),
                    dev.exec_conversion(dev.MUX_SHORT, 4),
                    dev.exec_conversion(dev.MUX_TS)
                )

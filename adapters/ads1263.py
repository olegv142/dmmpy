"""
ADS1263 32-bit ADC adapter
See https://www.ti.com/lit/gpn/ADS1263
"""

import time
import struct
import logging

from .spi import SPIDevice
from .params import IParam, init_defaults, parse

log = logging.getLogger('DEV')

class ADS1263Device(SPIDevice):
    """ADS1263 ADC adapter"""
    MODEL_NAME  = 'ADS1263'
    MAX_CLK_MHZ = 8 # max clock frequency in MHz
    NCHANNELS   = 2
    TOUT        = 4
    IDLE_DELAY  = .05

    # Chop modes
    CHOP_DIS        = 0 # Input chop and IDAC rotation disabled (default)
    CHOP_INPUT      = 1 # Input chop enabled
    CHOP_IDAC       = 2 # IDAC rotation enabled
    CHOP_INPUT_IDAC = 3 # Input chop and IDAC rotation enabled
    # Filter modes
    FILTER_Sinc1 = 0 # Sinc1 mode
    FILTER_Sinc2 = 1 # Sinc2 mode
    FILTER_Sinc3 = 2 # Sinc3 mode
    FILTER_Sinc4 = 3 # Sinc4 mode
    FILTER_FIR   = 4 # FIR mode (default)
    # MUX modes
    MUX_AIN0   = 0  # AIN0
    MUX_AIN1   = 1  # AIN1
    MUX_AIN2   = 2  # AIN2
    MUX_AIN3   = 3  # AIN3
    MUX_AIN4   = 4  # AIN4
    MUX_AIN5   = 5  # AIN5
    MUX_AIN6   = 6  # AIN6
    MUX_AIN7   = 7  # AIN7
    MUX_AIN8   = 8  # AIN8
    MUX_AIN9   = 9  # AIN9
    MUX_AINCOM = 10 # AINCOM
    MUX_TS     = 11 # Temperature sensor monitor
    MUX_AVDD_4 = 12 # Analog power supply monitor (AVDD/4)
    MUX_DVDD_4 = 13 # Digital power supply monitor (DVDD/4)
    MUX_TDAC   = 14 # TDAC test signal
    MUX_OPEN   = 15 # Float (open connection)
    # IDAC MUX modes
    IMUX_AIN0   = 0  # AIN0
    IMUX_AIN1   = 1  # AIN1
    IMUX_AIN2   = 2  # AIN2
    IMUX_AIN3   = 3  # AIN3
    IMUX_AIN4   = 4  # AIN4
    IMUX_AIN5   = 5  # AIN5
    IMUX_AIN6   = 6  # AIN6
    IMUX_AIN7   = 7  # AIN7
    IMUX_AIN8   = 8  # AIN8
    IMUX_AIN9   = 9  # AIN9
    IMUX_AINCOM = 10 # AINCOM
    IMUX_Off    = 11 # No Connection (default)
    # IDAC magnitude
    IDAC_Off  = 0  # off (default)
    IDAC_50   = 1  # 50 µA
    IDAC_100  = 2  # 100 µA
    IDAC_250  = 3  # 250 µA
    IDAC_500  = 4  # 500 µA
    IDAC_750  = 5  # 750 µA
    IDAC_1000 = 6  # 1000 µA
    IDAC_1500 = 7  # 1500 µA
    IDAC_2000 = 8  # 2000 µA
    IDAC_2500 = 9  # 2500 µA
    IDAC_3000 = 10 # 3000 µA
    # TDAC settings
    TDAC_P2_5 = 9     # AVDD / 2 + AVDD / 2.5
    TDAC_P5   = 8     # AVDD / 2 + AVDD / 5
    TDAC_P10  = 7     # AVDD / 2 + AVDD / 10
    TDAC_P20  = 6     # AVDD / 2 + AVDD / 20
    TDAC_P40  = 5     # AVDD / 2 + AVDD / 40
    TDAC_P80  = 4     # AVDD / 2 + AVDD / 80
    TDAC_P160 = 3     # AVDD / 2 + AVDD / 160
    TDAC_P320 = 2     # AVDD / 2 + AVDD / 320
    TDAC_P640 = 1     # AVDD / 2 + AVDD / 640
    TDAC_MID  = 0     # AVDD / 2
    TDAC_N640 = 0x11  # AVDD / 2 - AVDD / 640
    TDAC_N320 = 0x12  # AVDD / 2 - AVDD / 320
    TDAC_N160 = 0x13  # AVDD / 2 - AVDD / 160
    TDAC_N80  = 0x14  # AVDD / 2 - AVDD / 80
    TDAC_N40  = 0x15  # AVDD / 2 - AVDD / 40
    TDAC_N20  = 0x16  # AVDD / 2 - AVDD / 20
    TDAC_N10  = 0x17  # AVDD / 2 - AVDD / 10
    TDAC_N5   = 0x18  # AVDD / 2 - AVDD / 5
    TDAC_N2_5 = 0x19  # AVDD / 2 - AVDD / 2.5
    # Sensor bias settings
    SB_Off = 0 # No sensor bias current or resistor (default)
    SB_0_5 = 1 # 0.5-µA sensor bias current
    SB_2   = 2 # 2-µA sensor bias current
    SB_10  = 3 # 10-µA sensor bias current
    SB_50  = 4 # 50-µA sensor bias current
    SB_200 = 5 # 200-µA sensor bias current
    SB_10M = 6 # 10-MΩ resistor

    # Status bits
    STA_ADC2   = 1 << 7 # ADC2 data new since the last ADC2 read operation
    STA_ADC1   = 1 << 6 # ADC1 data new since the last ADC1 read operation
    STA_EXTCLK = 1 << 5 # ADC clock is external
    STA_REF_ALM    = 1 << 4 # ADC1 Low Reference Alarm (VREF < 0.4V)
    STA_PGAL_ALM   = 1 << 3 # ADC1 PGA Output Low Alarm. The bit is set if the absolute voltage of either PGA output is less than VAVSS + 0.2 V.
    STA_PGAH_ALM   = 1 << 2 # ADC1 PGA Output High Alarm. The bit is set if the absolute voltage of either PGA output is greater than VAVDD – 0.2 V.
    STA_PGAD_ALM   = 1 << 1 # ADC1 PGA Differential Output Alarm. The bit is set if the PGA differential output voltage exceeds +105% FS or –105% FS.
    STA_RESET      = 1      # This bit is the same as the RESET bit of the POWER register

    ADC_PARAMS = [ # Current ADC configuration
        # The following parameters can be set either by setting the corresponding attribute of the device instance,
        # or by specifying a key=value pair that is ultimately passed to the self.set_param(key, val) function. Note that
        # in the second scenario the gain values will be translated to the corresponding device specific codes like 8 -> 3 for gain=8.
        # For example one can use the following parameters to observe the noise floor o the ADC1 (~10nV): muxp=14,muxn=14,chop=1,gain=32,filter=3
        IParam('SPI_CS', 0, 1),     # CS line index
        IParam('VBIAS',  0, 1),     # if set to 1 enables the internal level shift voltage to the AINCOM <- (VAVDD + VAVSS)/2
        IParam('INTREF', 0, 1, 1),  # internal reference (2.5V) enabled
        IParam('REFREV', 0, 1),     # if set to 1 reverses the ADC1 reference multiplexer output polarity
        IParam('RUNMODE', 0, 1),    # 0 - continuous conversion (default), 1 - one shot conversion
        IParam('CHOP',   0, 3),     # chop mode
        IParam('DELAY',  0, 11),    # additional conversion delay (see datasheet)
        IParam('FILTER', 0, 4, FILTER_FIR), # filter mode
        IParam('SBADC',  0, 1),     # sensor bias connected to ADC1 mux out (default)
        IParam('SBPOL',  0, 1),     # sensor bias pull-up mode (AINP pulled high, AINN pulled low) (default)
        IParam('SBMAG',  0, 6),     # no sensor bias current or resistor (default)
        IParam('BYPASS1', 0, 1, 1), # PGA bypass for gain == 1
        IParam('GAIN', 0, 5, mapping = {1: 0, 2: 1, 4: 2, 8: 3, 16: 4, 32: 5}), # PGA gain
        IParam('DR', 0, 15),        # data rate (0: 2.5, 1: 5, 2: 10, 3: 16.6, 4: 20, 5: 50, 6: 60, 7: 100, 8: 400, 9: 1200, 10: 2400, 11: 4800, 12: 7200, 13: 14400, 14: 19200, 15: 38400)
        IParam('MUXP', 0, 15, 0),   # positive input connection (AIN0)
        IParam('MUXN', 0, 15, 1),   # negative input connection (AIN1)
        IParam('I1MUX', 0, 11, IMUX_Off), # IDAC1 connection
        IParam('I2MUX', 0, 11, IMUX_Off), # IDAC2 connection
        IParam('I1DAC', 0, 10),     # IDAC1 magnitude
        IParam('I2DAC', 0, 10),     # IDAC2 magnitude
        IParam('RMUXP', 0, 4),      # REFP is connected to internal 2.5 V reference
        IParam('RMUXN', 0, 4),      # REFN is connected to internal 2.5 V reference
        IParam('TDAC_OUTP', 0, 1),  # test DAC positive output not connected (AIN6 if set to 1)
        IParam('TDAC_OUTN', 0, 1),  # test DAC negative output not connected (AIN7 if set to 1)
        IParam('TDAC_MAGP', 0, 25), # test DAC positive output set to 2.5V
        IParam('TDAC_MAGN', 0, 25), # test DAC negative output set to 2.5V
        IParam('DR2', 0, 3),        # ADC2 data rate (0: 10, 1: 100, 2: 400, 3: 800)
        IParam('REF2', 0, 4),       # ADC2 uses internal reference (2.5V)
        IParam('GAIN2', 0, 7, mapping = {1: 0, 2: 1, 4: 2, 8: 3, 16: 4, 32: 5, 64: 6, 128: 7}), # ADC2 gain
        IParam('MUXP2', 0, 15, MUX_TS), # ADC2 positive input connection (temperature sensor)
        IParam('MUXN2', 0, 15, MUX_TS), # ADC2 negative input connection (temperature sensor)
    ]

    def __init__(self, *args):
        super().__init__(*args)
        self.device_id = None
        self.adc1_mode_cfg = None
        self.adc1_mux_cfg = None
        self.adc2_cfg = None
        init_defaults(self, self.ADC_PARAMS)

    def _mk_power_cfg(self) -> list[int]:
        """Returns POWER register"""
        return [(self.VBIAS << 1) + self.INTREF]

    def _mk_adc1_mode_cfg(self) -> list[int]:
        """Returns MODE0,1,2, IPMUX registers (addr 3..6)"""
        return [
            (self.REFREV << 7) + (self.RUNMODE << 6) + (self.CHOP << 4) + self.DELAY,
            (self.FILTER << 5) + (self.SBADC << 4) + (self.SBPOL << 3) + self.SBMAG,
            ((self.BYPASS1 if self.GAIN == 0 else 0) << 7) + (self.GAIN << 4) + self.DR,
            (self.MUXP << 4) + self.MUXN
        ]

    def _mk_adc1_aux_cfg(self) -> list[int]:
        """Returns IDACMUX, IDACMAG, REFMUX registers (addr 13..15)"""
        return [
            (self.I2MUX << 4) + self.I1MUX,
            (self.I2DAC << 4) + self.I1DAC,
            (self.RMUXP << 3) + self.RMUXN
        ]

    def _mk_adc2_cfg(self) -> list[int]:
        """Returns ADC2CFG, ADC2MUX registers (addr 21, 22)"""
        return [
            (self.DR2 << 6) + (self.REF2 << 3) + self.GAIN2,
            (self.MUXP2 << 4) + self.MUXN2
        ]

    def _mk_tdac_cfg(self) -> list[int]:
        """Returns TDACP,TDACN (addr 16..17)"""
        return [
            (self.TDAC_OUTP << 7) + self.TDAC_MAGP,
            (self.TDAC_OUTN << 7) + self.TDAC_MAGN
        ]

    def _tx_cmd(self, cmd_data: list[int] = []) -> list[int]|None:
        """Transmits command padded with data and returns response or None on failure"""
        resp = self.adapter.spi_tx(cmd_data)
        if resp is None:
            log.error('ADS1263 command %x failed', cmd_data[0])
            return None
        return resp

    def _reset(self) -> bool:
        """Reset chip"""
        return self._tx_cmd([6]) is not None

    def _cs(self, active):
        """Make CS signal active/inactive if it was not explicitly disabled"""
        assert self.SPI_CS >= 0
        # Using CS is mandatory. Although ADS1263 has SPI clock timeout feature, it is not
        # enabled by default. The clock counter may have non-zero value after power up so that
        # it will be impossible to communicate with device without CS signal that resets it.
        self.adapter.spi_set_cs(self.SPI_CS, active)

    def _wreg(self, addr: int, data: list[int], validate: bool = True) -> bool:
        """Write registers starting from the given address and validate by reading them back"""
        n = len(data)
        cmds = [0x40 + addr, n - 1] + data
        if validate: # read back registers
            cmds += [0x20 + addr, n - 1] + [0] * n
        resp = self._tx_cmd(cmds)
        if resp is None:
            return False
        if validate and resp[-n:] != data:
            log.error('ADS1263 failed to update %d reg(s) at addr %d', n, addr)
            return False
        return True

    def _rreg(self, addr: int, count: int = 1) -> list[int]|None:
        """Read registers starting from the given address"""
        resp = self._tx_cmd([0x20 + addr, count - 1] + [0] * count)
        return resp[2:] if resp is not None else None

    def wreg(self, addr: int, data: list[int], validate: bool = True) -> bool:
        """Write registers starting from the given address"""
        self._cs(True)
        res = self._wreg(addr, data, validate)
        self._cs(False)
        return res

    def rreg(self, addr: int, count: int = 1) -> list[int]|None:
        """Read registers starting from the given address"""
        self._cs(True)
        data = self._rreg(addr, count)
        self._cs(False)
        return data

    def _adc1_init(self, start: bool = True) -> bool:
        """Writes configuration registers and optionally start ADC1"""
        adc1_mode_cfg = self._mk_adc1_mode_cfg()
        adc1_aux_cfg  = self._mk_adc1_aux_cfg()
        n1, n2 = len(adc1_mode_cfg), len(adc1_aux_cfg)
        cmds = [
            0xa, # stop ADC1
            0x40 + 3, n1 - 1,  # WREG 3..6
            *adc1_mode_cfg,
            0x40 + 13, n2 - 1, # WREG 13..17
            *adc1_aux_cfg,
            0x20 + 3, n1 - 1,  # RREG 3..6
            *[0] * n1,
            0x20 + 13, n2 - 1, # RREG 13..17
            *[0] * n2
        ]
        if start:
            cmds.append(0x8) # start ADC1
        resp = self._tx_cmd(cmds)
        if resp is None:
            return False
        off1 = 1 + 2 + n1 + 2 + n2 + 2
        off2 = off1 + n1 + 2
        if resp[off1:off1+n1] != adc1_mode_cfg or resp[off2:off2+n2] != adc1_aux_cfg:
            log.error('ADS1263 ADC1 configuration failed')
            return False
        self.adc1_mode_cfg = adc1_mode_cfg
        self.adc1_aux_cfg  = adc1_aux_cfg
        return True

    def _adc2_init(self, start: bool = True) -> bool:
        """Writes configuration registers and optionally start ADC2"""
        adc2_cfg = self._mk_adc2_cfg()
        n = len(adc2_cfg)
        cmds = [
            0xe, # stop ADC2
            0x40 + 21, n - 1, # WREG 21..22
            *adc2_cfg,
            0x20 + 21, n - 1, # RREG 21..22
            *[0] * n
        ]
        if start:
            cmds.append(0xc) # start ADC2
        resp = self._tx_cmd(cmds)
        if resp is None:
            return False
        off = 1 + 2 + n + 2
        if resp[off:off+n] != adc2_cfg:
            log.error('ADS1263 ADC2 configuration failed')
            return False
        self.adc2_cfg = adc2_cfg
        return True

    def adc_init(self, adci: int, start: bool = True) -> bool:
        """Initializes and optionally starts ADC. Returns True on success, False on error."""
        assert 0 <= adci <= 1
        self._cs(True)
        res = self._adc2_init(start) if adci else self._adc1_init(start)
        self._cs(False)
        return res

    def _is_pga1_bypassed(self) -> bool:
        """Return True if ADC1 PGA is bypassed, False otherwise"""
        return self.adc1_mode_cfg is not None and (self.adc1_mode_cfg[2] & (1 << 7)) != 0

    def _is_adc1_run_continuous(self) -> bool:
        """Returns True if ADC1 is running in continuous mode, False otherwise"""
        return self.adc1_mode_cfg is not None and (self.adc1_mode_cfg[0] & (1 << 6)) == 0

    def _query_data(self, adci: int) -> list[int]|None:
        """
        Read data given ADC index (0 or 1).
        Returns 4 bytes data list on success, None on error
        or empty list if there were no new data since last read.
        """
        assert 0 <= adci <= 1
        resp = self._tx_cmd([0x12 + adci * 2] + [0]*6)
        if resp is None:
            return None
        sta, data, csum = resp[1], resp[2:6], resp[6]
        if sta & self.STA_RESET:
            log.warning('ADS1263 was reset unexpectedly')
        if not self._is_pga1_bypassed():
            if sta & self.STA_PGAL_ALM:
                log.warning('ADS1263 PGA output too low (negative input is below the limit)')
            if sta & self.STA_PGAH_ALM:
                log.warning('ADS1263 PGA output too high (positive input is above the limit)')
        if sta & self.STA_PGAD_ALM:
            log.warning('ADS1263 PGA output overflow detected (gain is too high)')
        if csum != (sum(data) + 0x9b) & 0xff:
            if any(data) or csum:
                # before first conversion completion it just returns all zero response
                log.error('ADS1263 returns bad data checksum: %s %s %s', sta, data, csum)
            return None
        if adci == 0 and not (sta & self.STA_ADC1):
            return []
        if adci == 1 and not (sta & self.STA_ADC2):
            return []
        return data

    def _adc_restart(self, adci: int) -> bool:
        """Start/restart ADC given its index"""
        assert 0 <= adci <= 1
        # send stop + start commands
        return self._tx_cmd(([0xa, 0x8], [0xe, 0xc])[adci]) is not None

    def adc_restart(self, adci: int = 0) -> bool:
        """Start/restart ADC given its index"""
        self._cs(True)
        res = self._adc_restart(adci)
        self._cs(False)
        return res

    def query_data(self, adci: int, restart: bool = True) -> list[int]|None:
        """
        Read data given ADC index (0 or 1) and optionally start
        next conversion if not running in continuous mode.
        Returns 4 bytes data list on success, None on error
        or empty list if there were no new data since last read.
        """
        self._cs(True)
        data = self._query_data(adci)
        if data and adci == 0 and not self._is_adc1_run_continuous():
            self._adc_restart(0)
        self._cs(False)
        return data

    def query_result(self, adci: int, restart: bool = True, idle_sleep=time.sleep) -> int|None:
        """
        Wait for the next conversion result given ADC index (0 or 1)
        and optionally start next conversion if not running in continuous mode.
        Returns integer value on success, None on error.
        """
        if not self.is_connected():
            return None
        wait = self.TOUT
        while True:
            if data := self.query_data(adci, restart):
                val = struct.unpack('>i', bytes(data))[0]
                if adci > 0:
                    val >>= 8 # drop zero padding byte
                return val
            idle_sleep(self.IDLE_DELAY)
            wait -= self.IDLE_DELAY
            if wait <= 0:
                log.error('ADS1220 ADC%d conversion timeout', adci + 1)
                return None                

    def set_param(self, key: str, val: str|int|float) -> bool:
        """
        Set device specific parameter.
        Returns True if key / val pair is recognized, False otherwise.
        """
        if parse(self, self.ADC_PARAMS, key, val):
            return True
        return super().set_param(key, val)

    def set_gain(self, val: int) -> bool:
        """Set ADC1 gain (1..32)"""
        return parse(self, self.ADC_PARAMS, 'GAIN', val)

    def set_gain2(self, val: int) -> bool:
        """Set ADC2 gain (1..128)"""
        return parse(self, self.ADC_PARAMS, 'GAIN2', val)

    def init(self, nchannels: int = 1):
        """
        Initialize device setting the number of channels we are going to read.
        Should be called before the first query_raw call.
        """
        assert 0 < nchannels <= self.NCHANNELS
        super().init(nchannels)
        try:
            self._cs(True)
            if self.device_id is None:
                dev_id = self._rreg(0)
                if dev_id is not None:
                    self.device_id = dev_id[0]
                    dev_name = {0: 'ADS1262', 1: 'ADS1263'}.get(self.device_id >> 5, 'unknown device')
                    log.debug('connected to %s, rev.%d' % (dev_name, self.device_id & 0x1f))
            res = self.device_id is not None \
                and self._reset() \
                and self._wreg(1, self._mk_power_cfg()) \
                and self._wreg(16, self._mk_tdac_cfg()) \
                and self._adc1_init()
            if nchannels > 1:
                res = res and self._adc2_init()
            self._cs(False)
            if not res:
                log.error('ADS1263 initialization failed')
                self.disconnected = True
                return
        except Exception as e:
            self.disconnected = True
            log.debug(e, exc_info=True)

    def query_raw(self, idle_sleep=time.sleep) -> Any|None:
        """Query data from device"""
        data = []
        try:
            for adci in range(self.channels):
                val = self.query_result(adci, idle_sleep=idle_sleep)
                if val is None:
                    return None
                if adci == 0:
                    val = 2.5 * val / (1 << 31)
                    val /= (1 << self.GAIN)
                    if self.MUXP == self.MUX_TS and self.MUXN == self.MUX_TS:
                        val = (1e3 * val - 122.4) / .42 + 25
                else:
                    val = 2.5 * val / (1 << 23)
                    val /= (1 << self.GAIN2)
                    if self.MUXP2 == self.MUX_TS and self.MUXN2 == self.MUX_TS:
                        val = (1e3 * val - 122.4) / .42 + 25
                data.append(val)
            return data
        except Exception as e:
            self.disconnected = True
            log.debug(e, exc_info=True)
            return None

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
        ts_mode = (self.MUXP  == self.MUX_TS and self.MUXN  == self.MUX_TS) if channel == 0 else \
                  (self.MUXP2 == self.MUX_TS and self.MUXN2 == self.MUX_TS)
        if not ts_mode:
            return 'V'
        else:
            return '°C'

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    if dev := ADS1263Device.open():
        print(dev)
        with dev:
            # dev.RUNMODE = 1 # one short
            dev.init(2)
            while dev.is_connected():
                data = dev.query_raw()
                print(
                    dev.get_value(data, 0), dev.get_mode(data, 0),
                    dev.get_value(data, 1), dev.get_mode(data, 1),
                )

"""
ADS1263 self test
"""

import logging
from .ads1263 import ADS1263Device

log = logging.getLogger('DEV')

AVDD = None
NADC = 2 # set to 1 for ADS1262

def test_avdd(dev):
    global AVDD
    # connect inputs to AVDD divider
    dev.MUXP = dev.MUXN = dev.MUX_AVDD_4
    dev.init()
    data = dev.query_raw()
    AVDD = 4 * dev.get_value(data)

def test_tdac(dev, tdacp, tdacn, tdacr):
    # connect inputs to TDAC
    dev.MUXP = dev.MUXN = dev.MUXP2 = dev.MUXN2 = dev.MUX_TDAC
    dev.TDAC_MAGP = tdacp
    dev.TDAC_MAGN = tdacn
    tdacv = tdacr * AVDD
    gain, max_gain = 1, 2.4 / tdacv
    max_err = 0
    while gain <= max_gain:
        if dev.set_gain(gain) + dev.set_gain2(gain) == 0:
            break
        dev.init(NADC)
        data = dev.query_raw()
        for i in range(NADC):
            v = dev.get_value(data, i)
            err = abs((v - tdacv) / tdacv)
            if err > max_err:
                max_err = err
            log.debug(' ADC%d gain=%d, err=%.2g',
                i + 1, (1 << dev.GAIN, 1 << dev.GAIN2)[i], err)
        gain *= 2
    return max_err

if __name__ == '__main__':
    import sys
    if '-v' in sys.argv[1:]:
        logging.basicConfig(level=logging.DEBUG)
    if dev := ADS1263Device.open():
        test_avdd(dev)
        max_err = 0
        with dev:
            for tdacp, tdacn, tdacr in [
                (dev.TDAC_P2_5, dev.TDAC_N2_5, .8),
                (dev.TDAC_P5,   dev.TDAC_N5,   .4),
                (dev.TDAC_P10,  dev.TDAC_N10,  .2),
                (dev.TDAC_P20,  dev.TDAC_N20,  1/10),
                (dev.TDAC_P40,  dev.TDAC_N40,  1/20),
                (dev.TDAC_P80,  dev.TDAC_N80,  1/40),
                (dev.TDAC_P160, dev.TDAC_N160, 1/80),
                (dev.TDAC_P320, dev.TDAC_N320, 1/160),
                (dev.TDAC_P640, dev.TDAC_N640, 1/320)
            ]:
                max_err = max(max_err, test_tdac(dev, tdacp, tdacn, tdacr))
        print ('AVDD=%.3fV, max error = %.2g%%' % (AVDD, max_err * 100))

"""
CH347T USB HID adapter SPI API
See https://www.wch-ic.com/downloads/CH347DS1_PDF.html
The following repositories were used as a reference:
https://github.com/i2cy/CH347-HIDAPI
https://github.com/serfreeman1337/go-ch347
"""

import os
import sys
import logging
import struct

if __package__: sys.path.append(os.path.realpath(os.path.dirname(__file__)))

from ch347t import find_device, READ_TOUT_MS, MAX_PKT

log = logging.getLogger('DEV')

SPIMode0 = 0 # CPOL = 0, CPHA = 0 (Clock idles low; data sampled on the first/rising edge)
SPIMode1 = 1 # CPOL = 0, CPHA = 1 (Clock idles low; data sampled on the second/falling edge)
SPIMode2 = 2 # CPOL = 1, CPHA = 0 (Clock idles high; data sampled on the first/falling edge)
SPIMode3 = 3 # CPOL = 1, CPHA = 1 (Clock idles high; data sampled on the second/rising edge)

SPIClock0 = 0 # 60 MHz
SPIClock1 = 1 # 30 MHz
SPIClock2 = 2 # 15 MHz
SPIClock3 = 3 # 7.5 MHz
SPIClock4 = 4 # 3.75 MHz
SPIClock5 = 5 # 1.875 MHz
SPIClock6 = 6 # 937.5 KHz
SPIClock7 = 7 # 468.75 KHz

SPIBitOrderMSB = 0
SPIBitOrderLSB = 1

CmdSPISetup    = 0xc0
CmdSPISetCS    = 0xc1
CmdSPITransfer = 0xc2

def spi_init(
        dev, mode: int, bit_order: int = SPIBitOrderMSB, clk: int = SPIClock2,
        cs_active_high: tuple[bool,bool] = (False, False) # active low by default
    ):
    """Initialize SPI interface"""
    dev.set_nonblocking(False)

    p = [0x1d, 0x00] # packet length = 29
    p += [CmdSPISetup, 0x1a, 0x00] # command and length
    # next fields are not well documented
    p += [0x00, 0x00, 0x04, 0x01, 0x00, 0x00]

    if mode == SPIMode0:
        p += [0x00, 0x00, 0x00, 0x00]
    elif mode == SPIMode1:
        p += [0x00, 0x00, 0x01, 0x00]
    elif mode == SPIMode2:
        p += [0x02, 0x00, 0x00, 0x00]
    elif mode == SPIMode3:
        p += [0x02, 0x00, 0x01, 0x00]

    p += [0x00, 0x02]
    p += [clk<<3, 0x00]
    p += [bit_order<<7, 0x00, 0x07, 0x00, 0x00, 0x00]

    # 24 byte - default data
    # Output MISO data during MOSI read ?
    p += [0xff]

    # 25 byte - CS Polarity
    # 0x80 - active high CS0
    # 0x40 - active high CS1
    p += [cs_active_high[0] * 0x80 + cs_active_high[1] * 0x40]
    p += [0x00, 0x00, 0x00, 0x00]

    dev.write([0] + p)
    p = dev.read(MAX_PKT, timeout_ms=READ_TOUT_MS)
    if not p:
        log.error('SPI setup packet response timeout')
        return False
    if p[2] != CmdSPISetup or p[3] != 0x01:
        log.error('bad SPI setup packet response: %s', p)
        return False
    return True

def spi_tx(dev, data: list[int]) -> list[int]|None:
    """
    Transmit data. Returns data received on success or None on failure.
    Variations of this functions for transmit or receive only are possible but
    for small amount of data they don't make sense. Transmitting more than
    MAX_TX_SIZE bytes of data should be performed by several calls of this
    function.
    """
    dlen = len(data)
    pkt = list(struct.pack("<BHBH", 0, dlen + 3, CmdSPITransfer, dlen)) + data
    assert len(pkt) <= MAX_PKT
    dev.write(pkt)
    frame = dev.read(MAX_PKT, timeout_ms=READ_TOUT_MS)
    if not frame:
        log.error('SPI transfer response timeout')
        return None
    frame_len, cmd_id, payload_len = struct.unpack("<HBH", bytes(frame[:5]))
    if cmd_id != CmdSPITransfer:
        log.error('bad SPI transfer response: %s', frame)
        return None
    if payload_len != dlen:
        log.error('bad SPI transfer response length: expect %d, got %d', dlen, payload_len)
        return None
    return frame[5:5 + payload_len]

MAX_TX_SIZE = MAX_PKT - 6

def spi_set_cs(dev, cs: int, active: bool):
    """
    Set CS active/inactive. Note that active CS level is low by default. It can be changed
    to high by providing cs_active_high tuple to spi_init function.
    """
    p = [0x0d, 0x00, CmdSPISetCS, 0x0a, 0x00] + [0] * 10
    pos = 5 + 5*cs
    p[pos] = 0x80 if active else 0xc0
    dev.write([0] + p)

if __name__ == '__main__':
    import time
    import random
    dev = find_device()
    assert dev
    try:
        while True:
            assert spi_init(dev,
                    random.randint(SPIMode0, SPIMode3),
                    random.randint(SPIBitOrderMSB, SPIBitOrderLSB),
                    random.randint(SPIClock0, SPIClock7)
                )
            for i in range(1000):
                n = random.randint(1, MAX_TX_SIZE)
                chunk = list(random.randbytes(n))
                cs = (i // 8) % 2
                spi_set_cs(dev, cs, True)
                rx = spi_tx(dev, chunk)
                spi_set_cs(dev, cs, False)
                assert chunk == rx
                print('.', end='', flush=True)
            print()
    except KeyboardInterrupt:
        pass

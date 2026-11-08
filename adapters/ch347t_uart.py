"""
CH347T UART API
Only UART1 is available in HID mode
"""

import os
import sys
import time
import logging
import struct
import threading

if __package__: sys.path.append(os.path.realpath(os.path.dirname(__file__)))

from ch347t import MAX_PKT

log = logging.getLogger('DEV')

def find_uart():
    """Find device and open it"""
    import device
    from ch347t import DEVICE_VID, DEVICE_PID
    devs = device.find_hid_devices(DEVICE_VID, DEVICE_PID, ifnum=0)
    if not devs:
        return None
    if len(devs) > 1:
        log.warning('%d devices found', len(devs))
        return None
    path = devs[0]['path']
    dev = device.hid.device()
    try:
        dev.open_path(path)
    except Exception:
        log.error('failed to open CH347T UART at %s', path)
        return None
    return dev

PARITY_NONE = 0
PARITY_ODD  = 1
PARITY_EVEN = 2

STPBIT_ONE  = 0
STPBIT_ONEH = 1
STPBIT_TWO  = 2

CmdUARTSetup = 0xCB

def uart_init(dev, baud_rate: int, parity: int = PARITY_NONE, stop_bits: int = STPBIT_ONEH, tout_ms: int = 32):
    """
    Initialize UART. The tout_ms parameter is the report transmit timeout. The report with chunk of received data
    will be transmitted to the host if buffer is full or time elapsed since last receive exceeds this timeout.
    This is important since OS report queue associated with open handle has capacity of 32 reports only. Therefore
    sending received data ASAP without timeout may decrease receiving queue down to 32 symbols.
    Unfortunately there is no hardware flow control in hid mode.
    """
    hdr = [0, CmdUARTSetup, 8, 0]
    payload = list(struct.pack('<IBBBB', baud_rate, stop_bits, parity, 8, tout_ms))
    dev.send_feature_report(hdr + payload)
    dev.set_nonblocking(True)

def uart_tx(dev, data: list[int]|bytes):
    """
    Transmit chunk of data. Note that the adapter buffer for transmission has capacity of 4k.
    Unlike VCP interface the HID does not reject packets if the buffer is full. Therefore, be careful not
    to overflow the transmit buffer.
    """
    dlen = len(data)
    assert 0 < dlen <= 510
    dev.write([0, dlen & 0xff, (dlen >> 8)] + data)

def uart_rx(dev, tout=0) -> list[int]:
    """
    Receive UART data
    """
    data = []
    IDLE_WAIT = .01
    while True:
        if pkt := dev.read(MAX_PKT):
            data_len = pkt[0] + (pkt[1] << 8)
            data += pkt[2:2+data_len]
        elif tout <= 0:
            return data
        else:
            time.sleep(IDLE_WAIT)
            tout -= IDLE_WAIT

class UARTReceiver:
    """Class reading received data in separate thread"""
    IDLE_WAIT = .01

    def __init__(self, dev):
        self.dev    = dev
        self.worker = None
        self.cancel = None
        self.lock = threading.Lock()
        self.buff = []
        self.failed = False

    def start(self):
        assert self.worker is None
        self.cancel = False
        self.worker = threading.Thread(target=self._work, daemon=True)
        self.worker.start()

    def stop(self):
        assert self.worker is not None
        self.cancel = True
        self.worker.join()
        self.worker = None

    def _work(self):
        while not self.cancel:
            try:
                chunk = uart_rx(self.dev)
                with self.lock:
                    self.buff += chunk
            except Exception as e:
                log.debug(e, exc_info=True)
                self.failed = True
                return
            time.sleep(self.IDLE_WAIT)

    def get(self) -> list[int]:
        assert self.worker is not None
        with self.lock:
            buff, self.buff = self.buff, []
            return buff

    def is_running(self) -> bool:
        return self.worker is not None

    def is_failed(self) -> bool:
        return self.failed

    def __enter__(self):
        """Start receiver on entering 'with' block"""
        if not self.is_running():
            self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Stop receiver on exiting 'with' block"""
        if self.is_running():
            self.stop()

if __name__ == '__main__':
    import random

    dev = find_uart()
    assert dev
    uart_init(dev, 115200, PARITY_ODD)
    receiver = UARTReceiver(dev)
    receiver.start()
    BUNCH = 8
    try:
        while True:
            sent = []
            for _ in range(BUNCH):
                n = random.randrange(1, 511)
                chunk = list(random.randbytes(n))
                uart_tx(dev, chunk)
                sent += chunk
            time.sleep(.5)
            if receiver.is_failed():
                print('\naborted')
                break
            received = receiver.get()
            assert sent == received
            print('.', end='', flush=True)
    except KeyboardInterrupt:
        receiver.stop()
        assert not receiver.is_failed()

"""
Bases classes for devices using SPI communication bus
"""

import os
import sys
import logging
from typing import Any

if __package__: sys.path.append(os.path.realpath(os.path.dirname(__file__)))

from device import Device, HIDMixin
import ch347t
import ch347t_spi

log = logging.getLogger('DEV')

class SPIAdapter:
    """Base class for SPI adapters"""
    # The following should be defined in subclasses
    DEVICE_VID = None
    DEVICE_PID = None
    DEVICE_IFN = None
    NAME = None

    def __init__(self, dev, mode: int):
        """Initialize SPI bus given the clock mode (0..3)"""
        raise NotImplementedError()

    def spi_tx(self, data: list[int]) -> list[int]|None:
        """Transmits data. Returns data received or None if failed."""
        raise NotImplementedError()

    def spi_set_cs(self, cs: int, active: bool):
        """Set CS output state given its index"""
        raise NotImplementedError()

    def gpio_set_pin(self, pin: int, out_dir: bool, level: bool = False) -> bool|None:
        """
        Setup direction and level of the particular pin given its index.
        Setting level is ignored for the input pins (out_dir = False).
        Returns True on success, None on error.
        """
        raise NotImplementedError()

    def gpio_get_pin(self, pin: int) -> bool|None:
        """
        Read the particular GPIO pin level given its index. Returns True if the level is high,
        False if low, None in case of error.
        """
        raise NotImplementedError()

class CH347TSPIAdapter(SPIAdapter):
    """CH347T SPI adapter class"""
    DEVICE_VID = ch347t.DEVICE_VID
    DEVICE_PID = ch347t.DEVICE_PID
    DEVICE_IFN = ch347t.DEVICE_IFN
    NAME = 'CH347T'

    def __init__(self, dev, mode: int):
        """Initialize SPI bus given the clock mode (0..3)"""
        assert 0 <= mode < 4
        self.dev = dev
        ch347t_spi.spi_init(self.dev, mode)

    def spi_tx(self, data: list[int]) -> list[int]|None:
        """Transmits data. Returns data received or None if failed."""
        return ch347t_spi.spi_tx(self.dev, data)

    def spi_set_cs(self, cs: int, active: bool):
        """Set CS output state given its index"""
        ch347t_spi.spi_set_cs(self.dev, cs, active)

    def gpio_set_pin(self, pin: int, out_dir: bool, level: bool = False) -> bool|None:
        """
        Setup direction and level of the particular pin given its index.
        Setting level is ignored for the input pins (out_dir = False).
        Returns True on success, None on error. Note that the response from the adapter
        contains the actual level of the output pin after the new level was set up.
        In case it differs from the level set up the function returns False. This may be
        expected due to the low impedance or capacitive load. 
        """
        return ch347t.gpio_set_pin(self.dev, pin, out_dir, level)

    def gpio_get_pin(self, pin: int) -> bool|None:
        """
        Read the particular GPIO pin level given its index. Returns True if the level is high,
        False if low, None in case of error. This function returns the actual pin level
        even in case the pin is configured as output. So it will return zero bit for short
        circuited pin even if it was set to high level by gpio_set_pin.
        """
        return ch347t.gpio_get_pin(self.dev, pin)


class SPIDevice(Device, HIDMixin):
    """Base class for devices using SPI communication bus"""
    SPI_ADAPTER_TYPE = CH347TSPIAdapter # the only SPI adapter supported for now
    DEVICE_VID = SPI_ADAPTER_TYPE.DEVICE_VID
    DEVICE_PID = SPI_ADAPTER_TYPE.DEVICE_PID
    DEVICE_IFN = SPI_ADAPTER_TYPE.DEVICE_IFN
    SPI_MODE = 1 # SPI clock mode

    def __init__(self, *args):
        super().__init__(*args)
        self.adapter = None

    @classmethod
    def set_spi_adapter(cls, T):
        cls.SPI_ADAPTER_TYPE = T
        cls.DEVICE_VID = T.DEVICE_VID
        cls.DEVICE_PID = T.DEVICE_PID
        cls.DEVICE_IFN = T.DEVICE_IFN
        return cls

    @classmethod
    def model_info(cls) -> str:
        """The implementation may redefine this method to provide extended information"""
        return cls.MODEL_NAME + '-' + cls.SPI_ADAPTER_TYPE.NAME

    def init(self, nchannels: int = 0):
        """
        Initialize device setting the number of channels we are going to read.
        Should be called before the first query_raw call.
        """
        self.channels = nchannels
        if self.adapter is None:
            self.adapter = self.SPI_ADAPTER_TYPE(self.dev, self.SPI_MODE)

    def get_channel(self, data: Any) -> int | None:
        """
        Returns the index of the channel the data belongs to. If there are multiple
        channels contained in the data, it returns -1. It there are no data, it returns None.
        """
        if not data:
            return None
        return -1 if self.channels > 1 else 0

    @classmethod
    def attach_to_i2c_adapter(cls, master: 'I2CDevice') -> 'SPIDevice':
        """
        Create device attaching it to already initialized I2C adapter.
        Should be called before init method. The master device init()
        should be already called at this point.
        """
        assert master.adapter is not None
        assert cls.SPI_ADAPTER_TYPE.NAME == master.adapter.NAME
        dev = cls(master.dev, master.path)
        return dev

"""
ADS1263 GPIO functions.

Note that GPIO levels are related to analog power supply.
AVDD is logic 1, AVSS is logic 0. The middle point is the threshold.
So don't expect them to match digital domain levels. While using bipolar
analog power supply the GPIO levels also become bipolar.
"""

from .ads1263 import ADS1263Device

def gpio_pin_to_mux(pin_idx : int) -> int:
    """Converts pin index (0..7) to input or idac multiplexer code"""
    assert 0 <= pin_idx < 8
    # GPIO pins are AIN3 .. AIN9, AINCOM
    return ADS1263Device.MUX_AIN3 + pin_idx

def gpio_setup(dev: ADS1263Device, conn_mask: int, inp_mask: int) -> bool:
    """
    Setups GPIO pins. The pin will be used as GPIO if it has corresponding bit set in conn_mask.
    The pin will used as an input if it has corresponding bit set in inp_mask.
    Returns True on success, False on error.
    """
    return dev.wreg(0x12, [conn_mask, inp_mask])

def gpio_write(dev: ADS1263Device, out_bits: int) -> bool:
    """Sets GPIO pin levels given the bitmask. Returns True on success, False on error."""
    return dev.wreg(0x14, [out_bits], validate = False)

def gpio_read(dev: ADS1263Device) -> int|None:
    """
    Reads GPIO inputs. The pins configured as outputs will be read as zero.
    Returns 8 bit mask on success, None on error.
    """
    reg = dev.rreg(0x14)
    if reg is None:
        return None
    return reg[0]


if __name__ == '__main__':
    # GPIO self test
    if dev := ADS1263Device.open():
        dev.VBIAS = 1 # AINCOM <- (VAVDD + VAVSS)/2
        dev.MUXN = dev.IMUX_AINCOM
        for pin in range(7): # test outputs
            pin_mask = 1 << pin
            dev.MUXP = gpio_pin_to_mux(pin)
            dev.init(1)
            assert gpio_setup(dev, pin_mask, 0)
            assert gpio_write(dev, 0)
            assert dev.adc_restart()
            v = dev.get_value(dev.query_raw())
            print('#%d 0 -> %.3f' % (pin, v))
            assert v < -2.4
            assert gpio_write(dev, pin_mask)
            assert dev.adc_restart()
            v = dev.get_value(dev.query_raw())
            print('#%d 1 -> %.3f' % (pin, v))
            assert v > 2.4
        dev.SBMAG = dev.SB_10 # 10µA input bias
        for pin in range(7): # test inputs
            pin_mask = 1 << pin
            dev.MUXP = gpio_pin_to_mux(pin)
            dev.SBPOL = 0 # pull up positive input
            dev.init(1)
            assert gpio_setup(dev, pin_mask, pin_mask)
            assert dev.adc_restart()
            v = dev.get_value(dev.query_raw())
            inp_bits = gpio_read(dev)
            inp = (inp_bits & pin_mask) != 0
            print('#%d %.3f -> %s' % (pin, v, inp))
            assert inp is True
            assert inp_bits == pin_mask
            dev.SBPOL = 1 # reverse bias polarity
            dev.init(1)
            assert gpio_setup(dev, pin_mask, pin_mask)
            assert dev.adc_restart()
            v = dev.get_value(dev.query_raw())
            inp_bits = gpio_read(dev)
            inp = (inp_bits & pin_mask) != 0
            print('#%d %.3f -> %s' % (pin, v, inp))
            assert inp is False
            assert inp_bits == 0

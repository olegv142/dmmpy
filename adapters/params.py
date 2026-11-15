"""
Helper functions for parameters handling
"""

import logging
from typing import Any
from dataclasses import dataclass

log = logging.getLogger('DEV')

class ParamTag:
    def cast(self, val: str|int|float):
        raise NotImplementedError()

@dataclass
class IParam(ParamTag):
    """Integer parameter descriptor"""
    name : str # capitalized name
    vmin : int # min value
    vmax : int # max value
    vdef : int = 0 # def value
    mapping : dict[int, int] = None

    def cast(self, val: str|int|float) -> int:
        ival = int(val)
        if self.mapping:
            return self.mapping[ival]
        return ival

def init_defaults(obj: Any, tags: list[ParamTag]):
    """Assign defaults as obj attributes"""
    for t in tags:
        assert t.vmin <= t.vdef <= t.vmax
        setattr(obj, t.name, t.vdef)

def parse(obj: Any, tags: list[ParamTag], key: str, val: str|int|float) -> bool:
    """Parse key, value pair. Assign value as obj attribute and returns True on success or False on error."""
    try:
        uname = key.upper()
        for t in tags:
            if t.name == uname:
                cval = t.cast(val)
                if cval < t.vmin or t.vmax < cval:
                    return False
                setattr(obj, t.name, cval)
                return True
    except Exception as e:
        log.debug(e, exc_info=True)
        return False

if __name__ == '__main__':
    # logging.basicConfig(level=logging.DEBUG)
    class Test:
        pass
    t = Test()
    pars = [IntTag('A', 0, 1, 1), IntTag('B', 0, 2)]
    init_defaults(t, pars)
    assert t.A == 1
    assert t.B == 0
    assert not parse(t, pars, 'a', 2)
    assert parse(t, pars, 'b', '2')
    assert parse(t, pars, 'a', 0)
    assert not parse(t, pars, 'a', 'x')
    assert t.A == 0
    assert t.B == 2

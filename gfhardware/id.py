"""
Copyright 2020-2026 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
https://community.openglow.org
SPDX-License-Identifier:    MIT

Machine identity, derived from the i.MX6 OCOTP fuses.

The fuse words are read through nvmem (imx-ocotp). The NXP fsl_otp sysfs
path (/sys/fsl_otp/HW_OCOTP_*) is tried first, so the module also works on a
factory kernel. /etc/gfhardware-id.conf can override or provide the values
for a machine whose fuses cannot be read.

nvmem layout (imx-ocotp): one 32-bit word per fuse, bank-major, 8 words per
bank -> byte offset = (bank * 8 + word) * 4.
  HW_OCOTP_MAC0   = bank 4 word 2 -> word 34, offset 136
  HW_OCOTP_SRK0-7 = bank 3 words 0-7 -> words 24-31, offset 96
"""
import configparser
import os
import struct

from gfhardware._common import read_file

_NVMEM_PATH = '/sys/bus/nvmem/devices/imx-ocotp0/nvmem'
_LEGACY_MAC0 = '/sys/fsl_otp/HW_OCOTP_MAC0'
_LEGACY_SRK = '/sys/fsl_otp/HW_OCOTP_SRK%d'
_CONF_PATH = '/etc/gfhardware-id.conf'

_MAC0_OFFSET = (4 * 8 + 2) * 4
_SRK_OFFSET = (3 * 8 + 0) * 4


def _nvmem_words(offset: int, count: int) -> list:
    with open(_NVMEM_PATH, 'rb') as f:
        f.seek(offset)
        data = f.read(count * 4)
    if len(data) != count * 4:
        raise IOError('short nvmem read at offset %d' % offset)
    return list(struct.unpack('<%dI' % count, data))


def _conf_value(key: str) -> str:
    cfg = configparser.ConfigParser()
    if not cfg.read(_CONF_PATH):
        raise IOError('%s not found' % _CONF_PATH)
    return cfg['MACHINE'][key]


def serial() -> int:
    if os.path.exists(_LEGACY_MAC0):
        return int(read_file(_LEGACY_MAC0), 16)
    try:
        return _nvmem_words(_MAC0_OFFSET, 1)[0]
    except (OSError, IOError):
        return int(_conf_value('serial'), 0)


_HOSTNAME_ALPHABET = 'BCDFGHJKMQRTVWXY2346789'


def hostname_for(serial_num) -> str:
    """The factory serial -> hostname encoding (base 23 over the consonant
    alphabet, up to six characters, split XXX-YYY). Integer arithmetic
    only - float division corrupts digits on large serials - and no
    trailing dash on short serials. The one Python implementation; the C
    daemon mirrors it."""
    enc = ''
    serial_num = int(serial_num)
    while serial_num > 0 and len(enc) < 6:
        enc = _HOSTNAME_ALPHABET[serial_num % 23] + enc
        serial_num //= 23
    return '{}-{}'.format(enc[:3], enc[3:]) if len(enc) > 3 else enc


def hostname() -> str:
    return hostname_for(serial())


def password() -> str:
    if os.path.exists(_LEGACY_SRK % 0):
        return "".join("%08x" % int(read_file(_LEGACY_SRK % w), 16)
                       for w in range(8))
    try:
        return "".join("%08x" % w for w in _nvmem_words(_SRK_OFFSET, 8))
    except (OSError, IOError):
        return _conf_value('password')

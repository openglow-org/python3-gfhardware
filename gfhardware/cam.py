"""
Copyright 2026 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
https://community.openglow.org
SPDX-License-Identifier:    MIT

Glowforge camera capture for the mainline imx-media stack.

The factory firmware drove a single NXP mxc_v4l2_capture node (/dev/video0)
that exposed the sensor controls directly and selected the lid/head camera via
a private V4L2_CID_GLOWFORGE_SEL_CAM control. ForgeFIRM runs mainline
imx-media, which is a media-controller graph: links and per-pad formats must be
configured before the capture node will stream, the sensor controls live on the
sensor subdev (not the capture node), camera selection is the video-mux input,
and there is no flash-LED control -- the bed/head is lit with the lid_led / head
white_led instead.

This module configures that pipeline (via media-ctl / v4l2-ctl) and the lighting
(via gfhardware.leds), then hands the streaming capture node to the _cam C
extension which grabs one frame, debayers it, and JPEG-encodes it. Both cameras
share one video-mux -> MIPI CSI-2 -> IPU CSI path to the 'ipu1_csi0 capture'
video node; only the mux input and the illumination LED differ.

Two sensors ship in the field -- the 5 MP OV5648 and, in "HD" machines, the 8 MP
OV8856 -- and they differ in geometry, bit depth and control set, so the format,
the frame size and the manual exposure/gain come from the profile of whichever
driver bound on that camera's I2C bus (see _SENSORS).

PRIVACY GATE: capture() refuses unless the lid is closed. forgectrl enforces the
same rule on the path everything normally takes; this is the second enforcement
point, for the callers that reach V4L2 directly -- the cloud client's fallback
when forgectrl is unreachable, and the capture utility. Both cameras are gated,
and the check fails closed (see gfhardware.switches.lid_closed).
"""
import logging
import os
import re
import subprocess
import time

from gfhardware._common import LOGGER_NAME, SYSFS_GF_BASE, load_installed_extension, read_file, write_attr
from gfhardware.switches import lid_closed

if os.getenv('REMOTE_DEBUG'):
    grab = load_installed_extension('_cam').grab
else:
    from gfhardware._cam import grab

logger = logging.getLogger(LOGGER_NAME)


class LidOpen(PermissionError):
    """Raised when a capture is requested with the lid open.

    A distinct type so callers can tell the privacy refusal apart from a
    hardware failure: there is nothing to retry and nothing is wrong with the
    machine -- the lid has to be closed.
    """

# Camera selection (identical to the factory cam module's constants).
GFCAM_LID = 0
GFCAM_HEAD = 1

# Frame size of the 5 MP OV5648, kept as the module's historical constants.
# They are the default only: capture() reports and uses the size of the sensor
# that actually bound (see sensor_geometry()).
GFCAM_WIDTH = 2592
GFCAM_HEIGHT = 1944

# imx-media capture entity (ipu1_csi0). The /dev/videoN node is resolved from
# the entity name at capture time: imx-media registers several video nodes from
# modules, so the number depends on probe order and a hardcoded /dev/videoN
# breaks whenever it shifts.
_CAPTURE_ENTITY = 'ipu1_csi0 capture'

# Brief settle after switching the illumination LED on. The lamp is driven via
# the raw PIC register (instant, no fade), so this is just to ensure it is on
# before the grab starts streaming.
_SETTLE_S = 0.2

# The factory mirrored both camera images (HFLIP=1). The ov5648 HFLIP *register*
# breaks imx-media CSI capture (frames never complete -> ipu1_csi0 EOF timeout),
# so the sensor is left un-flipped and the mirror is applied in software by the
# _cam grabber instead. VFLIP was 0 at the factory, so no software vflip needed.
_HFLIP = 1

# Per-camera wiring. Both sensors feed the same video-mux; selection is which
# mux sink link is enabled, and illumination is that camera's LED.
#   bus    -- I2C bus the sensor sits on (its media entity is resolved by
#             address, so this works whichever sensor is fitted)
#   muxpad -- video-mux sink pad the sensor is wired to
#   lamp   -- sysfs attribute for that camera's illumination LED (raw PIC /
#             head register, 10-bit PWM, read+write, no fade)
_CAMERAS = {
    GFCAM_LID:  {'bus': 0, 'muxpad': 0, 'lamp': SYSFS_GF_BASE + 'pic/lid_led'},
    GFCAM_HEAD: {'bus': 3, 'muxpad': 1, 'lamp': SYSFS_GF_BASE + 'head/white_led'},
}


def _ctrls_ov5648(subdev, exposure, gain):
    """OV5648 manual controls. The auto-clusters must be switched to manual
    before the manual values take effect. Exposure is in 1/16-line units and
    cannot exceed the frame length: the 2592x1944 mode is 1984 lines (vts), so
    the usable ceiling is ~(1984-margin)*16 -- beyond it the sensor cannot
    integrate within the frame and returns a blank frame. Gain is in 1/16 steps
    (16 = 1x)."""
    _v4l2_ctl('-d', subdev,
              '-c', 'auto_exposure=1',
              '-c', 'gain_automatic=0',
              '-c', 'white_balance_automatic=0')
    _v4l2_ctl('-d', subdev,
              '-c', 'exposure=%d' % exposure,
              '-c', 'gain=%d' % gain,
              '-c', 'red_balance=1100',
              '-c', 'blue_balance=1400',
              '-c', 'horizontal_flip=0',
              '-c', 'vertical_flip=0')


def _ctrls_ov8856(subdev, exposure, gain):
    """OV8856 manual controls. The driver exposes a different set: exposure
    counts whole lines (it shifts into the 1/16-line register itself) and is
    capped by the frame length -- 2482 lines in the 3264x2448 mode; analog
    gain is 128 = 1x, and there are no auto-exposure, auto-gain or white-balance
    controls to switch off, so the sensor comes up manual and its white balance
    stays uncorrected."""
    _v4l2_ctl('-d', subdev,
              '-c', 'exposure=%d' % exposure,
              '-c', 'analogue_gain=%d' % gain,
              '-c', 'digital_gain=1024',
              '-c', 'horizontal_flip=0',
              '-c', 'vertical_flip=0')


# What differs between the sensors the machine ships with, keyed by the driver
# name in the media entity ('ov5648 0-0036'). One image covers both.
#   width/height -- the frame the pipeline is configured for
#   mbus         -- media-bus code set on every pad of the active path
#   depth        -- bits per sample; 10-bit frames arrive as 16-bit words and
#                   _cam narrows them (see gfcam.c)
#   defaults     -- per-camera (exposure, gain) in that sensor's units
#   *_max        -- the control ranges those units are validated against
#
# UNPROVEN (OV8856): no 8 MP machine has been on the bench. Full resolution
# comes from the RAW8 mode the BSP adds to the driver -- the sensor's stock
# RAW10 full-resolution mode asks for 1.44 Gbps/lane and the i.MX6 CSI-2 D-PHY
# stops at 1 Gbps, while 8-bit samples carry the same frame at half that -- and
# the exposure/gain below are the OV5648 defaults translated into the OV8856's
# units (the same fraction of the frame, the same gain multiple), a starting
# point for commissioning rather than measured values.
_SENSORS = {
    'ov5648': {
        'model': 'OV5648', 'width': 2592, 'height': 1944,
        'mbus': 'SBGGR8_1X8', 'depth': 8, 'ctrls': _ctrls_ov5648,
        'defaults': {GFCAM_LID: (24000, 50), GFCAM_HEAD: (24000, 200)},
        'exposure_max': 31600, 'gain_max': 1023,
    },
    'ov8856': {
        'model': 'OV8856', 'width': 3264, 'height': 2448,
        'mbus': 'SBGGR8_1X8', 'depth': 8, 'ctrls': _ctrls_ov8856,
        'defaults': {GFCAM_LID: (1886, 400), GFCAM_HEAD: (1886, 1600)},
        'exposure_max': 2476, 'gain_max': 2047,
    },
}


def _media_ctl(*args: str) -> str:
    """Run media-ctl (default /dev/media0), returning stripped stdout."""
    return subprocess.run(
        ['media-ctl', *args], check=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE).stdout.decode().strip()


def _v4l2_ctl(*args: str) -> None:
    subprocess.run(['v4l2-ctl', *args], check=True,
                   stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def _capture_device() -> str:
    """Resolve the capture entity's /dev/videoN node at runtime."""
    return _media_ctl('-e', _CAPTURE_ENTITY)


def _sensor_entity(bus: int, required: bool = True):
    """Resolve the camera sensor's media entity name on the given I2C bus.

    Returns e.g. 'ov5648 0-0036', found by I2C address so it works whether an
    OV5648 or an OV8856 (HD model) bound to the shared camera node. Returns None
    when the sensor is absent and not required.
    """
    match = re.search(r'entity \d+: (\S+ %d-0036)\b' % bus, _media_ctl('-p'))
    if match:
        return match.group(1)
    if required:
        raise IOError('no camera sensor found on i2c-%d' % bus)
    return None


def _sensor_profile(entity: str) -> dict:
    """The profile of the driver that bound, from its media entity name."""
    driver = entity.split()[0]
    try:
        return _SENSORS[driver]
    except KeyError:
        raise IOError('unsupported camera sensor %r' % driver) from None


def sensor_geometry(cam_sel: int = GFCAM_LID):
    """(model, width, height) of the sensor fitted to the given camera.

    Callers that size buffers or scale coordinates must ask rather than assume
    2592x1944: an 8 MP ("HD") machine captures 3264x2448.
    """
    p = _sensor_profile(_sensor_entity(_CAMERAS[cam_sel]['bus']))
    return p['model'], p['width'], p['height']


def _configure_pipeline(cam_sel: int, sensor: str, other_sensor,
                        profile: dict) -> None:
    """Route the selected sensor through the video-mux to the capture node and
    propagate the raw-Bayer format along every pad of the active path.

    Exactly one sensor link on the video-mux may be enabled -- if both are, the
    CSI cannot resolve a unique upstream sensor (get_mbus_config / link-freq
    lookup) -- so the other camera's link is disabled first.
    """
    cam = _CAMERAS[cam_sel]
    other = _CAMERAS[GFCAM_HEAD if cam_sel == GFCAM_LID else GFCAM_LID]
    # 'field:none' is mandatory: without it link validation rejects STREAMON
    # with -EPIPE.
    mbus_fmt = '%s/%dx%d field:none' % (profile['mbus'], profile['width'],
                                        profile['height'])

    links = []
    if other_sensor:
        links.append('"%s":0 -> "video-mux":%d [0]' % (other_sensor, other['muxpad']))
    links += [
        '"%s":0 -> "video-mux":%d [1]' % (sensor, cam['muxpad']),
        '"video-mux":2 -> "imx6-mipi-csi2":0 [1]',
        '"imx6-mipi-csi2":1 -> "ipu1_csi0_mux":0 [1]',
        '"ipu1_csi0_mux":5 -> "ipu1_csi0":0 [1]',
        '"ipu1_csi0":2 -> "ipu1_csi0 capture":0 [1]',
    ]
    for link in links:
        _media_ctl('-l', link)

    pads = [
        '"%s":0' % sensor,
        '"video-mux":%d' % cam['muxpad'], '"video-mux":2',
        '"imx6-mipi-csi2":0', '"imx6-mipi-csi2":1',
        '"ipu1_csi0_mux":0', '"ipu1_csi0_mux":5',
        '"ipu1_csi0":0', '"ipu1_csi0":2',
    ]
    for pad in pads:
        _media_ctl('-V', '%s [fmt:%s]' % (pad, mbus_fmt))


def _configure_sensor(sensor: str, profile: dict,
                      exposure: int, gain: int) -> None:
    """Apply the manual exposure/gain/white-balance for the bound sensor.
    Values match the factory gfcam defaults on the OV5648 and are translated
    into the OV8856's units on an HD machine. Note: the sensor flips are forced
    off in both -- HFLIP would break CSI capture, so the factory's horizontal
    mirror is applied in software during the grab (see _HFLIP).
    """
    subdev = _media_ctl('-e', sensor)
    if not subdev:
        raise IOError('could not resolve subdev for %s' % sensor)
    profile['ctrls'](subdev, exposure, gain)


def capture(cam_sel: int = GFCAM_LID, exposure: int = None, gain: int = None,
            illumination: int = 132) -> bytes:
    """Capture a JPEG image from the selected Glowforge camera.

    cam_sel      -- GFCAM_LID (0, bed) or GFCAM_HEAD (1)
    exposure     -- sensor exposure in that sensor's units (OV5648: 1/16-line
                    units; OV8856: whole lines); None uses the camera's default.
                    Clamped to the sensor's frame-length ceiling -- a larger
                    value cannot integrate within the frame and returns a blank
                    frame.
    gain         -- sensor gain in that sensor's units (OV5648: 16 = 1x, max
                    1023; OV8856 analog gain: 128 = 1x, max 2047); None uses
                    the camera's default.
    illumination -- scene-lighting LED brightness during the grab (lid_led for
                    the lid camera, head white_led for the head)

    Returns the frame as JPEG bytes, at the fitted sensor's full resolution
    (sensor_geometry() reports it).

    Raises LidOpen if the lid is not closed: neither camera captures with the
    enclosure open, and an unreadable lid counts as open.
    """
    if cam_sel not in _CAMERAS:
        raise ValueError('cam_sel must be GFCAM_LID (0) or GFCAM_HEAD (1)')

    # Privacy gate, checked before the pipeline is configured and before the
    # lamp is touched, so a refused capture leaves the machine as it was.
    if not lid_closed():
        raise LidOpen('lid is open: the cameras only capture with the lid closed')

    cam = _CAMERAS[cam_sel]
    other = _CAMERAS[GFCAM_HEAD if cam_sel == GFCAM_LID else GFCAM_LID]
    sensor = _sensor_entity(cam['bus'])
    other_sensor = _sensor_entity(other['bus'], required=False)
    profile = _sensor_profile(sensor)

    default_exposure, default_gain = profile['defaults'][cam_sel]
    if exposure is None:
        exposure = default_exposure
    if gain is None:
        gain = default_gain
    exposure = max(0, min(exposure, profile['exposure_max']))
    if not 0 <= gain <= profile['gain_max']:
        raise ValueError('gain must be between 0 and %d on the %s'
                         % (profile['gain_max'], profile['model']))

    _configure_pipeline(cam_sel, sensor, other_sensor, profile)
    _configure_sensor(sensor, profile, exposure, gain)

    # Light the scene at the requested level for the grab, then restore the lamp
    # to whatever it was before (don't leave it forced off). If the current level
    # can't be read back, fall back to off.
    lamp = cam['lamp']
    try:
        previous = int(read_file(lamp))
    except (OSError, ValueError):
        previous = None
    write_attr(lamp, illumination)
    try:
        time.sleep(_SETTLE_S)
        return grab(_capture_device(), profile['width'], profile['height'],
                    _HFLIP, profile['depth'])
    finally:
        write_attr(lamp, previous if previous is not None else 0)


__all__ = ['GFCAM_LID', 'GFCAM_HEAD', 'GFCAM_WIDTH', 'GFCAM_HEIGHT',
           'LidOpen', 'capture', 'sensor_geometry']

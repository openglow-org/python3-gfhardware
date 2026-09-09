"""
ffmachine - shared ForgeFIRM hardware-machine glue for the Glowforge
web-service clients: the gfhome one-shot homing runner and the gfcloud
full-cloud daemon both drive the same hardware Machine with captures
routed through forgectrl, honor the same shared-config identity
overrides, and log the same way: through syslog under their own program
name, at the level the shared machine config sets for that logger.
Config-file parsing stays in each client.

Copyright 2026 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
SPDX-License-Identifier: MIT
"""
import logging
import logging.handlers
import os
import sys

from gfutilities.configuration import get_cfg, set_cfg

logger = logging.getLogger('openglow')

# The shared machine config (identity overrides, homing/controller mode,
# log levels), managed from the forgectrl UI. Override the path with
# GFHOME_CONF.
MACHINE_CONF = os.environ.get('GFHOME_CONF', '/data/forgefirm/forgefirm.conf')

# Where the optional debug captures (raw pulse files, sent images) go
# when enabled: never inside the log tree, so an export stays small.
CAPTURE_ROOT = '/data/forgefirm/captures'

# The image version stamp, written by the image build: a release version
# "v<x.y.z>", or the dev image's build timestamp plus a dev tag.
VERSION_FILE = '/etc/forgefirm-version'


def read_machine_conf(machine_conf: str = MACHINE_CONF) -> dict:
    """The shared machine config as a dict ("key = value" lines, '#'
    comments); {} when unreadable."""
    keys = {}
    try:
        with open(machine_conf) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#') or '=' not in line:
                    continue
                k, v = line.split('=', 1)
                keys[k.strip()] = v.strip()
    except OSError:
        pass
    return keys


# Level names of the shared config -> logging levels. Python has no
# notice: notice emits INFO records, and rsyslog's notice filter then
# keeps warnings and above from these loggers.
_LEVELS = {'off': None, 'error': logging.ERROR, 'warning': logging.WARNING,
           'notice': logging.INFO, 'info': logging.INFO,
           'debug': logging.DEBUG}


def emit_level(app: str, machine_conf: str = MACHINE_CONF):
    """The level this process emits at: the more verbose of its disk and
    remote levels (rsyslog filters per destination), FFLOG_LEVEL winning
    over the file. None = nothing is emitted."""
    keys = read_machine_conf(machine_conf)
    names = [keys.get('log_%s_disk' % app, 'info'),
             keys.get('log_%s_remote' % app, 'off')]
    env = os.environ.get('FFLOG_LEVEL')
    if env:
        names = [env]
    levels = [_LEVELS[n.lower()] for n in names if n.lower() in _LEVELS]
    if not levels:
        return logging.INFO
    levels = [l for l in levels if l is not None]
    return min(levels) if levels else None


def setup_logging(app: str) -> None:
    """Route this process's logging to syslog under program name `app`
    (rsyslog files it under /data/log/forgefirm/<app>). The socket is
    non-blocking: a message that cannot be queued is dropped, never
    waited for. Lines are echoed to stderr on a terminal or with
    FFLOG_STDERR=1 (bench runs). Call once, first thing."""
    root = logging.getLogger()
    for h in list(root.handlers):
        root.removeHandler(h)
    level = emit_level(app)
    root.setLevel(logging.CRITICAL + 1 if level is None else level)
    logging.raiseExceptions = False
    # rsyslog stamps the severity; the line carries only the origin.
    fmt = logging.Formatter('%(module)s:%(funcName)s %(message)s')
    try:
        h = logging.handlers.SysLogHandler(
            address=os.environ.get('FFLOG_SOCK') or '/dev/log',
            facility=logging.handlers.SysLogHandler.LOG_DAEMON)
        h.socket.setblocking(False)
        h.ident = '%s[%d]: ' % (app, os.getpid())
        h.setFormatter(fmt)
        root.addHandler(h)
        have_syslog = True
    except OSError:
        have_syslog = False
    echo = os.environ.get('FFLOG_STDERR', '') not in ('', '0') or sys.stderr.isatty()
    if echo or not have_syslog:
        sh = logging.StreamHandler(sys.stderr)
        sh.setFormatter(logging.Formatter(
            app + ': (%(levelname)s) %(module)s:%(funcName)s %(message)s'))
        root.addHandler(sh)


def setup_captures(app: str) -> None:
    """After the app config is parsed: point the optional debug captures
    (LOGGING.SAVE_PULS, LOGGING.SAVE_SENT_IMAGES) at LOGGING.CAPTURE_DIR,
    default CAPTURE_ROOT/<app>, creating it only when a capture is on."""
    d = get_cfg('LOGGING.CAPTURE_DIR') or '%s/%s' % (CAPTURE_ROOT, app)
    set_cfg('LOGGING.DIR', d)
    if get_cfg('LOGGING.SAVE_PULS') or get_cfg('LOGGING.SAVE_SENT_IMAGES'):
        try:
            os.makedirs(d, mode=0o700, exist_ok=True)
        except OSError:
            logger.warning('cannot create the capture directory %s', d)


def forgefirm_version(version_file: str = VERSION_FILE) -> str:
    """The ForgeFIRM version from the image stamp, as a product version:
    a leading "v" before a digit is dropped ("v0.0.1" -> "0.0.1"), the
    dev image's "<timestamp> (dev)" is kept as it is. "unknown" when the
    stamp is unreadable or empty."""
    try:
        with open(version_file) as f:
            stamp = f.readline()
    except OSError:
        stamp = ''
    # A header value: printable ASCII only.
    stamp = ''.join(c for c in stamp if ' ' <= c <= '~').strip()
    if not stamp:
        return 'unknown'
    if stamp[0] in 'vV' and len(stamp) > 1 and stamp[1].isdigit():
        stamp = stamp[1:]
    return stamp


def apply_user_agent(version_file: str = VERSION_FILE) -> None:
    """After the app config is parsed: the User-Agent the Glowforge
    service sees (SERVICE.USER_AGENT) is ForgeFIRM/<version>. A
    user_agent set in the app config wins."""
    if not get_cfg('SERVICE.USER_AGENT'):
        set_cfg('SERVICE.USER_AGENT', 'ForgeFIRM/%s' % forgefirm_version(version_file))
    logger.info('user agent: %s', get_cfg('SERVICE.USER_AGENT'))


def hostname_for(serial) -> str:
    """The factory serial -> hostname derivation. One implementation,
    in gfhardware.id; imported lazily because importing gfhardware pulls
    the hardware modules in."""
    from gfhardware.id import hostname_for as derive
    return derive(serial)


def apply_identity_overrides(machine_conf: str = MACHINE_CONF) -> None:
    """Identity overrides from the shared machine config (set in the
    forgectrl UI): non-empty gf_serial / gf_password beat the OCOTP fuse
    identity - Machine.__init__ sets its fuse values with keep_value, so
    whatever is in the config store first wins. The hostname is never
    overridden independently: it derives from the serial, so a serial
    override re-derives it."""
    keys = read_machine_conf(machine_conf)
    if not keys:
        return
    for key, cfg in (('gf_serial', 'MACHINE.SERIAL'),
                     ('gf_password', 'MACHINE.PASSWORD')):
        if keys.get(key):
            set_cfg(cfg, keys[key])
            logger.info('identity override: %s from %s', cfg, machine_conf)
    if keys.get('gf_serial'):
        try:
            set_cfg('MACHINE.HOSTNAME', hostname_for(keys['gf_serial']))
            logger.info('identity override: MACHINE.HOSTNAME derived '
                        'from gf_serial')
        except ValueError:
            logger.warning('gf_serial is not numeric; hostname left at '
                           'the fuse derivation')


def build_emulator(image_dir: str, work_dir: str):
    """The gfutilities Emulator in this machine's identity: the serial,
    hostname and password from the OCOTP fuses (an override from the
    shared config, applied before this, wins the way it does for the
    hardware machine), the canned frames from image_dir, the downloads
    under work_dir. It touches no hardware: the service sees a machine
    that homes, images and prints at once, and the protocol is what gets
    exercised. The connect-time hunt is kept: the settings report carries
    the machine values (an empty report is the emulator's way of asking
    the service to skip homing), so the service sends its hunt and the
    emulator walks it from the canned home frames."""
    import os
    from gfhardware import id as machine_id
    from gfutilities import Emulator
    for key, value in (('MACHINE.SERIAL', machine_id.serial()),
                       ('MACHINE.HOSTNAME', machine_id.hostname()),
                       ('MACHINE.PASSWORD', machine_id.password())):
        set_cfg(key, value, True)
    os.makedirs(work_dir, exist_ok=True)
    set_cfg('EMULATOR.IMAGE_SRC_DIR', image_dir)
    set_cfg('EMULATOR.MOTION_DL_DIR', work_dir)
    set_cfg('EMULATOR.FIRMWARE_DL_DIR', work_dir)
    set_cfg('EMULATOR.BYPASS_HOMING', False)
    set_cfg('EMULATOR.MATERIAL_THICKNESS', '.230')
    # A controller under forgectrl reports its job state to the cooling
    # engine (~1 Hz; the supervisor waits for the first report before it
    # calls a mode switch complete). The emulator never arms or runs, so
    # it reports idle and unarmed for as long as it lives.
    from gfhardware.coolsvc import cooling_svc
    cooling_svc.set_mode('idle')
    cooling_svc.set_armed(False)
    if not cooling_svc.is_alive():
        cooling_svc.start()
    logger.info("EMULATE: the emulator in this machine's identity, frames from %s, no hardware; "
                "reporting idle to the cooling engine", image_dir)
    return Emulator()


def inhibit_connect_hunt() -> None:
    """No connect-time hunt: the first settings report goes out in the
    reconnect form (no values), which the service answers by keeping the
    head position it already has instead of homing. Only for a start
    where the service already knows where the head is; a print placed on
    a stale position can run the gantry into a rail, so the acceptance
    tests hunt before the one real print regardless."""
    set_cfg('SETTINGS.SET', True)
    logger.info('NO-HUNT: the first settings report in the reconnect form; the service keeps '
                'its head position, no connect-time hunt')


def acknowledge_hunts() -> None:
    """A GRBL homing session's hunts are answered as done without moving
    the lens or playing the hunt file: the session borrows the service for
    its camera homing only, and the lens reference is the runner's own,
    after the session."""
    set_cfg('HOMING.HUNT_ACK_ONLY', True)
    logger.info('HUNT-ACK: hunts are acknowledged without motion; the lens is referenced '
                'after the session')


def park_lens(half_steps: int) -> None:
    """After the runner's reference the lens sits on the hall sensor's
    rising edge. Move it `half_steps` (up positive) to the park height the
    controller asked for, at the run current in half-step mode, and rest
    the motor at the hold current. The count comes from the controller
    (GFHOME_PARK_HALF_STEPS), clamped there to the window every head
    reaches without touching a stop; the lens never meets a stop here."""
    from gfhardware.z_axis import ZAxis
    from gfhardware._common import ZCur, Microstep, Dir
    if not half_steps:
        logger.info('lens park: none, the park height is the edge')
        return
    ZAxis.configure(enabled=True, current=ZCur.HIGH, mode=Microstep.HALF)
    direction = Dir.Pos if half_steps > 0 else Dir.Neg
    for _ in range(abs(half_steps)):
        ZAxis.step(direction)
    ZAxis.configure(current=ZCur.LOW, mode=Microstep.HALF)
    logger.info('lens parked %+d half-steps from the hall edge', half_steps)


def request_header_capture(path: str) -> None:
    """The commissioning wizard's request: the next print's pulse header
    goes to `path` as JSON and that print is canceled before it arms. A
    one-shot: the machine clears the request as it writes the file."""
    set_cfg('CAPTURE.HEADER_PATH', path)
    logger.info('HEADER CAPTURE: the next print reports its header to %s and does not run', path)


def build_machine():
    """Build the hardware Machine with captures routed through forgectrl.

    forgectrl owns the imx-media pipeline whenever it serves a stream
    (LightBurn typically keeps one open), so direct V4L2 grabs fail busy.
    Its snapshot endpoint delivers the same factory-configured
    full-resolution JPEG, works during an active stream (mux borrow), and
    takes a per-shot lamp override - head captures request lamp=0 because
    added white light washes out the measure-laser dot the cloud's focus
    analysis needs. Direct capture remains the fallback when the daemon is
    unreachable.

    The cameras only capture with the lid closed (a privacy rule, enforced
    both in forgectrl and in gfhardware.cam). A refusal is distinguished from
    a daemon failure and propagates instead of falling back: the action runner
    reports it to the service as a failed action, so a request the machine
    will not honor resolves instead of hanging.
    """
    import requests
    from gfhardware import Machine
    from gfhardware.cam import LidOpen
    from gfhardware.leds import head_all_led_off, set_head_led_from_pulse
    from gfutilities.service.websocket import img_upload

    class ForgectrlMachine(Machine):

        @staticmethod
        def _snapshot(cam: str, lamp: int = None) -> bytes:
            url = '%s/cam/snapshot?cam=%s&res=full' % (
                get_cfg('FORGECTRL.URL') or 'http://127.0.0.1', cam)
            if lamp is not None:
                url += '&lamp=%d' % lamp
            rsp = requests.get(url, timeout=45)
            # The privacy gate answers 409 with an explicit reason. That is a
            # refusal, not a daemon failure: raise the same exception the
            # direct path raises so the caller does not retry through a
            # fallback that will refuse identically.
            if rsp.status_code == 409 and b'lid is open' in rsp.content:
                raise LidOpen(rsp.content.decode('utf-8', 'replace').strip())
            rsp.raise_for_status()
            if not rsp.content.startswith(b'\xff\xd8'):
                raise ValueError('forgectrl returned a non-JPEG body')
            return rsp.content

        def _save_sent(self, img: bytes, msg: dict) -> None:
            if get_cfg('LOGGING.SAVE_SENT_IMAGES'):
                with open('%s/%s.jpeg' % (get_cfg('LOGGING.DIR'), msg['id']),
                          'wb') as f:
                    f.write(img)

        def _lid_image(self, msg: dict, settings: dict = None) -> None:
            lamp = self.lamp_level(settings)
            logger.info('capturing Lid Image via forgectrl (lamp %d)', lamp)
            try:
                img = self._snapshot('lid', lamp=lamp)
            except LidOpen as e:
                logger.warning('lid image refused: %s', e)
                raise
            except Exception:
                logger.exception('forgectrl snapshot failed; direct capture')
                return super()._lid_image(msg, settings)
            logger.info('uploading Lid Image')
            img_upload(self._session, img, msg)
            self._save_sent(img, msg)

        def _head_image(self, msg: dict, settings: dict = None) -> None:
            logger.info('capturing Head Image via forgectrl')
            if settings and settings.get('HCil') is not None:
                set_head_led_from_pulse(settings['HCil'])
            try:
                img = self._snapshot('head', lamp=0)
            except LidOpen as e:
                # The measure laser may have just been armed from HCil.
                head_all_led_off()
                logger.warning('head image refused: %s', e)
                raise
            except Exception:
                logger.exception('forgectrl snapshot failed; direct capture')
                return super()._head_image(msg, settings)
            head_all_led_off()
            logger.info('uploading Head Image')
            img_upload(self._session, img, msg)
            self._save_sent(img, msg)

    return ForgectrlMachine()

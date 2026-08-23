# ForgeFIRM Cloud Mode

ForgeFIRM's optional **factory cloud mode** runs the machine under the Glowforge
web service: the machine presents itself as a stock Glowforge, and the
Glowforge phone/web app drives it end to end: connect homing, Set Focus,
material imaging, and full prints (button press, cut, return-home). It is
distinct from the `gfhome.py` one-shot, which borrows the service only for a
camera-referenced homing cycle.

The Glowforge protocol is undocumented and can change without notice. Each
ForgeFIRM release validates cloud mode against one specific factory
service/firmware version, the version it advertises as `MCov` (currently
`2.6.0-2228`). forgectrl surfaces a compatibility warning when the live service
moves past that baseline (see "Firmware-update policy").

## Components

| Piece | Role |
|---|---|
| `gfcloud.py` (`/usr/sbin`) | Full cloud-mode controller daemon. Spawned and supervised by forgectrl when `controller_mode = cloud` (the init script defers to the supervisor and remains a manual stop only); the pulse device arrives as a broker-inherited fd (`GF_PULSE_FD`) that is never closed, so job boundaries and mode switches do not cycle the 40 V rail. SIGTERM stops the service loop, safes the hardware, and exits. |
| `gfhome.py` (`/usr/sbin`) | One-shot service-driven homing. Invoked for `$H` when `homing_mode = gfcloud`; dispatches with `allow_print=False` so a print can never run inside a homing session. Completion is guarded: a run of near-identical service corrections aborts (the machine is not physically moving), and quiet only counts as homed when the head accelerometer witnessed real motion during the session. |
| `ffmachine.py` (site-packages) | Shared hardware-machine glue: identity overrides from the shared config, and the forgectrl-routed capture machine both clients use. |
| `gfutilities` | Protocol/service layer: auth, WebSocket client, action dispatch, settings report, pulse-file handling. |
| `gfhardware` | The hardware `Machine`: motion, laser latch, switches, cameras. Thermal hardware belongs to the forgectrl cooling engine: the cloud client reports job state (`POST /cool/state`, with the pulse header's run fan duties as the per-job profile) and enforces the published verdict on its fire path, gaining the flow verification and over-temp protection the engine provides. |

Camera captures route through forgectrl's snapshot endpoint (it owns the
imx-media pipeline whenever a stream is open; the snapshot works during an
active stream and takes a per-shot lamp override), with direct V4L2 capture as
the fallback when the daemon is unreachable.

**The cameras only capture with the lid closed.** This is a privacy rule, not a
factory behavior: the lid camera faces the room once the lid is raised, and the
service asks for images on its own schedule. Both capture paths enforce it
(forgectrl answers `409` and the direct fallback raises `gfhardware.cam.LidOpen`)
and the check fails closed, so an unreadable lid also refuses. There is no
setting to disable it. A refused image action is reported to the service as
`<action>:failed` so it resolves rather than hanging, and the client does not
fall back to a direct grab that would refuse identically. **Consequence:** the
factory ran focus hunts with the lid open and a hunt includes a head capture,
so a hunt attempted with the lid open now fails; the lid must be shut before
the app focuses or prints.

## Scope: telemetry is excluded

The factory streams continuous telemetry; ForgeFIRM does not and will not.
Excluded channels:

- `POST /api/sensor`, the binary sensor firehose.
- WSS `type:"log"` messages, the in-band advisory logs.
- The `fault:*` / `estop:*` / `interlock:*` **cloud reporting** namespace.
  (Local fault-to-safe handling is independent of reporting and fully active.)

On-demand requests are still answered: the `settings` report, image captures,
and the functional action handshake.

**One exception, deliberately kept: the progress frame.** A capture of the
factory's own cloud session showed that the app's progress bar rides a WSS
`type:"progress"` frame the machine sends every 30 s during a job (below,
"Progress reporting"). That is a UI status update, not the sensor telemetry
this section excludes, so ForgeFIRM carries it. The bar is the operator's only
sign a multi-hour print is advancing; going dark for hours is not a scope we
want. Everything else above stays excluded, including inside that frame: of
the fifteen periodic tags the factory packs into it, ForgeFIRM fills the five
that describe the job (`CAid`, `CCbp`, `CCst`, `CCxp`, `CCyp`) and leaves the
temperatures and the IR readings out. The service already has this machine's
settings report; it does not get a sensor feed by the side door.

## Connection and authentication

- `sign_in` returns two JWTs: `auth_token` (Bearer, ~6 h) and `ws_token` (a
  path component of the WS URL, ~30 s expiry, single-use).
- The WS client reconnects through a loop that re-runs `sign_in` for a fresh
  `ws_token` and rebuilds the URL on every reconnect.
- The service drops the socket on its own schedule, roughly hourly in a long
  idle session. That is routine: the reconnect loop re-authenticates with a
  fresh `auth_token` and the machine stays signed in and serving actions
  without operator involvement.
- Any HTTP request that gets a 401 re-signs-in and replays once (the sign-in
  request itself never retries, so there is no recursion).
- `ws_connect()` returns the running client; `GFUIService` stops it (flush
  final events, close socket, join) when the session ends. The WS client and
  action threads are daemon threads: nothing about a session can keep the
  process alive after the service loop exits.
- TLS: standard certificate validation. The factory additionally pins the
  server SPKI and refuses unpinned connections; ForgeFIRM does not pin, by
  decision. A pin is a copy of a key the service can rotate whenever it
  likes, and chain validation already gives interoperability everything it
  needs.

## Wire format

A WS text frame may pack **several newline-delimited JSON objects**; each is
parsed separately, and unparseable objects are logged and skipped.

**Incoming action envelope** (server→machine): `id` (int64, becomes
`action_id`), `action_type`, `machine_serial` (ignored), `status`,
`motion_url` (hunt/motion/print), `settings` (sparse per-action dict), and on
image actions `endpoint` (presigned upload URL).

Only `status:"ready"` starts an action; the other statuses in the protocol
(`new`/`started`/`success`/`failure`) never launch work and are ignored.
**Cancellation is the same action `id` re-sent with `status:"cancelled"`.**

**Outgoing event envelope** (machine→server): `id` (monotonic per-connection
counter), `timestamp` (ms since daemon start, not wall clock), `type:"event"`,
`version:1`, `level`, `action_id` (absent on unsolicited events), `event`.
Parser note: an event's `<action>` prefix segment can be empty; never assume
`split(':')[0]` is non-empty.

## Actions

The 2.6.0 vocabulary is twelve `action_type` values, and they are handled
through a single dispatch table shared by `gfcloud` and `gfhome`:

| action_type | Behavior |
|---|---|
| `settings` | Sends the on-demand ~600-key settings report. Machine IP, firmware/app version, and serial reach the service through this report (`MCip`, `MCov`, `MCdv`, `MCsn`); there is no separate status endpoint. |
| `hunt` | Focus-lens homing (Z home + the service's hunt pattern + home offset). |
| `motion` | Downloads and runs the pulse file at `motion_url`. |
| `print` | Full print lifecycle (gfcloud only; gfhome refuses prints). |
| `lid_image` | Lid-camera capture + upload. |
| `head_image` | Head-camera capture + upload. |
| `lidar_image` | Head captures with the distance-measuring laser (per-shot settings arrive as a list). |
| `user_image` | User-requested snapshot: a lid-camera capture, lid closed. Same as `lid_image` in the factory too, name apart. |
| `factory_reset` | Refused with `factory_reset:failed` and **never acted on** - a cloud command must not wipe a ForgeFIRM machine. |
| `update_check` | Answered with `update_check:completed`; see the firmware policy below. |
| `head_firmware_update` | Refused with `head_firmware_update:failed` - a cloud command does not flash the laser head. |
| `focus` | Ignored, as the factory application ignores it: its own dispatch has no case for the name. |
| unknown | Ignored. |

Only a `ready` status is a request to do anything. The three answered on the
protocol thread (`settings`, `update_check`, `factory_reset`,
`head_firmware_update`) act on `ready` alone: the service also sends a
`cancelled` for actions a machine never received, and the factory ignores
those rather than answering them. The job actions are deliberately not gated
that way, because a `cancelled` is how a print in flight is stopped.

### What the three unprompted actions do in the factory

None of these has ever been seen on the wire here, so they were read out of
the 2.6.0 binary rather than observed. All three turn out to be hand-offs to
programs ForgeFIRM does not have:

- **`update_check` checks nothing.** The handler writes `'u'` to the runit
  control fifo `/var/run/svs/glowforge-updater/control` and reports
  `:completed`, or `:failed` if that write fails; on a service-sent failure or
  cancel it writes `'d'` to stop the service again. Everything an update means
  happens in that separate daemon. The application carries no update endpoint
  at all, and a cut refuses to start while the updater holds its lock.
- **`factory_reset` replaces the application with a script.** The action posts
  a command to the hardware task, which tells runit not to restart the
  application and then `execl`s `/usr/bin/factory_reset.sh`, passing `reboot`
  when the request's flag asks for one.
- **`head_firmware_update` flashes the laser head.** It takes
  `head_firmware_filename` from the request, reads it out of
  `/glowforge/fw/head/` and runs `/usr/bin/head-update.sh`.

The two refusals report `:failed` rather than `:cancelled` on purpose. In this
protocol a cancel is what the service says when it withdraws an action;
failure is what a machine says when the thing did not happen, and it is the
factory's own report when its reset script cannot be launched.

### Per-action settings

Actions carry a sparse `settings` dict (for lidar, a list of dicts), normalized
and made available to every image handler. Policy per key:

- **Honored:** `HCil` (head illumination) and `LCfl` (lid flash) are lighting
  the capture path can apply directly.
- **Deliberately not applied:** `HCex`/`HCga`/`HCae`/`HCag`: the factory-scale
  exposure/gain values use different units than the mainline camera controls
  and would mis-expose; per-camera defaults are used instead.
- The opening `settings` action can carry service-pushed values (e.g. the
  `NRic` network-retry family); ForgeFIRM reports its own values and the
  service tolerates that.

The service pushes very little, and it is worth knowing exactly how little
before going looking for a setting that is not in a pulse header. Across every
captured session, counting every action type and not just the opening one, the
service has ever pushed seven keys: `IMct`, `NRic`, `HCil`, `HCae`, `HCex`,
`HCag` and `HCga`. The opening `settings` action on its own carries one,
`NRic`. Nothing thermal, nothing about fans, nothing that bounds the machine
arrives this way. The operating envelope reaches the machine only in the pulse
header, per job.

The other half of that: for the header fields the service does not override
with its own policy, what comes back is what this machine last reported. The
`MACHINE_SETTINGS` defaults are largely placeholders, so those fields
round-trip as placeholders. A field arriving as zero usually means ForgeFIRM
sent zero, not that the service has nothing to say. Nothing downstream consumes
them today, but it does mean the service is not a source of truth for any limit
the machine itself declares.

Head images are captured with the white torch off, because added white light washes
out the measure-laser dot the cloud's focus analysis needs.

### Events emitted

The service drives the entire lifecycle on this reduced event set (the large
factory event/progress state machine is advisory):

- Per action: `<action>:starting`, `<action>:completed`, `<action>:cancelled`.
- Print lifecycle: `print:download:completed`, `print:running`,
  `print:paused` / `print:resumed`, `print:cancelled`,
  `print:return_to_home:succeeded`, `print:completed`.
- Button: `button:pressed` / `button:released` (the app's "push the button"
  screen needs nothing else).
- Unsolicited `lid:opened` / `lid:closed`, which drive the app's header state
  and trigger an immediate service `lid_image` refresh.
- Alongside the events, a running print sends the `type:"progress"` frame
  described below. It is a different message type, not an event.

Service behavior worth knowing:

- After any mid-job abort the service re-hunts; after a completed print it
  issues a `lid_image` and a Z re-hunt.
- **The service dead-reckons machine position.** The return-to-home park runs
  after every print, finished or aborted, and ignores the lid and the cancel
  flag while it runs (the factory parks with the lid open, and a park cut short
  would offset every subsequent motion until the next camera re-home);
  `print:return_to_home:succeeded` is sent only when the park actually
  completed. A job refused before it moved (lid or interlock open at start,
  a backstop, since the app itself will not print until the lid is closed and imaged)
  ends `:cancelled`, never `:completed`.
- Server-side session state can be sticky: after abnormal session deaths the
  service may stall silently mid-sequence in the next session. A fresh WS
  session recovers it.

### Progress reporting

The app's progress bar rides one carrier, settled by a capture of the
factory's own session:

- **The carrier is an outbound WSS `type:"progress"` frame**, machine to
  service, and nothing else. No `<action>:progress` event, and no
  `progress_bytes` query on the action endpoint, appeared in a full session.
- **The progress frame is the periodic settings report.** Its `settings.values`
  block is exactly `periodic_settings_tags` (`BTvl CAid CCbp CCst CCxp CCyp
  CMet FTvl HTvl IRva IRvb IRvc IRvd ITvl LTvl`): board / fused / head /
  interconnect / lid temperatures, four IR values, a camera id, a coolant
  value, the byte position `CCbp`, the state `CCst`, and X/Y position
  `CCxp`/`CCyp`. Progress and the periodic telemetry are one message, which is
  why carrying it is cheap: the client already builds the ~600-key settings
  report on demand.

  ```json
  {"id":359,"type":"progress","version":1,"action_id":1577564802,
   "progress":"print:progress","current":994,"units":"steps","total":33291208,
   "settings":{"values":{"CCbp":1009,"CCst":1,"CCxp":0,"CCyp":0, ...}}}
  ```

- **Cadence is 30 s** (`progress_update_interval_ms` = 30000), plus a burst at
  every phase transition. During a cut `current` advances at the step
  frequency; at the phase boundaries the frame is `<action>:download`,
  `<action>:upload`, etc., with `current` in bytes.
- **`total` is bytes enqueued, not the job.** In the captured print it grew
  33,291,208 → 33,553,352 → 33,815,496 in steps of 262,144 (256 KiB per
  interval): the factory live-appending to its ring, on the wire. A progress
  report of `current/total` therefore divides by a denominator that is itself
  growing.
- `CCbp` reads the byte position (1009 against `current` 994 in the frame
  above). It is telemetry, not a job parameter: the factory's own tag table
  marks `CCbp` and `CCbt` report-only, so neither can appear in a pulse
  header at all, and the pause constants stay in `forgefirm.conf`.

**What ForgeFIRM sends.** The same frame, at the same 30 s cadence, forced at
every phase change: the run's start, each pause and resume, each hold for a
stalled feed, and once more where the job ends. A print is the only action
that reports, which is what the factory does too, and its park reports under
the print as its last leg. `current` is the byte position the kernel has
played, so it steps back after a pause backtracks, exactly as the factory's
does. A pause is reported as the two events `print:paused` and
`print:resumed`; the factory's ten-event pause phase machine
(`print:pausing_decel`, `print:pausing_backtrack`, `print:paused`,
`print:resuming`, each with `:starting`/`:succeeded`) is advisory and is
not mirrored, and neither are its transfer-phase frames
(`<action>:download`, `<action>:upload`), since the `:starting` event
already says the same thing.

The denominator is the one place ForgeFIRM deliberately does better. The job's
length is known before a byte plays: a plain body carries it in its size and a
compressed one in the gzip ISIZE trailer, so the whole job never has to be
inflated to learn it. That figure is frozen when the run starts and every
frame divides by it, which is why the bar means what it says under a live feed
where the kernel's own byte total is still climbing. A job that plays past its
declared length reports complete rather than overshooting, and says so in the
log once. The length is named in the log at the start of every job, and a job
that does not end where it said it would is named there too.

## Image upload

Image actions carry a presigned `endpoint` URL; the image is `PUT` there as a
plain request (the presigned URL carries its own auth; a Bearer header makes
the storage backend reject it). Without `endpoint`, the legacy
`POST /api/machines/<action_type>/<id>` fallback is used.

## Jobs (motion / print)

- The pulse file at `motion_url` is downloaded and written into the kernel
  pulse-device ring, then run. The deadman flock is held on one fd for the
  whole job; process death fires the kernel dead man's switch.
- The job is supervised the way the factory firmware supervises it. The
  hardware chain kills the beam on the lid and the interlock loop by itself;
  the client decides what motion and the job do, reacting on the switch edge
  (the switch thread wakes the run loop; the level read every 100 ms is the
  backstop):
  - lid or interlock loop opens during a print/motion, or the service cancels
    it, or the cooling verdict pulls fire: controlled stop (`cnc/stop`,
    position kept), job cancelled; a print then parks (with the lid open, if
    it is) and reports `:cancelled`. The park first drops what the job left
    in the ring (the rest of an aborted print, or the whole print after a
    cancel at the button wait) - the factory's "clearing pulse data" - so
    nothing plays ahead of it; a job that never moved parks nothing;
  - lid or interlock open during the pre-print button wait: latch relocked,
    job cancelled; a press with the lid open never arms;
  - a hunt ignores the lid (lens travel plus the service's XY hunt pattern);
  - **the button pauses and resumes a print**: press → controlled stop, then
    `cloud_pause_backtrack_ticks` (default 2000) ticks backward with the laser
    off, `print:paused`; press again → forward with the laser re-enabled after
    `cloud_resume_lead_ticks` (default 1950), `print:resumed`. What the two
    counts really say is that the beam comes back on 50 ticks before the point
    the pause stopped at, over ground the job already cut, and that is the
    invariant the client keeps: the retrace is sized to `cnc/max_backtrack`,
    the history the ring still holds, and the lead follows it down. A pause in
    a print's first moments therefore retraces a little and leads a little,
    rather than failing; a live-fed print pauses exactly like a preloaded one,
    because the ring's retained gap is history whether the job was preloaded
    or is being fed. The laser latch stays unlocked and the armed window open
    through the pause (HV_ENABLE drops by itself when the stream stops, and the
    resume lead covers its re-arm); lid, interlock or a service cancel while
    paused cancel the job from where it stands. Motions and hunts do not pause.
    Both tick counts are `forgefirm.conf` keys.
  - **a live feed that wedges holds the job the same way.** A feeder with room
    in the ring in front of it and no progress behind it is not feeding, and
    left alone it ends one way: the ring plays out what it holds, tens of
    minutes at the print tick, and then goes dry, which is an underrun, a
    position no longer trusted and a job that cannot be picked back up. Thirty
    seconds of that is enough to stop the machine cleanly and retrace, which
    is done while there is still history to retrace over. If the feed moves
    again within a minute the job resumes over ground it already cut, seam
    hidden, `print:paused` and `print:resumed` reporting it exactly as the
    button pause does; if it does not, the job is cancelled rather than left
    stopped in the material. A feed that stalls repeatedly (three holds) is
    cancelled rather than cut in pieces, and a full ring is never mistaken for
    a stall. A press during a hold is not lost: pausing a stopped job is not a
    thing the machine can do, so the press is read once the job is moving
    again.
- Post-action cleanup always locks the laser latch and drops the pulse-device
  registration, including when an action crashes.
- A job larger than the ring runs anyway: the client holds the compressed body
  in memory, fills the ring before the button is asked for, and tops it up as
  it drains, so the ring is a window onto the job rather than the place the job
  lives. The ring size caps how much of a job is buffered at once (~56 min at
  the print tick), not how long a job may be.
- What does have a ceiling is the memory that body sits in. The client refuses
  a job whose declared length is past `pulse_reject_threshold_bytes`, before it
  takes a byte, and abandons a download that runs past it whatever was
  declared, because a service that declares nothing (or declares wrongly) must
  not be handed all the memory there is. A refusal is logged as `refusing the
  job:` with the size and the limit, so it reads differently from a transport
  failure, and the print is reported `:cancelled` like any job that never
  moved. `pulse_warn_threshold_bytes` only logs. Both are memory guards and
  neither is a ring guard: they say nothing about how long a job may be.
  The defaults (32 MiB warn, 128 MiB refuse) are reasoned rather than
  measured, because at the compression the service actually uses 128 MiB
  of body is days of cutting and nothing has come near it; every job logs
  the body it arrived as and the program it played, so the ratio the
  guards are sized against is on record should a job ever get close.

### The pulse header

Every pulse file opens with a header: a length, then a flat list of 4-character
tags each carrying a 32-bit little-endian value. It is the job's operating
envelope, and the factory firmware treats a well-formed one as a precondition
for cutting at all. Of the tags it knows, 346 are accepted in a header and 29
are mandatory; a header missing any one of the 29 is refused outright, and a
known tag that is not header-legal is refused too. An unrecognized tag is only
logged and skipped, which is why a newer service can talk to an older machine.

The header is not a set of echoes. Roughly two thirds of the fields come back
holding whatever the machine last reported, but the service substitutes real
operating values for the ones that matter: fan duties, per-sensor temperature
ceilings, lid IR flame thresholds, head accelerometer limits and a high-voltage
current cap all arrive filled in per job.

Every tag the service fills in has a disposition here, and the job log says
which: each job names its lifecycle keys, counts the keys with no applier,
and splits that count into *declared ignored* (a decision with a reason,
listed below) and *undecided* (`N of M header keys have no applier here (D
declared ignored, U undecided)`, the undecided ones named at debug level).
The undecided count is the one that should be zero.

| Tags | Disposition |
|---|---|
| `AArd`, `EFrd`, `IFrd` | **Applied**: the run-phase fan duties, handed to the forgectrl cooling engine as the per-job profile on the scale the service uses (air assist 0 to 1023, exhaust and intake 0 to 65535). While the laser is armed the engine raises any of them to its configured run duty (the airflow floors were measured there), so a print's fans never run slower than the cut profile, while a hunt's `0` duties stand and the hunt is measured, not judged. |
| `STfr` | **Applied**: step frequency. |
| `XSrc`, `YSrc` / `XShc`, `YShc` | **Applied**: stepper current while running / idle. |
| `XSdm`, `YSdm` / `XSmm`, `YSmm` | **Applied**: decay mode / microstep mode. |
| `ZSmd` | **Applied**: Z mode. |
| `CMrx`, `CMrn` | **Passed through** as the job's coolant window (millidegrees, sent as degrees) on every `POST /cool/state` while the job is loaded. The engine applies each only where it is stricter than its configured value, never looser, never to a gate the operator turned off; the coolant ceiling is the consumer. |
| `EFrx`, `IFrx`, `AArx` | **Passed through** as the tach floors (the maximum periods, sent as the minimum speed each means in the kernel's units); the airflow gates are the consumers, and a header can only raise a floor for its job. A sentinel (0, 1023, the signed extremes, the unsigned rail) or an absurd value is dropped. |
| `AArn`, `EFrn`, `IFrn` | **Read, inert**: the tach minimum periods are maximum speeds, which nothing gates on. |
| `MCsn`, `PDfm` | **Refused on**: the serial the job is locked to and the pulse-data format, both checked before a byte reaches the ring; a mismatch is `refusing the job:` in the log and `:cancelled` to the service. |
| `CFrh`, `CCwp`, `CCrp`, `CCup` | **Logged**: the lifecycle keys, named in every job's log and driving nothing, which is what the factory does with them (below, "A print's warm-up and its rest"). |
| `AAw?`, `EFw?`, `IFw?` | **Declared ignored**: the warm-up phase fan profile. The run profile covers the warm-up hold, and every captured header sets the warm-up values equal to the run values. |
| `PTmn`, `PTmx` | **Declared ignored**: the supply temperature window. The service sends the whole ADC range (a window that cannot trip) and the factory binds the pair to nothing; the supply's raw reading is watched per job by the engine instead. |
| `BT??`, `HT??`, `LT??`, `IT??`, `FT??` | **Declared ignored**: the board, head, lid, interconnect and fused temperature ceilings, sent in a unit that is not millidegrees and not established. The chassis (board) sensor is watched per job; the other four locations have no sensor on this platform. |
| `CTrn`, `CTrx` | **Declared ignored**: the coolant window in raw counts, older files only; `CMrn`/`CMrx` carry the same window. |
| `HA??` | **Declared ignored**: the head accelerometer thresholds (unit and filter unknown). The accelerometer is the motion-liveness probe here; a bench-measured crash detector is its own BRINGUP item. |
| `IR??` | **Declared ignored**: the lid IR flame thresholds. The lid IR channels read the lid lamp, so these absolute numbers are the prior for the lamp-aware fire watch (BRINGUP "Fire watch"), not a gate. |
| `HIix`, `HIrx` | **Declared ignored**: the HV current caps. The sampled `LASER_ON` witness covers the idle case, and HV current is ranged in every job's log line. |
| `TRuc` | **Declared ignored**: thermal report upload conditions, a knob for the factory's telemetry, which is out of scope. |
| `WPon` | **Declared ignored**: the pump is held on as part of the engine's idle posture; per-job pump control would belong in its per-job profile. |
| everything else | **Undecided**: counted and named at debug level by every job. Of the 346 header-legal tags, the factory binds 283 to a source; 20 configure the client's own network backoff, 39 belong to the three fans of an air filter, the camera families are exposure and gain values the mainline driver's units do not take, and most of the rest are per-phase idle variants of the limits above. A hunt header from the live service leaves 49 undecided today. |

Thermal policy is the cooling engine's, on purpose: it runs its own coolant
ceiling and critical line, flow verification, airflow gates, emission
witness and silence timeout, and a remote service can tighten those limits
for a job and never loosen them. The factory's own policy, decoded from its
firmware, is in BRINGUP's facts bank ("The factory's envelope").

## A print's warm-up and its rest

The factory holds twice around a print, and so does ForgeFIRM. Measured on
a factory slot: **3.05 s** between configuring the run and starting it, and
about **10.35 s** of rest after the park before the machine goes idle. A
motion or a hunt gets neither.

Both are equipment protection rather than ceremony. The warm-up is what gets
air and coolant moving before the first fire; the rest is what purges the
enclosure and the tube after the last one. The service assumes both have
happened, so a machine that skips them is running hardware nobody looked
after.

`MOTION.WARM_UP_DELAY` and `MOTION.COOL_DOWN_DELAY` carry the seconds and
default to the factory's measurements. 0 skips either, deliberately, and a
skipped period says so in the log rather than passing in silence: a config
that carries explicit zeros keeps them until someone changes them.

The pulse header looks like the source of these periods and is not. Every
captured print header carries `CCwp` 5000 and `CCrp` 10000, with `CFrh` for
the park, and a motion or a hunt carries none of them; the correlation is
real and the causation is not. In the 2.6.0 application all four lifecycle
keys (`CCrp`, `CCup`, `CCwp`, `CFrh`) are parsed, stored, copied into the
settings batch and acted on by nothing: a tag reaches behavior either
through a peripheral that registers it against a source or through an
inlined lookup by index, and these four have neither. The factory's warm-up
and rest come from somewhere other than the job, so configured periods
defaulted to what the factory was measured doing are the right model rather
than a placeholder. The keys stay in the per-job log line as a record of
what the service sends.

## Firmware-update policy

ForgeFIRM **never downloads or installs factory firmware.**

- The `update_check` action is answered with `update_check:completed`, which
  is what a factory machine sends once it has started its updater, without
  the hand-off that would install a factory image over ForgeFIRM. A version
  probe that fails answers `update_check:failed`, the one honest failure
  here: the check itself did not happen.
- On connect (when `FACTORY_FIRMWARE.CHECK` is set) a read-only
  `GET /update/current` probe records
  `{latest_gf_version, tested_against_gf, checked_at}` to
  `FACTORY_FIRMWARE.STATUS_FILE` (`/data/forgefirm/gf-latest.json`).
- forgectrl reads that file and shows a **compatibility warning** whenever the
  live service has moved past the tested baseline, regardless of whether a
  newer ForgeFIRM exists, plus an **upgrade recommendation** only when one
  does. The factory `.fw` is never offered.
- `tested_against_gf` is `FACTORY_FIRMWARE.FW_VERSION` from the client's
  configuration: the factory firmware version this build advertises as
  `MCov` and was tested against. There is no separate release-side field:
  the value travels config → `gf-latest.json` → forgectrl's status (`gfsvc`)
  → the panel banner.
- One caveat on that probe: `/update/current` is inherited from the 1.x-era
  client and appears nowhere in the 2.6.0 application, which reaches its
  updater through the service action instead. The endpoint answering is an
  assumption, not something the current factory firmware demonstrates.

## Configuration

| Where | Keys |
|---|---|
| `/data/etc/gfhome.conf` (seeded from `/etc/gfhome.conf.sample`) | `SERVICE.*` (server/status URLs), `FACTORY_FIRMWARE.CHECK` / `STATUS_FILE`, `FORGECTRL.URL`, `LOGGING.SAVE_PULS` / `SAVE_SENT_IMAGES` (both default off) and `LOGGING.CAPTURE_DIR` (default `/data/forgefirm/captures/<app>`), `MOTION.*`, `THERMAL.*`. |
| `/data/forgefirm.conf` (managed from the forgectrl UI) | `controller_mode` (`grbl` / `cloud`, read by the forgectrl supervisor, which spawns exactly one controller at boot and on every mode switch; the init scripts defer to it), `homing_mode`, identity overrides `gf_serial` / `gf_password` (a serial override re-derives the hostname), the pause pair `cloud_pause_backtrack_ticks` / `cloud_resume_lead_ticks`, the download guards `pulse_warn_threshold_bytes` / `pulse_reject_threshold_bytes` (bytes of compressed body held in memory, unset = 32 MiB warn and 128 MiB refuse, 0 lifts either), and the log levels `log_gfcloud_disk` / `log_gfcloud_remote` and `log_gfhome_*` (each `off`..`debug`; read at process start, so applied at reboot). |

## The offline service

`gfcloud.py --offline`, or a start while the marker file
`/run/gfcloud-offline` exists, runs the machine under gfutilities'
`OfflineService` instead of the web service: no sign-in, no WebSocket,
no network. A UNIX socket, `/run/gfcloud-offline.sock`, takes the
service's action messages exactly as the service would send them (one
JSON object per line: `id`, `action_type`, `status`, `motion_url`,
`settings`) and hands back every event the machine would have sent the
service, one per line, to every connected client; each event is also
logged (`offline event: ...`). The session the machine is given serves
`file://` URLs, so a `print` whose `motion_url` names a local pulse file
loads it through the same header check, pulse source and run loop a
service job takes, and sinks any upload (an image capture completes
without reaching anywhere). Everything downstream of dispatch is the
object graph the service drives; the service itself is what is absent.

This is the lever the acceptance tests use to exercise the machine's
print behavior (the lid and interlock aborts, the button-wait cancel, a
paused print ended by the lid, a print longer than the ring) without an
account, a network, or a job designed in the app; the service protocol
is proven separately, by a real print and by the emulator. The marker
is under `/run`, so a reboot never comes up offline by accident, and the
log carries `OFFLINE service` while the client runs this way.

## The emulator in this machine's identity

`gfcloud.py --emulate`, or a start while the marker file
`/run/gfcloud-emulate` exists, runs the real service session with
gfutilities' `Emulator` in place of the hardware machine: the serial,
hostname and password come from the fuses (or the shared config's
overrides) exactly as for the real client, the session signs in and
opens the WebSocket exactly as the real client does, and every action
the service sends is answered the way the emulator answers it: canned
frames for the captures (the dev image's gfutilities fixtures under
`/usr/share/gfutilities/emulator/`, captured on a machine of this type
with the head at its calibrated home), the pulse file downloaded and
parsed through the same path a job takes, and a print reported running
and complete without a button wait and without moving anything. Nothing
is armed and nothing moves; the pulse device is never opened.

This is the lever the acceptance tests use to prove the service protocol
(sign-in, the firmware check, the WebSocket, the hunt, the image uploads,
a print's download and lifecycle as the app sees them) with nobody at
the machine: only the app has to be driven. What it cannot prove is the
machine's side of a print (progress, the button wait, the pause, the
aborts); those are the real print and the offline tests. The real client
must be down before the emulator signs in (the service holds one session
per machine), so the tests stop the controller, set the marker, start it,
and restart it without the marker when done. Same marker rule as the
offline service: under `/run`, one start, never a persisted setting.

## Starting without the connect-time hunt

`gfcloud.py --no-hunt`, or a start while the marker file
`/run/gfcloud-nohunt` exists, sends the first settings report in the
reconnect form (no values). The service answers that form by keeping the
head position it already has instead of sending its connect-time hunt,
which is what the factory client does on every reconnect within a
session (gfutilities does the same: only a process's first report
carries the values). The log carries `NO-HUNT:` at the start. It is for
a restart where the service already knows where the head is, which is
what the acceptance tests do between tests that do not home; a fresh
boot, the homing tests, and the one real print report the values and
get the hunt, because a print placed on a stale head position can run
the gantry into a rail. The offline service has no hunt to skip and the
emulator keeps its hunt, so the marker is ignored under either.

The three markers share one rule: under `/run`, so a reboot never comes
up with one; read by the client that starts, and taken down by it, so a
marker applies to that one start and never to a respawn after it.

## Outstanding items

Everything the bench can exercise is exercised: the `cloud.*` acceptance
tests cover mode switching, service homing, the lid and interlock aborts,
the button-wait cancel, a hunt with the lid open, pause and resume, a
paused print cancelled by the lid, and a print longer than the ring with
the cancel from the app. What is left is below.

- **8 MP ("HD") machines:** an OV8856 machine captures 3264x2448, not the
  2592x1944 a 5 MP machine sends. Whether the service accepts a larger image
  for bed alignment and focus analysis is unknown; no 8 MP machine has been
  on the bench.
- **Coolant control per job:** the forgectrl cooling engine holds the pump
  on as part of its idle posture; the `WPon` pulse-header key has no
  applier. If per-job pump control is ever wanted, it belongs in the
  engine's per-job profile (the `/cool/state` report), not here.

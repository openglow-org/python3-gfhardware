Python modules for accessing and controlling Glowforge brand CNC laser hardware.

`forgefirm-app/` contains the ForgeFIRM Glowforge web-service applications
built on these modules: `gfhome.py` (one-shot service-driven homing),
`gfcloud.py` (full cloud-mode controller daemon, with init script), and
`ffmachine.py` (the shared hardware-machine glue both use). They are not part
of the `gfhardware` Python package; the ForgeFIRM image recipes install them
directly from this directory. Cloud mode is documented on the documentation
site: https://docs.forgefirm.org/technical/forgefirm/cloud-mode/

## Test

```sh
python3 -m pytest tests/
```

## Contributing

[AGENTS.md](AGENTS.md) carries the rules for this repository and for the
project: safety ordering, proof before done, the push order, and the writing
rules. They apply to human contributors too.

## License

`gfhardware` and the ForgeFIRM applications are MIT licensed (see `LICENSE`),
with one third-party component under a different license:

- `gfhardware/src/bayer.c` and `bayer.h` are the Bayer demosaicing routines
  from libdc1394 (Damien Douxchamps, Frederic Devernay; VNG and AHD from Dave
  Coffin's DCRAW), licensed **LGPL-2.1-or-later** (`LICENSE.LGPL-2.1`). They
  are compiled into the `gfhardware._cam` extension module together with the
  MIT sources. Because this repository carries the complete source of both
  parts and the standard build (`setup.py`), anyone can modify the LGPL
  component and rebuild or relink `_cam` against it, which is how the LGPL's
  relinking requirement is met.

Binary packages built from this repository (including the ForgeFIRM image
recipes) declare `MIT & LGPL-2.1-or-later` accordingly.

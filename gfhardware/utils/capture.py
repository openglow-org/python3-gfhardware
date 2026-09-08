"""
Copyright 2020-2026 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
https://community.openglow.org
SPDX-License-Identifier:    MIT
"""

if __name__ == '__main__':
    import argparse
    import sys
    from gfhardware.cam import capture, GFCAM_LID, GFCAM_HEAD, LidOpen
    parser = argparse.ArgumentParser(
        description='Capture jpeg image from Glowforge camera. '
                    'The cameras only capture with the lid closed.')
    parser.add_argument('--head', action='store_true',
                        help='Capture from head camera [default: lid camera]')
    parser.add_argument('filename', action='store',
                        default="capture.jpeg", type=str,
                        nargs='?',
                        help='Specify output filename [default: capture.jpeg]')
    parser.add_argument('exposure', action='store',
                        default=None, type=int,
                        nargs='?',
                        help='Sensor exposure in the fitted sensor\'s units, clamped to its '
                             'frame length [default: per-camera]')
    parser.add_argument('gain', action='store',
                        default=None, type=int,
                        nargs='?',
                        help='Sensor gain in the fitted sensor\'s units (OV5648 16 = 1x, '
                             'OV8856 128 = 1x) [default: per-camera]')
    parser.add_argument('illumination', action='store',
                        default=132, type=int,
                        nargs='?',
                        help='Scene LED brightness during capture [default: 132]')
    args = parser.parse_args()

    camera = GFCAM_LID
    if args.head:
        camera = GFCAM_HEAD

    # Capture before opening the output: a refused or failed capture must not
    # truncate a file that already holds a good image.
    try:
        img = capture(camera, args.exposure, args.gain, args.illumination)
    except LidOpen as e:
        sys.exit('%s' % e)

    with open(args.filename, 'wb') as f:
        f.write(img)

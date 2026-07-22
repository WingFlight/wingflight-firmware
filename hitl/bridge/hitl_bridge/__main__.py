"""CLI entry point: `python -m hitl_bridge --port COM5 --source synthetic`."""

from __future__ import annotations

import argparse
import logging
import sys

from .bridge import Bridge
from .sensor_source import GazeboSensorSource, SyntheticSensorSource
from .serial_link import NullLink, SerialLink


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Wingflight HIL sensor injection bridge")
    parser.add_argument(
        "--source",
        choices=("synthetic", "gazebo"),
        default="synthetic",
        help="Sensor data source. 'synthetic' needs no Gazebo and is for "
        "bench/link testing; 'gazebo' subscribes to the real simulation "
        "(requires Phase A world + gz-transport bindings). Default: synthetic",
    )
    parser.add_argument(
        "--port",
        help="Serial port connected to the FC's FUNCTION_HIL_SENSOR UART "
        "(e.g. COM5 or /dev/ttyUSB0). Omit with --dry-run.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Don't open a real serial port; just encode packets (for "
        "testing the sensor source / rate in isolation).",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=250.0,
        help="Target packet send rate in Hz (default: 250).",
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable debug logging.")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-8s %(message)s",
    )

    if not args.dry_run and not args.port:
        print("error: --port is required unless --dry-run is set", file=sys.stderr)
        return 2

    if args.source == "synthetic":
        sensor_source = SyntheticSensorSource()
    else:
        sensor_source = GazeboSensorSource()

    link = NullLink() if args.dry_run else SerialLink(args.port)

    Bridge(sensor_source, link, rate_hz=args.rate_hz).run_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

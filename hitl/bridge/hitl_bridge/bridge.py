"""HIL bridge orchestrator: reads sensor samples from a SensorSource and
sends them over a SerialLink at a fixed rate.

See hitl/bridge/PROTOCOL.md for the wire format and
hitl/bridge/README.md for usage.
"""

from __future__ import annotations

import logging
import time

from .sensor_source import SensorSource

logger = logging.getLogger("hitl_bridge")


class Bridge:
    def __init__(self, sensor_source: SensorSource, link, rate_hz: float = 250.0):
        self._sensor_source = sensor_source
        self._link = link
        self._period_s = 1.0 / rate_hz
        self._sent_count = 0
        self._skipped_count = 0

    def run_forever(self) -> None:
        logger.info("HIL bridge started (target rate %.1f Hz)", 1.0 / self._period_s)
        try:
            while True:
                loop_start = time.monotonic()

                sample = self._sensor_source.read()
                if sample is not None:
                    self._link.send(sample)
                    self._sent_count += 1
                else:
                    self._skipped_count += 1

                elapsed = time.monotonic() - loop_start
                sleep_s = self._period_s - elapsed
                if sleep_s > 0:
                    time.sleep(sleep_s)
        except KeyboardInterrupt:
            logger.info(
                "Stopping (sent=%d, skipped-no-new-sample=%d)",
                self._sent_count,
                self._skipped_count,
            )
        finally:
            self._sensor_source.close()
            self._link.close()

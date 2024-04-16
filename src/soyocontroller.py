#!/usr/bin/env python3

from __future__ import annotations
from typing import Optional

import sys
import time
import math
import urllib
import signal
import logging
import argparse
import functools
import threading
import http.client
import paho.mqtt.client as mqtt


class SoyoController:
    def __init__(
        self,
        inverter_output_limits: tuple,
        holdoff_secs: int,
        dampening_factor: float,
        dampening_limit: int,
        mqtt_topic_currentdemand: str,
        url_setpoint: str,
        mqtt_server: str,
        mqtt_port: int = 1883,
        mqtt_topic_setpoint: Optional[str] = None,
    ):
        self._running = True
        self._holdoff_secs = holdoff_secs
        self._dampening_limit = dampening_limit
        self._dampening_factor = dampening_factor
        self._inverter_limits = inverter_output_limits

        self._mqtt_server_address = mqtt_server
        self._mqtt_server_port = mqtt_port
        self._mqtt_client = None

        self._url_setpoint = url_setpoint
        self._mqtt_topic_setpoint = mqtt_topic_setpoint
        self._mqtt_topic_currentdemand = mqtt_topic_currentdemand

        self._current_demand = 0.0
        self._last_measurement_dt = None
        self._last_measurement_used_dt = None
        self._current_setpoint = 0.0
        self._data_semaphore = threading.Semaphore()

        self._log = logging.getLogger("soyocontroller")

    def _subscribe_topics(
        self, 
        client, 
        userdata, 
        flags, 
        reason_code, 
        properties
    ):
        self._log.info(
            "Subscribing to MQTT topic: %s", 
            self._mqtt_topic_currentdemand)
        client.subscribe(self._mqtt_topic_currentdemand)

    def _on_message(self, client, userdata, msg):
        if msg.topic == self._mqtt_topic_currentdemand:
            self.set_current_demand(float(msg.payload))

    def set_current_demand(self, demand: float):
        self._log.debug(
            "Demand: %.fW", 
            demand, 
        )
        with self._data_semaphore:
            self._current_demand = demand
            self._last_measurement_dt = time.monotonic()

    def _calculate_setpoint(self) -> int:
        if self._current_demand < 0 and self._current_setpoint == 0:
            return 0  # Nothing to do here

        demand_to_consider = min(
            self._current_demand,
            self._inverter_limits[1],
            self._inverter_limits[1] - self._current_setpoint
        )
        setpoint_delta = demand_to_consider
        if setpoint_delta > self._dampening_limit:
            setpoint_delta *= self._dampening_factor  # Don't overshoot
        setpoint = min(
            self._inverter_limits[1],
            self._current_setpoint + setpoint_delta
        )
        setpoint = max(
            0,
            self._current_setpoint + setpoint_delta
        )

        self._log.info(
            "Current demand: %.fW, "
            "demand to consider: %.fW, "
            "setpoint = %dW + %dW = %dW",
            self._current_demand,
            demand_to_consider,
            self._current_setpoint,
            setpoint_delta,
            setpoint
        )
        return setpoint

    def set_output(self, setpoint: int) -> int:
        output = int(setpoint)
        self._log.info(
            "Adjusting setpoint: %dW => %dW",
            self._current_setpoint,
            output,
        )
        setpoint = max(
            min(
                self._current_setpoint + output, 
                self._inverter_limits[1]
            ),
            0
        )
        setpoint = output

        while True:
            try:
                rsp = urllib.request.urlopen(
                    urllib.request.Request(
                        url=f"{self._url_setpoint}?Value={setpoint}",
                        method='GET'
                    )
                )
                self._log.debug(
                    "Sent '%d' '%s', got HTTP/%s back: %s", 
                    setpoint,
                    self._url_setpoint, 
                    rsp.reason, 
                    rsp.msg
                )
                break
            except (
                urllib.error.URLError, 
                http.client.RemoteDisconnected,
                ConnectionResetError
            ) as e:
                self._log.warn("Could not transmit setpoint: %s. Retrying...")
                time.sleep(1.0)
        if self._mqtt_topic_setpoint and self._mqtt_client is not None:
            self._mqtt_client.publish(self._mqtt_topic_setpoint, setpoint)
        return setpoint


    def run(self):
        _ = self.set_output(0)
        self._log.debug("Connecting to MQTT...")
        self._mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self._mqtt_client.on_connect = self._subscribe_topics
        self._mqtt_client.on_message = self._on_message
        self._mqtt_client.connect(self._mqtt_server_address, self._mqtt_server_port)
        self._mqtt_client.loop_start()

        while self._running:
            time.sleep(self._holdoff_secs)
            if not self._running:
                break
            setpoint = 0
            with self._data_semaphore:
                if self._last_measurement_dt is None:
                    continue
                    self._log.debug("No data available yet, going back to sleep")
                if (
                    self._last_measurement_used_dt is not None 
                    and self._last_measurement_dt == self._last_measurement_used_dt
                ):
                    self._log.debug("No new data, going back to sleep")
                    continue
                setpoint = self._calculate_setpoint()
            self._current_setpoint = self.set_output(setpoint)   
            self._last_measurement_used_dt = self._last_measurement_dt
        self._log.info("Main loop ended")

    def stop(self):
        self._log.info("Stopping controller and cleaning up...")
        self._running = False
        self.set_output(0)
        self._mqtt_client.loop_stop()


def _setup_logging():
    logging.basicConfig(format="%(asctime)s %(message)s")
    logging.getLogger().setLevel(logging.INFO)

def _parse_argv():
    argparser = argparse.ArgumentParser(
        prog='soyocontroller',
        description="Controls a Soyosource grid tie inverter via KlausLi's controller and MQTT",
    )
    argparser.add_argument("--mqtt-server-address", required=True)
    argparser.add_argument("--mqtt-server-port", default=1883, type=int)
    argparser.add_argument("--mqtt-topic-currentdemand", required=True)
    argparser.add_argument("--mqtt-topic-setpoint", required=False)
    argparser.add_argument("--url-setpoint", required=True)
    argparser.add_argument(
        "--inverter-output-limit", 
        default=900, 
        type=int,
        help="Maximum output of the inverter"
    )
    argparser.add_argument(
        "--output-dampening-factor",
        default=0.65, 
        type=float,
        help="Dampening factor to multiply setpoints with to avoid overshooting",
    )
    argparser.add_argument(
        "--output-dampening-limit",
        default=30,
        type=int,
        help="Apply dampening factor for setpoints above this value"
    )
    argparser.add_argument(
        "--holdoff",
        default=3, 
        type=int,
        help="Time in seconds to sleep between adjustments"
    )
    argparser.add_argument(
        "--debug", 
        action="store_true", 
        help="Enable debugging output"
    )
    return argparser.parse_args()


def _sighandler_stop(signum, frame, controller):
    controller.stop()

def main():
    _setup_logging()
    args = _parse_argv()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    controller = SoyoController(
        inverter_output_limits=(
            0,
            args.inverter_output_limit,
        ),
        mqtt_server=args.mqtt_server_address,
        mqtt_port=args.mqtt_server_port,
        mqtt_topic_currentdemand=args.mqtt_topic_currentdemand,
        mqtt_topic_setpoint=args.mqtt_topic_setpoint,
        url_setpoint=args.url_setpoint,
        dampening_factor=args.output_dampening_factor,
        dampening_limit=args.output_dampening_limit,
        holdoff_secs=int(args.holdoff),
    )

    sighandler = functools.partial(_sighandler_stop, controller=controller)
    signal.signal(signal.SIGHUP, sighandler)
    signal.signal(signal.SIGINT, sighandler)
    signal.signal(signal.SIGTERM, sighandler)

    t = threading.Thread(target=controller.run())
    t.start()
    t.join()
    return 0

if __name__ == "__main__":
    exit(main())

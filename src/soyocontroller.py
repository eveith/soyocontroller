#!/usr/bin/env python3

from __future__ import annotations
from typing import Optional, List

import sys
import time
import math
import json
import urllib
import signal
import logging
import argparse
import functools
import threading
import http.client
from datetime import datetime
import paho.mqtt.client as mqtt
from collections import deque, namedtuple


WattageMeasurement = namedtuple("WattageMeasurement", ["t", "w"])
VoltageMeasurement = namedtuple("VoltageMeasurement", ["t", "v"])


class SoyoController:
    def __init__(
        self,
        inverter_output_limits: tuple,
        holdoff_secs: int,
        dampening_factor: float,
        dampening_limit: int,
        mqtt_topic_currentdemand: str,
        mqtt_server: str,
        battery_voltage_low_cutoff = 0.0,
        battery_voltage_reconnect = 0.0,
        url_setpoint: Optional[str] = None,
        mqtt_topic_setpoint: Optional[str] = None,
        mqtt_topics_epever_chargers: Optional[List[str]] = None,
        mqtt_port: int = 1883,
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
        self._mqtt_topics_epever_chargers = (
            mqtt_topics_epever_chargers if mqtt_topics_epever_chargers 
            else []
        )

        self._battery_voltages = deque(maxlen=5)
        self._battery_is_recharging = False
        self._battery_voltage_reconnect = battery_voltage_reconnect
        self._battery_voltage_low_cutoff = battery_voltage_low_cutoff

        self._demands = deque(maxlen=5)
        self._last_demand_measurement_used_t = None

        self._current_setpoint = 0.0
        self._current_setpoint_t = None

        self._log = logging.getLogger("soyocontroller")

    @property
    def current_demand(self) -> float:
        if len(self._demands) == 0:
            return 0.0
        if len(self._demands) == 1:
            return self._demands[-1].w
        if len(self._demands) == 2:
            return 0.4 * self._demands[-2].w + 0.6 * self._demands[-1].w
        return (
            0.1 * self._demands[-3].w 
            + 0.3 * self._demands[-2].w 
            + 0.6 * self._demands[-1].w
        )

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
        for t in self._mqtt_topics_epever_chargers:
            client.subscribe(t)

    def _on_message(self, client, userdata, msg):
        if msg.topic == self._mqtt_topic_currentdemand:
            self.set_current_demand(float(msg.payload))
        if msg.topic in self._mqtt_topics_epever_chargers:
            data = json.loads(msg.payload)
            try:
                self._battery_voltages.append(
                    VoltageMeasurement(t=datetime.now(), v=float(data["BatteryV"]))
                )
                self._log.debug(
                    "Current battery voltage: %.2fV", 
                    self._battery_voltages[-1].v
                )
            except KeyError:
                self._log.warning(
                    "Epever JSON payload does not contain key 'BatteryV', "
                    "cannot react to battery state. Payload is: %s",
                    data
                )

    def set_current_demand(self, demand: float):
        self._log.debug(
            "Demand: %.fW", 
            demand, 
        )
        self._demands.append(
            WattageMeasurement(t=datetime.now(), w=demand)
        )

    def _calculate_setpoint(self) -> int:
        if self.current_demand < 0.0 and self._current_setpoint == 0:
            return 0  # Nothing to do here

        # Try to cater to the battery's needs:

        if (
            len(self._battery_voltages) > 0 
            and self._battery_voltages[-1].v 
                <= self._battery_voltage_low_cutoff
        ):
            self._log.info(
                "Battery at low voltage cutoff: %.2fV <= %.f2V",
                self._battery_voltages[-1].v,
                self._battery_voltage_low_cutoff
            )
            self._battery_is_recharging = True
            return 0  # Low discharge protection
        if (
            self._battery_is_recharging
            and len(self._battery_voltages) > 1
            and self._battery_voltages[-1].v >= self._battery_voltages[-2].v
            and self._battery_voltages[-1].v < self._battery_voltage_reconnect
        ):
            self._log.info(
                "Battery recharging: %s, "
                "below reconnect voltage: %.2fV <= %.2fV",
                self._battery_voltages,
                self._battery_voltages[-1].v,
                self._battery_voltage_reconnect
            )
            return 0  # Let the battery recharge

        demand_to_consider = min(
            self.current_demand,
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
        setpoint = int(setpoint)

        self._log.info(
            "Current demand: %.fW"
            " demand to consider: %.fW, "
            "setpoint = %dW + %dW = %dW",
            self.current_demand,
            demand_to_consider,
            self._current_setpoint,
            setpoint_delta,
            setpoint
        )
        return setpoint

    def set_output(self, setpoint: int):
        self._log.info(
            "Adjusting setpoint: %dW => %dW",
            self._current_setpoint,
            setpoint,
        )
        if self._mqtt_topic_setpoint and self._mqtt_client is not None:
            self._mqtt_client.publish(self._mqtt_topic_setpoint, setpoint)
        if self._url_setpoint:
            self._set_output_http(setpoint)
        self._current_setpoint_t = datetime.now()

    def _set_output_http(self, setpoint: int):
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


    def run(self):
        _ = self.set_output(0)
        self._log.debug("Connecting to MQTT...")
        self._mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self._mqtt_client.on_connect = self._subscribe_topics
        self._mqtt_client.on_message = self._on_message
        self._mqtt_client.connect(self._mqtt_server_address, self._mqtt_server_port)
        self._mqtt_client.loop_start()

        while self._running:
            time.sleep(0.1)
            if not self._running:
                break
            if len(self._demands) == 0:
                self._log.debug("No data available yet, going back to sleep")
                continue
            if self._demands[-1].t == self._last_demand_measurement_used_t:
                self._log.debug("Stale data, waiting...")
                continue
            now = datetime.now()
            if (
                self._current_setpoint_t is not None
                and (
                    (now - self._current_setpoint_t).seconds 
                    <= self._holdoff_secs
                )
            ):
                self._log.debug(
                    "Holding off, only %s elapsed...", 
                    (now-self._current_setpoint_t)
                )
                continue
            setpoint = self._calculate_setpoint()
            self.set_output(setpoint)
            self._current_setpoint = setpoint
            self._last_demand_measurement_used_t = self._demands[-1].t
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
    argparser.add_argument(
        "--mqtt-topic-epever-charger", 
        action="append", 
        required=False, 
        help="MQTT topics of Epever charger(s), can be given multiple times"
    )
    argparser.add_argument(
        "--url-setpoint", 
        action="store",
        type=str,
        required=False,
        help="URL to HTTP GET the setpoint to",
    )
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
        "--battery-low-cutoff-voltage",
        type=float,
        default=0.0,
        required=False,
        help="Voltage value below which the inverter will stop feed-in"
    )
    argparser.add_argument(
        "--battery-reconnect-voltage",
        type=float,
        required=False,
        default=0.0,
        help="Voltage at which the inverter re-connects after recharging",
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
        mqtt_topics_epever_chargers=args.mqtt_topic_epever_charger,
        url_setpoint=args.url_setpoint,
        dampening_factor=args.output_dampening_factor,
        dampening_limit=args.output_dampening_limit,
        holdoff_secs=int(args.holdoff),
        battery_voltage_low_cutoff=args.battery_low_cutoff_voltage,
        battery_voltage_reconnect=args.battery_reconnect_voltage,
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

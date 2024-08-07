#!/usr/bin/env python3

import re
import sys
import time
import argparse
import paho.mqtt.client as mqtt
from subprocess import Popen, PIPE

POWER_CURR_RE = re.compile(r"^1-0:16\.7\.0\*255#([^#]+)#W$")
TOTAL_DEMAND_RE = re.compile(r"1-0:1\.8\.0\*255#([^#]+)#Wh")

def _parse_argv():
    argparser = argparse.ArgumentParser(
        prog='sml2mqtt',
        description="Read meter values from sml_server and push them to mqtt"
    )
    argparser.add_argument("--mqtt-server-address", required=True)
    argparser.add_argument("--mqtt-server-port", default=1883, type=int)
    argparser.add_argument("--mqtt-topic-currentdemand", required=True)
    argparser.add_argument("--mqtt-topic-totalconsumption", required=True)
    argparser.add_argument("--meter-device", default="/dev/ttyUSB0")
    return argparser.parse_args()

def try_publish_power_curr(mqtt_client, topic, line):
    m = POWER_CURR_RE.search(line)
    if not m:
        return
    power_curr = float(m.group(1))
    mqtt_client.publish(topic, power_curr)

def try_publish_total_demand(mqtt_client, topic, line):
    m = TOTAL_DEMAND_RE.search(line)
    if not m:
        return
    total_demand_kwh = float(m.group(1)) / 1000.0
    mqtt_client.publish(topic, total_demand_kwh)


def main():
    args = _parse_argv()
    mqtt_client = mqtt.Client()
    while True:
        try:
            mqtt_client.connect(
                args.mqtt_server_address,
                args.mqtt_server_port
            )
            break
        except OSError as e:
            print(
                f"Could not connect to MQTT: {e}, retrying...", 
                file=sys.stderr
            )
            time.sleep(3)
    mqtt_client.loop_start()
    while True:
        with Popen(
            ["/usr/bin/sml_server", args.meter_device], 
            stdout=PIPE,
            text=True
        ) as fp:
            for line in fp.stdout:
                try_publish_power_curr(
                    mqtt_client, 
                    args.mqtt_topic_currentdemand, 
                    line
                )
                try_publish_total_demand(
                    mqtt_client, 
                    args.mqtt_topic_totalconsumption,
                    line
                )
        print("sml_server exited, restarting...", file=sys.stderr)

if __name__ == "__main__":
    main()

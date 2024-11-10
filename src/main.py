import argparse
import logging.config
import os
import time

import requests as requests
from dotenv import load_dotenv

from service import data


def send_json_to_server(data: str):
    url = os.getenv("BASE_URL")
    if url is None or url == "":
        raise ValueError("BASE_URL is not set")

    requests.post(os.getenv("BASE_URL"), data=data)
    log.info("Data sent to server")


def get_interval() -> float:
    interval = os.getenv("INTERVAL")
    if interval is None or interval == "":
        raise ValueError("INTERVAL is not set")
    return float(interval)


def handle_arguments():
    parser = argparse.ArgumentParser(description="Send sensor data to server")
    parser.add_argument(
        "--i",
        type=float,
        help="Interval in seconds to send sensor data to server",
        default=60
    )
    parser.add_argument(
        "--u",
        type=str,
        help="URL to send sensor data to"
    )

    args = parser.parse_args()

    if args.u is not None:
        os.environ["BASE_URL"] = args.u
    if args.i is not None:
        os.environ["INTERVAL"] = str(args.i)


def main():
    logging.config.fileConfig('config/logging.conf')
    log = logging.getLogger(__name__)
    log.info("Starting sensor data client.")
    load_dotenv()
    log.info("Starting sensor data client..")
    handle_arguments()
    log.info("Starting sensor data client...")
    interval = get_interval()

    while True:
        log.info("Sending sensor data to server...")
        data.build_sensor_json()
        send_json_to_server(data.build_sensor_json())
        time.sleep(interval)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        log.info("Exiting...")
        exit(0)

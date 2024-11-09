import json
import logging
import os
import time
import uuid

import service as sensor

log = logging.getLogger(__name__)


def __create_sensor_info() -> dict[str, any]:
    try:
        output = {
            "id": f"{os.getenv('DEVICE_ID')}",
            "uuid": str(uuid.uuid4()),
            "pressure": sensor.pressure(),
            "temperature": sensor.get_temperature(),
            "humidity": sensor.get_humidity(),
            "light": sensor.get_light(),
            "uv": sensor.get_uv(),
            "gas": sensor.get_gas(),
            "roll": sensor.get_roll(),
            "pitch": sensor.get_pitch(),
            "yaw": sensor.get_yaw(),
            "acceleration": sensor.get_acceleration(),
            "gyroscope": sensor.get_gyroscope(),
            "time": time.time()
        }
        log.info("Sensor data created successfully")
        return output
    except Exception as e:
        log.error(f"Error creating sensor data: {e}")
        return {}


def build_sensor_json() -> str:
    return json.dumps(__create_sensor_info())

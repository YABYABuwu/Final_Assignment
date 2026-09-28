"""Stationary IR wiring/polarity check. Run: python test_rear_ir.py"""

import argparse
import time

from src.config_loader import load_config
from src.logger import SensorLogger
from src.rear_ir import RearIRBumper, adapter_index, recovery_vector


def main():
    from robomaster import conn, robot

    parser = argparse.ArgumentParser(description="Stationary rear IR wiring check")
    parser.add_argument("--direct", action="store_true",
                        help="also query SDK get_io/get_adc for ID 3/4 ports 1/2")
    args = parser.parse_args()
    config = load_config()
    settings = config["rear_ir"]
    ep_robot = robot.Robot()
    connected = False
    logger = None
    try:
        connection = {"ap": conn.CONNECTION_WIFI_AP,
                      "sta": conn.CONNECTION_WIFI_STA,
                      "rndis": conn.CONNECTION_USB_RNDIS}[config["connection"]["type"]]
        ep_robot.initialize(conn_type=connection)
        connected = True
        log_settings = config["logging"].copy()
        log_settings["streams"] = {"adapter": {"enabled": True, "save": False,
                                                "frequency_hz": 20}}
        logger = SensorLogger(ep_robot, log_settings)
        logger.start()
        bumper = RearIRBumper(logger, settings)
        print("ขณะทดสอบให้รถอยู่นิ่ง; นำวัตถุเข้าใกล้ IR ทีละด้าน กด Ctrl+C เพื่อจบ")
        print("ID 3/port 1 = IO index 4, ID 4/port 1 = IO index 6")
        last_direct = 0.0
        while True:
            state = bumper.snapshot()
            sample = logger.get_latest("adapter", max_age_s=settings["max_age_s"])
            parts = []
            for side in ("right", "left"):
                port = settings[side]
                index = adapter_index(port["id"], port["port"])
                io = state["sides"][side]["io"]
                adc = sample[12 + index] if sample is not None and len(sample) > 12 + index else None
                parts.append("{}: IO={} ADC={} detected={}".format(
                    side, io, adc, state["sides"][side]["detected"]))
            right = state["sides"]["right"]["detected"]
            left = state["sides"]["left"]["detected"]
            if right is None or left is None:
                escape = "wait_data"
            else:
                direction, escape_x, escape_y = recovery_vector(state["sides"], 1.0)
                escape = "{} (x={:.0f}, y={:.0f})".format(
                    direction, escape_x, escape_y) if direction != "clear" else "none"
            raw = []
            for sensor_id in (3, 4):
                for sensor_port in (1, 2):
                    index = adapter_index(sensor_id, sensor_port)
                    io = sample[index] if sample is not None and len(sample) > index else None
                    adc = sample[12 + index] if sample is not None and len(sample) > 12 + index else None
                    raw.append("ID{} P{}: IO={} ADC={}".format(sensor_id, sensor_port, io, adc))
            print("{} | {} | escape={} | raw: {}".format(
                state["state"], " | ".join(parts), escape, " | ".join(raw)), flush=True)
            # Synchronous SDK reads are optional and limited to once per second.
            if args.direct and time.monotonic() - last_direct >= 1.0:
                last_direct = time.monotonic()
                direct = []
                for sensor_id in (3, 4):
                    for sensor_port in (1, 2):
                        io = ep_robot.sensor_adaptor.get_io(id=sensor_id, port=sensor_port)
                        adc = ep_robot.sensor_adaptor.get_adc(id=sensor_id, port=sensor_port)
                        direct.append("ID{} P{}: IO={} ADC={}".format(
                            sensor_id, sensor_port, io, adc))
                print("direct: {}".format(" | ".join(direct)), flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("หยุดทดสอบ")
    finally:
        if logger is not None:
            logger.stop()
        if connected:
            ep_robot.close()


if __name__ == "__main__":
    main()

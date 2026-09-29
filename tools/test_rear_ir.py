"""Stationary front and rear IR wiring/polarity check. Run: python test_rear_ir.py"""

import argparse
import time

from src.config_loader import load_config
from src.logger import SensorLogger
from src.rear_ir import FrontIRBumper, RearIRBumper, adapter_index, recovery_vector


def main():
    from robomaster import conn, robot

    parser = argparse.ArgumentParser(description="Stationary front and rear IR wiring check")
    parser.add_argument("--direct", action="store_true",
                        help="also query SDK get_io/get_adc for configured IR ports")
    args = parser.parse_args()
    config = load_config()
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
        bumpers = {"rear": RearIRBumper(logger, config["rear_ir"]),
                   "front": FrontIRBumper(logger, config["front_ir"])}
        ports = sorted({(config[end + "_ir"][side]["id"],
                         config[end + "_ir"][side]["port"])
                        for end in ("rear", "front") for side in ("right", "left")})
        print("ขณะทดสอบให้รถอยู่นิ่ง; นำวัตถุเข้าใกล้ IR ทีละด้าน กด Ctrl+C เพื่อจบ")
        for end, settings in ((end, config[end + "_ir"]) for end in ("rear", "front")):
            for side in ("right", "left"):
                port = settings[side]
                print("{} {}: ID {} port {} = io_{}; active_io={}; enabled={}".format(
                    end, side, port["id"], port["port"],
                    adapter_index(port["id"], port["port"]) + 1,
                    port["active_io"], settings["enabled"]))
        last_direct = 0.0
        while True:
            sample = logger.get_latest("adapter", max_age_s=config["rear_ir"]["max_age_s"])
            parts = []
            for end, bumper in bumpers.items():
                state = bumper.snapshot()
                settings = config[end + "_ir"]
                for side in ("right", "left"):
                    port = settings[side]
                    index = adapter_index(port["id"], port["port"])
                    io = state["sides"][side]["io"]
                    adc = sample[12 + index] if sample is not None and len(sample) > 12 + index else None
                    parts.append("{} {}: IO={} ADC={} detected={}".format(
                        end, side, io, adc, state["sides"][side]["detected"]))
                if all(sensor["detected"] is not None for sensor in state["sides"].values()):
                    mode = settings.get("recovery_mode", "diagonal")
                    direction, escape_x, escape_y = recovery_vector(
                        state["sides"], 1.0, end=end, mode=mode)
                    parts.append("{} escape={} (x={:.0f}, y={:.0f})".format(
                        end, direction, escape_x, escape_y))
            raw = []
            for sensor_id, sensor_port in ports:
                index = adapter_index(sensor_id, sensor_port)
                io = sample[index] if sample is not None and len(sample) > index else None
                adc = sample[12 + index] if sample is not None and len(sample) > 12 + index else None
                raw.append("ID{} P{}: IO={} ADC={}".format(sensor_id, sensor_port, io, adc))
            print("{} | raw: {}".format(" | ".join(parts), " | ".join(raw)), flush=True)
            # Synchronous SDK reads are optional and limited to once per second.
            if args.direct and time.monotonic() - last_direct >= 1.0:
                last_direct = time.monotonic()
                direct = []
                for sensor_id, sensor_port in ports:
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

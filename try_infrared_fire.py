"""One stationary infrared command to verify the EP blaster.

Run from the project root: python try_infrared_fire.py --fire
The flag prevents an accidental shot when checking --help or importing this file.
"""

import argparse


def main():
    parser = argparse.ArgumentParser(description="Send one RoboMaster EP infrared fire command")
    parser.add_argument("--fire", action="store_true", help="send exactly one infrared command")
    parser.add_argument("--connection", choices=("ap", "sta", "rndis"), default="ap")
    args = parser.parse_args()
    if not args.fire:
        parser.error("add --fire to send the command")

    from robomaster import blaster, conn, robot

    connection = {
        "ap": conn.CONNECTION_WIFI_AP,
        "sta": conn.CONNECTION_WIFI_STA,
        "rndis": conn.CONNECTION_USB_RNDIS,
    }[args.connection]
    ep = robot.Robot()
    connected = False
    try:
        ep.initialize(conn_type=connection)
        connected = True
        accepted = ep.blaster.fire(fire_type=blaster.INFRARED_FIRE, times=1)
        print("Infrared command accepted by SDK" if accepted else
              "Infrared command not accepted; actual firing state unknown")
    finally:
        if connected:
            ep.close()


if __name__ == "__main__":
    main()

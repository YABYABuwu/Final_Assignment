"""Test script to play audio / startup sound on RoboMaster EP.

Usage:
    python Final_Assignment/tools/test_sound.py --connection ap
    python Final_Assignment/tools/test_sound.py --connection ap --volume 0.08
    python Final_Assignment/tools/test_sound.py --connection ap --sound fah.mp3 --volume 0.15
"""

import argparse
from pathlib import Path
import sys

# Ensure Final_Assignment directory is in sys.path
PROJECT_DIR = Path(__file__).resolve().parent.parent
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from src.sound_player import play_startup_sound


def main():
    parser = argparse.ArgumentParser(description="Test audio playback on RoboMaster EP")
    parser.add_argument("--connection", choices=("ap", "sta", "rndis"), default="ap",
                        help="Connection type (default: ap)")
    parser.add_argument("--sound", default="startup.wav",
                        help="Sound file name in sfx/ (default: startup.wav)")
    parser.add_argument("--volume", type=float, default=0.16,
                        help="Volume factor, 0.08 is very quiet, 0.16 is medium-quiet (default: 0.16)")
    args = parser.parse_args()

    from robomaster import conn, robot

    connection = {
        "ap": conn.CONNECTION_WIFI_AP,
        "sta": conn.CONNECTION_WIFI_STA,
        "rndis": conn.CONNECTION_USB_RNDIS,
    }[args.connection]

    print(f"Connecting to RoboMaster EP ({args.connection.upper()})...")
    ep = robot.Robot()
    connected = False
    try:
        ep.initialize(conn_type=connection)
        connected = True
        print(f"Connected! Playing sound '{args.sound}' (volume factor: {args.volume})...")
        action = play_startup_sound(ep, sound_file=args.sound, volume_factor=args.volume, wait=True)
        if action:
            print("Sound playback completed successfully!")
        else:
            print("Could not play sound.")
    except Exception as e:
        print(f"Error during audio test: {e}")
    finally:
        if connected:
            print("Closing connection...")
            ep.close()


if __name__ == "__main__":
    main()

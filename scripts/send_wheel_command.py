#!/usr/bin/env python3

import argparse
import time

import serial


def main() -> int:
    parser = argparse.ArgumentParser(description="Send wheel RPM commands over USB serial.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port path")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument("--right", type=int, default=100, help="Right wheel RPM")
    parser.add_argument("--left", type=int, default=-100, help="Left wheel RPM")
    parser.add_argument("--period", type=float, default=0.1, help="Send period in seconds")
    parser.add_argument("--count", type=int, default=0, help="Number of messages to send. 0 means forever.")
    args = parser.parse_args()

    line = f"{args.left:+d},{args.right:+d}\n".encode("ascii")

    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        sent = 0
        while True:
            ser.write(line)
            ser.flush()
            sent += 1
            print(line.decode("ascii").strip(), flush=True)
            if args.count and sent >= args.count:
                break
            time.sleep(args.period)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

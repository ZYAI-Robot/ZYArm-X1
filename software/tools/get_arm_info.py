#!/usr/bin/env python3
import serial
import argparse
import re
import sys


CMD_ID_GET_NAME = 22
CMD_ID_GET_VERSION = 4


def send_command(ser, cmd_id, params=None):
    if params:
        param_str = " ".join(str(p) for p in params)
        cmd = f"[CMD][{cmd_id}][{param_str}]\n"
    else:
        cmd = f"[CMD][{cmd_id}]\n"

    ser.write(cmd.encode('utf-8'))
    return cmd


def read_response(ser, timeout=2.0):
    ser.timeout = timeout
    lines = []

    while True:
        line = ser.readline()
        if not line:
            break
        try:
            lines.append(line.decode('utf-8').strip())
        except UnicodeDecodeError:
            lines.append(line.decode('latin-1').strip())

    return lines


def get_robot_name(ser):
    send_command(ser, CMD_ID_GET_NAME)
    lines = read_response(ser)

    for line in lines:
        match = re.search(r'NAME:(\S+)', line)
        if match:
            return match.group(1)

        if 'ACK_COMPLETED' in line:
            error_match = re.search(r'ERROR', line)
            if error_match:
                return None

    return None


def get_robot_version(ser):
    send_command(ser, CMD_ID_GET_VERSION)
    lines = read_response(ser)

    version_info = {}
    for line in lines:
        hw_match = re.search(r'Hardware Version:\s*(\S+)', line)
        if hw_match:
            version_info['hardware'] = hw_match.group(1)

        sw_match = re.search(r'Software Version:\s*(\S+)', line)
        if sw_match:
            version_info['software'] = sw_match.group(1)

        build_match = re.search(r'Build Date:\s*(.+)', line)
        if build_match:
            version_info['build'] = build_match.group(1).strip()

    return version_info if version_info else None


def main():
    parser = argparse.ArgumentParser(description='Get robot arm info via serial port')
    parser.add_argument('port', help='Serial port name (e.g., COM3 or /dev/ttyUSB0)')
    parser.add_argument('-b', '--baud', type=int, default=512000, help='Baud rate (default: 512000)')
    parser.add_argument('-t', '--timeout', type=float, default=2.0, help='Response timeout in seconds (default: 2.0)')
    parser.add_argument('--name', action='store_true', help='Get robot arm name')
    parser.add_argument('--version', action='store_true', help='Get robot arm version')

    args = parser.parse_args()

    if not args.name and not args.version:
        args.name = True
        args.version = True

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=args.timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
    except serial.SerialException as e:
        print(f"Error: Cannot open serial port {args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        if args.name:
            name = get_robot_name(ser)
            if name:
                print(f"Name: {name}")
            else:
                print("Error: Failed to get robot name", file=sys.stderr)

        if args.version:
            version_info = get_robot_version(ser)
            if version_info:
                if 'hardware' in version_info:
                    print(f"Hardware: {version_info['hardware']}")
                if 'software' in version_info:
                    print(f"Software: {version_info['software']}")
                if 'build' in version_info:
                    print(f"Build: {version_info['build']}")
            else:
                print("Error: Failed to get robot version", file=sys.stderr)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        ser.close()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Manual Motor Test — Jetson Nano → STM32
========================================
Run this on the Jetson Nano to manually send commands
to the STM32 and verify each motor movement.

Wiring:
  Jetson Pin 8  (TX) → STM32 PA10 (RX)
  Jetson Pin 10 (RX) → STM32 PA9  (TX)
  Jetson Pin 6  (GND) → STM32 GND

Usage:
  python3 manual_motor_test.py
"""

import serial
import time

# --- CONFIGURATION ---
STM32_PORT = '/dev/ttyTHS1'   # Jetson Nano UART1
BAUD_RATE  = 115200

COMMANDS = {
    'F': 'FORWARD      — Both motors full speed forward',
    'W': 'SLOW FORWARD — Both motors slow forward (LiDAR warning)',
    'B': 'REVERSE      — Both motors reverse',
    'L': 'LEFT         — Turn left (right fast, left slow)',
    'R': 'RIGHT        — Turn right (right slow, left fast)',
    'X': 'SPIN         — Rotate in place',
    'S': 'STOP         — All motors stop',
}


def main():
    print("=" * 50)
    print("  AGV Manual Motor Test")
    print("  Jetson Nano → STM32 UART")
    print("=" * 50)

    try:
        stm = serial.Serial(STM32_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for serial connection to stabilise
        print(f"\n✅ Connected to STM32 on {STM32_PORT}")
    except Exception as e:
        print(f"\n❌ Cannot open {STM32_PORT}: {e}")
        print("\nTroubleshooting:")
        print("  1. Check wiring (TX→RX, RX→TX, GND→GND)")
        print("  2. Run: sudo systemctl stop nvgetty")
        print("  3. Run: sudo chmod 666 /dev/ttyTHS1")
        print("  4. Run: ls -l /dev/ttyTHS1")
        return

    print("\nAvailable commands:")
    for key, desc in COMMANDS.items():
        print(f"  [{key}] {desc}")
    print(f"  [Q] Quit\n")

    # Safety: stop motors on start
    stm.write(b'S')
    print("Motors stopped (safety reset)\n")

    while True:
        try:
            user_input = input("Enter command (F/B/L/R/X/S/Q): ").strip().upper()

            if user_input == 'Q':
                stm.write(b'S')
                print("Motors stopped. Exiting.")
                break

            if user_input in COMMANDS:
                stm.write(user_input.encode())
                print(f"  → Sent '{user_input}' : {COMMANDS[user_input]}")

                # Auto-stop after 2 seconds for safety (except if already stop)
                if user_input != 'S':
                    print("    (Auto-stop in 2 seconds...)")
                    time.sleep(2)
                    stm.write(b'S')
                    print("    → Auto-stopped")
            else:
                print(f"  ⚠ Unknown command '{user_input}'. Use F/B/L/R/X/S/Q")

        except KeyboardInterrupt:
            stm.write(b'S')
            print("\n\nMotors stopped (Ctrl+C). Exiting.")
            break

    stm.close()
    print("Serial port closed.")


if __name__ == '__main__':
    main()

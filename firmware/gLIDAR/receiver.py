#!/usr/bin/env python3
import serial
import struct

PORT = "/dev/ttyUSB0"   # zmień na odpowiedni port, np. COM3 w Windows
BAUDRATE = 115200

def main():
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    buffer = bytearray()

    print("Listening on", PORT)

    try:
        while True:
            # czytamy dane z seriala
            data = ser.read(ser.in_waiting or 1)
            if data:
                buffer.extend(data)

                # szukamy nagłówka i stopki
                while b"LIDAR" in buffer and b"END\n\r" in buffer:
                    start = buffer.find(b"LIDAR")
                    end = buffer.find(b"END\n\r", start)
                    if end == -1:
                        break

                    frame = buffer[start:end+len(b"END\n\r")]
                    buffer = buffer[end+len(b"END\n\r"):]  # usuwamy już przeczytane

                    # parsujemy ramkę
                    if len(frame) > 7:  # minimalny rozmiar = nagłówek + size + stopka
                        size = frame[5] | (frame[6] << 8)
                        payload = frame[7:-5]  # wyciągamy dane (distance/strength)

                        print(f"\n📡 Frame received: {len(payload)} bytes (expected {size})")
                        for i in range(0, len(payload), 4):
                            if i + 3 < len(payload):
                                dist = payload[i] | (payload[i+1] << 8)
                                strength = payload[i+2] | (payload[i+3] << 8)
                                print(f"  Distance: {dist:4d} cm | Strength: {strength:4d}")

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()


if __name__ == "__main__":
    main()

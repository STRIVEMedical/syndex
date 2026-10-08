import serial
import threading
import ports

I2C_PORT, CAN_PORT = ports.identify_devices()

def read_serial(port, label):
    try:
        with serial.Serial(port, 115200, timeout=1) as ser:
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"[{label}] {line}")
    except serial.SerialException as e:
        print(f"[{label}] ERROR: {e}")

if __name__ == "__main__":
    t1 = threading.Thread(target=read_serial, args=(I2C_PORT, "I2C"))
    t2 = threading.Thread(target=read_serial, args=(CAN_PORT, "CAN"))

    t1.start()
    t2.start()

    t1.join()
    t2.join()
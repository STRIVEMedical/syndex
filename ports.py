import serial
import serial.tools.list_ports

def find_ports():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        print(f"{p.device}: {p.description}")
    return [p.device for p in ports]

def identify_devices():
    ports = serial.tools.list_ports.comports()
    i2c_port, can_port = None, None

    for p in ports:
        try:
            with serial.Serial(p.device, 115200, timeout=1) as ser:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if "I2C READY" in line:
                    i2c_port = p.device
                elif "CAN READY" in line:
                    can_port = p.device
        except:
            continue
    return i2c_port, can_port

I2C_PORT, CAN_PORT = identify_devices()

def main():
    print(f"I2C PORT: {I2C_PORT}")
    print(f"CAN PORT: {CAN_PORT}")

if __name__ == '__main__':
    main()
import serial.tools.list_ports

def find_arduino_port():
    # Common Arduino VIDs (e.g., 0x2341 for official boards)
    arduino_vid = 0x2341 
    ports = serial.tools.list_ports.comports()
    print(f"Available serial ports: {len(ports)}")
    for port in ports:
        print(f"Checking port: {port.device}, VID: {port.vid}, PID: {port.pid}")
        if port.vid == arduino_vid:
            return port.device
    return None

if __name__ == "__main__":
    arduino_port = find_arduino_port()
    if arduino_port:
        print(f"Arduino found on port: {arduino_port}")
    else:
        print("Arduino not found.")
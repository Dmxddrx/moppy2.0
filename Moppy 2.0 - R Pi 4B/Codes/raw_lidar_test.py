#!/usr/bin/env python3
import serial
import time

def main():
    # Update to /dev/ttyUSB1 if your port changed again
    port_name = '/dev/ttyUSB0' 
    baud_rate = 115200
    
    try:
        ser = serial.Serial(port_name, baud_rate, timeout=0.1)
        print(f"[*] Successfully opened {port_name} at {baud_rate} baud.")
        print("[*] Listening for raw LiDAR data... (Press Ctrl+C to stop)\n")
        
        while True:
            if ser.in_waiting > 0:
                # Read whatever is currently in the buffer
                raw_bytes = ser.read(ser.in_waiting)
                
                # Format as readable hex values (e.g., 'aa 55 00 ...')
                hex_string = ' '.join([f'{b:02x}' for b in raw_bytes])
                print(f"[{len(raw_bytes):03d} bytes] : {hex_string}")
                
            time.sleep(0.05) # Prevent maxing out the CPU
            
    except serial.SerialException as e:
        print(f"[!] Hardware Error: {e}")
    except KeyboardInterrupt:
        print("\n[*] Test stopped by user.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()

import serial
import csv
import time

SERIAL_PORT = 'COM3'  # Change to your serial port, e.g., '/dev/ttyUSB0' for Linux
BAUD_RATE = 9600      # Adjust as necessary
CSV_FILE = 'data.csv'

def main():
    # Open serial port
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Allow some time for the connection to initialize

    with open(CSV_FILE, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        # Write header row
        csvwriter.writerow(['AccelometerX', 'AccelometerY', 'AccelometerZ'])

        try:
            while True:
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='replace').strip()
                    if not line:
                        continue
                    
                    # Data is expected to be tab-separated
                    parts = line.split('\t')
                    if len(parts) >= 3:
                        try:
                            x = float(parts[0])
                            y = float(parts[1])
                            z = float(parts[2])
                            # Write to CSV and print the data
                            csvwriter.writerow([x, y, z])
                            csvfile.flush()
                            print(f"X: {x}, Y: {y}, Z: {z}")
                        except ValueError:
                            print("Received data in unexpected format:", line)
                    else:
                        print("Data does not have 3 parts:", line)
        except KeyboardInterrupt:
            print("Program terminated by user.")
        finally:
            ser.close()

if __name__ == '__main__':
    main()
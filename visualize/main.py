import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

def read_serial_data(ser):
    try:
        data = ser.readline().decode('utf-8').strip()
        if data:
            values = data.split("\t")
            if len(values) == 6:
                return list(map(float, values))
    except:
        pass
    return None

def update_plot(frame, ser, accel_lines, gyro_lines, accel_data, gyro_data):
    data = read_serial_data(ser)
    if data:
        ax, ay, az, gx, gy, gz = data
        print(data)
        accel_data[0].append(ax)
        accel_data[1].append(ay)
        accel_data[2].append(az)
        gyro_data[0].append(gx)
        gyro_data[1].append(gy)
        gyro_data[2].append(gz)

        for i, line in enumerate(accel_lines):
            line.set_ydata(accel_data[i])
        for i, line in enumerate(gyro_lines):
            line.set_ydata(gyro_data[i])

    return accel_lines + gyro_lines

def main():
    comport = 'COM3'
    baudrate = 9600
    ser = serial.Serial(comport, baudrate, timeout=0.025)

    max_len = 100
    accel_data = [deque([0]*max_len, maxlen=max_len) for _ in range(3)]
    gyro_data = [deque([0]*max_len, maxlen=max_len) for _ in range(3)]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 5))
    fig.suptitle('MPU6050 Real-time Data Visualization')

    # Accelerometer plot
    ax1.set_title('Accelerometer (X, Y, Z)')
    ax1.set_ylim(-7, 7)
    ax1.set_xlim(0, max_len)
    accel_lines = [ax1.plot(range(max_len), accel_data[i])[0] for i in range(3)]

    # Gyroscope plot
    ax2.set_title('Velocity (X, Y, Z)')
    ax2.set_ylim(-7, 7)
    ax2.set_xlim(0, max_len)
    gyro_lines = [ax2.plot(range(max_len), gyro_data[i])[0] for i in range(3)]

    ani = animation.FuncAnimation(fig, update_plot, fargs=(ser, accel_lines, gyro_lines, accel_data, gyro_data),
                                  interval=25, blit=True)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()

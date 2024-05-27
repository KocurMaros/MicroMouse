import socket
import matplotlib.pyplot as plt
import numpy as np
import time
from collections import deque

localIP = "192.168.137.1"
localPort = 3333
bufferSize = 1024

msgFromServer = "Hello UDP Client"
bytesToSend = str.encode(msgFromServer)

# Create a datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# Bind to address and ip
UDPServerSocket.bind((localIP, localPort))

print("UDP server up and listening")

fig, axs = plt.subplots(3, figsize=(10, 8))
plt.subplots_adjust(hspace=0.5)  # Add space between subplots

# Use deque for efficient data handling
max_length = 100
timestampArray = deque(maxlen=max_length)
motorArrayA = deque(maxlen=max_length)
motorArrayB = deque(maxlen=max_length)
gyroZarray = deque(maxlen=max_length)

# Set fixed y-axis limits
axs[0].set_ylim(0, 300)
axs[1].set_ylim(0, 1)
axs[2].set_ylim(0, 360)

# Disable interactive mode
plt.ioff()

# Listen for incoming datagrams
update_interval = 0.1  # Update the plot every 0.1 seconds
last_update_time = time.time()

while True:
    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    message = bytesAddressPair[0]
    values = message.decode('utf-8').split(',')

    timestamp, tofY_1, tofY_2, tofY_3, tofY_4, gyroZ, motorA, motorB = (float(value) for value in values)

    timestampArray.append(timestamp)
    motorArrayA.append(motorA)
    motorArrayB.append(motorB)
    gyroZarray.append(gyroZ)

    # Redraw the plot at fixed intervals
    current_time = time.time()
    if current_time - last_update_time >= update_interval:
        for ax in axs:
            ax.clear()
            ax.set_facecolor('black')

        axs[0].set_title('Motor A and B')
        axs[1].set_title('TOF 1, 2, 3, 4')
        axs[2].set_title('Gyro, Z axis')

        axs[0].plot(timestampArray, motorArrayA, color='g', label='Motor A')
        axs[0].plot(timestampArray, motorArrayB, color='r', label='Motor B')
        axs[0].legend(loc='upper right')

        axs[1].bar(['tof1', 'tof2', 'tof3', 'tof4'], [tofY_1, tofY_2, tofY_3, tofY_4], color=['r', 'g', 'b', 'c'], alpha=0.5)

        axs[2].plot(timestampArray, gyroZarray, color='b', label='Gyro Z axis')
        axs[2].legend(loc='upper right')

        plt.draw()
        plt.pause(0.01)
        last_update_time = current_time

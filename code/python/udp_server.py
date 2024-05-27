import socket
import matplotlib.pyplot as plt
import numpy as np
import time
 

localIP     = "192.168.137.1"

localPort   = 3333

bufferSize  = 1024

 

msgFromServer       = "Hello UDP Client"

bytesToSend         = str.encode(msgFromServer)

 

# Create a datagram socket

UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

 

# Bind to address and ip

UDPServerSocket.bind((localIP, localPort))

 

print("UDP server up and listening")

fig, axs = plt.subplots(3)
plt.subplots_adjust(hspace=0.5)  # Add space between subplots
plt.figure(facecolor='black')
timestampArray = []
motorArrayA = []
motorArrayB = []
gyroZarray = []



# Listen for incoming datagrams
startTime = time.time()
while(True):
    #if(time.time() - startTime > 0.016):
    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)

    # ocakavana forma -- "b'timestamp, tofX_1, tofY_1, tofX_2, tofY_2, tofX_3, tofY_3, gyroZ, motorA, motorB'"
    message = bytesAddressPair[0]

    # address = bytesAddressPair[1]

    # clientMsg = "{}".format(message) 
    # # clientIP  = "Client IP Address:{}".format(address)
    
    # print(clientMsg)
    # print(clientIP)
    values = message.decode('utf-8').split(',')

    # Extract the values from the list
    timestamp, tofY_1, tofY_2, tofY_3, tofY_4, gyroZ, motorA, motorB = (float(value) for value in values)
    # map(float, values)

    # print("timestamp: ", timestamp, " tofY_1: ", tofY_1, " tofY_2: ", tofY_2, " tofY_3: ", tofY_3, " tofY_4: ", tofY_4, " gyroZ: ", gyroZ, " motorA: ", motorA, " motorB: ", motorB)
    # # for _ in range(100):
    timestampArray.append(timestamp)
    motorArrayA.append(motorA)
    motorArrayB.append(motorB)
    gyroZarray.append(gyroZ)
    # tofArray1 = tofY_1
    # tofArray2 = tofY_2
    # tofArray3 = tofY_3
    # tofArray4 = tofY_4
    
    # Limit the size of the arrays to the last 500 elements
    if len(timestampArray) > 100:
        timestampArray = timestampArray[-100:]
    if len(motorArrayA) > 100:
        motorArrayA = motorArrayA[-100:]
    if len(motorArrayB) > 100:
        motorArrayB = motorArrayB[-100:]
    if len(gyroZarray) > 100:
        gyroZarray = gyroZarray[-100:]
    
    
    axs[0].clear()
    axs[1].clear()
    axs[2].clear()
    
    axs[0].set_facecolor('black')
    axs[1].set_facecolor('black')
    axs[2].set_facecolor('black')
    axs[0].set_title('Motor A and B')
    axs[1].set_title('TOF 1, 2, 3, 4')
    axs[2].set_title('Gyro, Z axis')
    axs[0].legend(loc='upper right')  # Add legend to the plot

    axs[2].legend(loc='upper right')
    axs[0].plot(timestampArray, motorArrayA, color='g', label='Motor A')  # Add label for Motor A
    axs[0].plot(timestampArray, motorArrayB, color='r', label='Motor B')  # Add label for Motor B
    axs[1].bar(['tof1', 'tof2', 'tof3', 'tof4'], [tofY_1, tofY_2, tofY_3, tofY_4], color=['r', 'g', 'b', 'c'], alpha=0.5)
    axs[2].plot(timestampArray, gyroZarray, color='b', label='Gyro Z axis')  # Add label for Gyro Z axis
    # plt.pause(0.025)
    
    # plt.show()
    plt.draw()
    plt.pause(0.01)
    # Sending a reply to client

    # UDPServerSocket.sendto(bytesToSend, address)
    #startTime = time.time()
import socket
import matplotlib.pyplot as plt
import numpy as np
import time
 

localIP     = "0.0.0.0"

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
timestampArray = []
motorArrayA = []
motorArrayB = []
tofArray1 = []
tofArray2 = []
tofArray3 = []
gyroZarray = []


# Listen for incoming datagrams

while(True):

    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)

    # ocakavana forma -- "b'timestamp, tofX_1, tofY_1, tofX_2, tofY_2, tofX_3, tofY_3, gyroZ, motorA, motorB'"
    message = bytesAddressPair[0]

    # address = bytesAddressPair[1]

    # clientMsg = "Message from Client:{}".format(message) 
    # clientIP  = "Client IP Address:{}".format(address)
    
    # print(clientMsg)
    # print(clientIP)
    values = message.split(',')

    # Extract the values from the list
    timestamp, tofX_1, tofY_1, tofX_2, tofY_2, tofX_3, tofY_3, gyroZ, motorA, motorB = map(float, values)


    # for _ in range(100):
    timestampArray.append(timestamp)
    motorArrayA.append(motorA)
    motorArrayB.append(motorB)
    gyroZarray.append(gyroZ)
    tofArray1 = tofX_1
    tofArray2 = tofX_2
    tofArray3 = tofX_3
    axs[0].clear()
    axs[1].clear()
    axs[2].clear()
    axs[0].set_title('Motor A and B')
    axs[1].set_title('TOF 1, 2, 3')
    axs[2].set_title('Gyro, Z axis')
    axs[0].plot(timestampArray, motorArrayA, color='g', label='Motor A')  # Add label for Motor A
    axs[0].plot(timestampArray, motorArrayB, color='r', label='Motor B')  # Add label for Motor B
    axs[0].legend(loc='upper right')  # Add legend to the plot
    axs[1].bar(['tof1', 'tof2', 'tof3'], [tofArray1, tofArray2, tofArray3], color=['r', 'g', 'b'], alpha=0.5)
    axs[2].plot(timestampArray, gyroZarray, color='b', label='Gyro Z axis')  # Add label for Gyro Z axis
    axs[2].legend(loc='upper right')
    # plt.pause(0.025)
    
    plt.show()
    # Sending a reply to client

    # UDPServerSocket.sendto(bytesToSend, address)
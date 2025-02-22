import numpy as np
import socket
import struct
import math
import cv2

# Create the window
cv2.namedWindow('Visualization', cv2.WINDOW_NORMAL)

def DisplayWaitingFrame():
    frame = np.zeros((60, 420, 3), np.uint8)
    cv2.resizeWindow('Visualization', frame.shape[1], frame.shape[0])
    cv2.putText(frame, "Waiting for connection...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.imshow('Visualization', frame)
    cv2.waitKey(1)

DisplayWaitingFrame()

HOST = '169.254.12.184' # EV3 IP address
PORT = 65432 # Port to connect to

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    server_socket.bind((HOST, PORT))
except:
    print(f"There was an error creating the socket at {HOST}:{PORT}. This could be caused by many thinga:\n1. Invalid IP address\n2. EV3 is not running or not connected via tethering\n3. Port already in use")
    input("Press any key to exit...")
    cv2.destroyAllWindows()
    exit()

server_socket.listen(1)
print(f"Listening for connections on {HOST}:{PORT}...")

conn, addr = server_socket.accept()
print(f"Accepted connection from {addr[0]}:{addr[1]}")

def receive_initial_data(chunk_size=1024):
    data = b''
    initial_data = None
    while initial_data is None:
        chunk = conn.recv(chunk_size)
        if not chunk:
            break
        data += chunk
        try:
            initial_data = eval(data)
        except:
            initial_data = None

    if initial_data:
        PATH = initial_data[0]
        LOOKAHEAD_DIS = initial_data[1]
        CURVATURE_POINTS = initial_data[2]
    else:
        print("Client disconnected")
        cv2.destroyAllWindows()
        server_socket.close()
        exit()
    
    return PATH, LOOKAHEAD_DIS, CURVATURE_POINTS

PATH, LOOKAHEAD_DIS, CURVATURE_POINTS = receive_initial_data()
window_width = 0
window_height = 0
padding_x = 0
padding_y = 0
base_frame = None
def SetupVisualization():
    global window_width, window_height, padding_x, padding_y, base_frame, PATH
    largest_x = max([PATH[i][0] for i in range(len(PATH))])
    largest_y = max([PATH[i][1] for i in range(len(PATH))])

    window_width = int(largest_x * 1.2)
    window_height = int(largest_y * 1.2)
    padding_x = int(largest_x * 0.1)
    padding_y = int(largest_y * 0.1)

    # Create the base frame
    base_frame = np.zeros((window_height, window_width, 3), np.uint8)
    for i in range(len(PATH)):
        if i > 0:
            cv2.line(base_frame, (int(PATH[i-1][0] + padding_x), int(window_height - PATH[i-1][1] - padding_y)), 
                    (int(PATH[i][0] + padding_x), int(window_height - PATH[i][1] - padding_y)), (0, 255, 0), 2)
        cv2.circle(base_frame, (int(PATH[i][0] + padding_x), int(window_height - PATH[i][1] - padding_y)), 4, (0, 0, 255), -1)
        
    # Resize the window
    cv2.resizeWindow('Visualization', window_width, window_height)

def UpdateVisualization(x, y, heading, targetSpeed, curvature, goalIndex) -> None:
    global base_frame, padding_x, padding_y, window_width, window_height, PATH, LOOKAHEAD_DIS, CURVATURE_POINTS
    frame = base_frame.copy()
    cv2.circle(frame, (int(x + padding_x), int(window_height - y - padding_y)), 10, (255, 0, 0), -1)
    
    # Calculate the end point of the line
    end_x = int(x + LOOKAHEAD_DIS * math.cos(math.radians(heading)) + padding_x)
    end_y = int(window_height - y - LOOKAHEAD_DIS * math.sin(math.radians(heading)) - padding_y)
    cv2.line(frame, (int(x + padding_x), int(window_height - y - padding_y)), 
             (end_x, end_y), (0, 165, 255), 2)
    
    # Draw lines with orange if they are in the path where curvature is calculated
    if len(PATH[goalIndex:]) < CURVATURE_POINTS:
        curve_path = PATH[goalIndex - 1:]
    else:
        curve_path = PATH[goalIndex - 1:goalIndex + CURVATURE_POINTS]
    for i in range(len(curve_path)):
        if i > 0:
            cv2.line(frame, (int(curve_path[i-1][0] + padding_x), int(window_height - curve_path[i-1][1] - padding_y)), 
                     (int(curve_path[i][0] + padding_x), int(window_height - curve_path[i][1] - padding_y)), (0, 165, 255), 2)

    cv2.putText(frame, f"Speed: {round(targetSpeed, 2)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(frame, f"Curvature: {round(curvature, 2)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    cv2.imshow('Visualization', frame)
    cv2.waitKey(1)

# Let the EV3 know that the visualization is ready
SetupVisualization()
conn.sendall("ok".encode())

print("Path received, starting visualization...")
paused = False
while True:
    data = conn.recv(512)
    if not data:
        break

    if paused:
        continue

    if data == b"stop":
        print("Paused, waiting for start command...")
        paused = True
        DisplayWaitingFrame()
    
    if data == b'restart':
        print("Restarting visualization...")
        paused = False
        PATH, LOOKAHEAD_DIS, CURVATURE_POINTS = receive_initial_data()
        SetupVisualization(PATH)
        conn.sendall("ok".encode())
        print("Path received, starting visualization...")

    # Process the received data
    data_length = len(data)
    num_floats = data_length // 4  # Each float is 4 bytes
    sets_of_floats = num_floats // 6  # Each set contains 6 floats

    # Ensure we have complete sets of 24 bytes
    if sets_of_floats > 0:
        for i in range(sets_of_floats):
            # Extract the relevant slice for unpacking
            start_index = i * 24
            end_index = start_index + 24
            float_set = data[start_index:end_index]
            
            try:
                # Unpack the data
                xpos, ypos, heading, targetSpeed, curvature, goalIndex = struct.unpack('6f', float_set)
                goalIndex = int(goalIndex)

                # Update the visualization with the new position and heading
                UpdateVisualization(xpos, ypos, heading, targetSpeed, curvature, goalIndex)
            except struct.error as e:
                print(f"Failed to unpack data: {e}")
                continue
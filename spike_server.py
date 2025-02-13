from bleak import BleakScanner, BleakClient
import numpy as np
import traceback
import threading
import asyncio
import struct
import math
import time 
import csv
import cv2
import os

console_banner_text = """
======================================================================
            ____        _____       __
        / __ \__  __/ ___/____  / /___ _____  ____  ___  _____
        / /_/ / / / /\__ \/ __ \/ / __ `/ __ \/ __ \/ _ \/ ___/
        / ____/ /_/ /___/ / /_/ / / /_/ / / / / / / /  __/ /
        /_/    \__, //____/ .___/_/\__,_/_/ /_/_/ /_/\___/_/
            /____/     /_/

===================== Spike Visualization Server =====================
"""
print(console_banner_text)
print("Loading...")

PYBRICKS_COMMAND_EVENT_CHAR_UUID = "c5f50002-8280-46da-89f4-6d8051e4aeef"
HUB_NAME = "TC"

terminate_thread = False

initial_data = None
data = b''
setup = False

PATH = None
LOOKAHEAD_DIS = None
CURVATURE_POINTS = None

base_frame = None
padding_x = None
padding_y = None
window_width = None
window_height = None

def SetupVisualization():
    global base_frame, padding_x, padding_y, window_width, window_height

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
            cv2.line(base_frame, (int(PATH[i - 1][0] + padding_x), int(window_height - PATH[i - 1][1] - padding_y)),
                     (int(PATH[i][0] + padding_x), int(window_height - PATH[i][1] - padding_y)), (0, 255, 0), 2)
        cv2.circle(base_frame, (int(PATH[i][0] + padding_x), int(window_height - PATH[i][1] - padding_y)), 4, (0, 0, 255), -1)

    # Resize the window
    cv2.namedWindow('Visualization', cv2.WINDOW_NORMAL)
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
            cv2.line(frame, (int(curve_path[i - 1][0] + padding_x), int(window_height - curve_path[i - 1][1] - padding_y)),
                     (int(curve_path[i][0] + padding_x), int(window_height - curve_path[i][1] - padding_y)), (0, 165, 255), 2)

    cv2.putText(frame, f"Speed: {round(targetSpeed, 2)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(frame, f"Curvature: {round(curvature, 2)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    cv2.imshow('Visualization', frame)
    cv2.waitKey(1)

async def main():
    global terminate_thread, setup, initial_data, PATH, LOOKAHEAD_DIS, CURVATURE_POINTS, data
    main_task = asyncio.current_task()
    datalog_data = []

    def handle_disconnect(_):
        print("Hub was disconnected.")
  
        terminate_thread = True
        if not main_task.done():
            main_task.cancel()

    def add_chunk_to_data(chunk : str):
        global initial_data, data, PATH, LOOKAHEAD_DIS, CURVATURE_POINTS
        data += chunk
        try:
            initial_data = eval(data)
            PATH = initial_data[0]
            LOOKAHEAD_DIS = initial_data[1]
            CURVATURE_POINTS = initial_data[2]
            print(f"Received initial data: PATH = {PATH}, LOOKAHEAD_DIS = {LOOKAHEAD_DIS}, CURVATURE_POINTS = {CURVATURE_POINTS}")
            return True
        except:
            initial_data = None
            PATH = None
            LOOKAHEAD_DIS = None
            CURVATURE_POINTS = None
            return False

    async def handle_rx(_, rx_data: bytearray):
        global setup, initial_data
        try:
            if rx_data[0] == 0x01: # "write stdout" event (0x01)
                payload_encoded = rx_data[1:] # Remove the event type

                if initial_data is None:
                    payload_decoded = payload_encoded.decode()  # Decode the remaining bytes
                    done = add_chunk_to_data(payload_encoded)

                    if not setup and done:
                        print("Received all initial data, setting up visualization...")
                        SetupVisualization()
                        setup = True

                        await client.write_gatt_char(
                            PYBRICKS_COMMAND_EVENT_CHAR_UUID,
                            b"\x06ok",  # prepend "write stdin" command (0x06) and "ok" response
                            response=True
                        )

                        print("Acknowledged initial data, starting visualization...")
                    else:
                        pass
                else:
                    decoded_data = struct.unpack('3h2fb', payload_encoded[:-2])
                    print(f"Received data chunk: {decoded_data}")
                    UpdateVisualization(*decoded_data)
        except struct.error:
            print(f"Bad data: {payload_encoded}")
            pass
        except Exception as e:
            print(f"Error in handle_rx: \n{traceback.format_exc()}")
            terminate_thread = True

    device = await BleakScanner.find_device_by_name(HUB_NAME)

    if device is None:
        print(f"Could not find hub with name: {HUB_NAME}")
        terminate_thread = True
        return
    else:
        print(f"Connected to hub with name: {HUB_NAME}")

    async with BleakClient(device, handle_disconnect) as client:
        await client.start_notify(PYBRICKS_COMMAND_EVENT_CHAR_UUID, handle_rx)

        print("Press the center button on your hub to begin.")

        try:
            while not terminate_thread:
                await asyncio.sleep(2)
        except asyncio.CancelledError:
            print("Program cancelled, disconnecting.")
            terminate_thread = True

def run_event_loop(loop):
    asyncio.set_event_loop(loop)
    loop.run_until_complete(main())

def start_async_thread():
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=run_event_loop, args=(loop,), daemon=True)
    thread.start()
    return thread, loop

if __name__ == "__main__":
    event_thread, event_loop = start_async_thread()
    try:
        print("Main thread is running. Press Ctrl+C to exit...")
        while not terminate_thread:
            time.sleep(2)
    except KeyboardInterrupt:
        print("Terminating program (Ctrl+C detected)...")
        terminate_thread = True
    finally:
        event_loop.stop()
        event_thread.join()
        print("Event loop stopped. Program terminated.")
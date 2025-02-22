#!/usr/bin/env pybricks-micropython

# PySplanner - A Pure Pursuit Robot Controller
# Licensed under the GNU General Public License v3.0
# © 2024 DylDev (https://github.com/DylDevs)
# © 2024 techcatgato (https://github.com/techcatgato)
# © 2024 SkroutzTBY (https://github.com/SkroutzTBY)

'''
Main PySplanner program, includes the run selector and the main run loop.

EV3:
    - Set _HUB_TYPE to "EV3" (This will be automated soon)
    - Run the program on the EV3 brick
    - If you want to use visualization, run the EV3 visualization server

SPIKE:
    - Set _HUB_TYPE to "Spike" (This will be automated soon)
    - Run the program on the SPIKE hub
    - If you want to use visualization, run the SPIKE visualization server

- Visualization:
    - Set _VISUALIZE to True
    - EV3 First Time Setup:
        - Plug in the EV3 hub to the PC via USB
        - On the EV3 go to Wireless and Networks > All Network Connections > Wired and click "Connect"
            - You should also check the box that says Connect Automatically
        - Wait for it to connect, once its connected you should see the IP address of the EV3 in the top left corner
            - Don't use this IP address, it will change every time and is incorrect
        - On the PC which the EV3 is connected to, open a terminal and run the following command:
            - ipconfig
        - Look for the IPv4 Address of the EV3 and copy it (it should be a 169.254.x.x address)
        - In both this file and the EV3 server file, replace _VISUALIZE_IP with the IPv4 address of the EV3
        - Ensure the port number is the same in both files
        - You will need to repeat this on every new EV3 or computer

    - Run the appropriate visualization server (EV3 or SPIKE)
    - On SPIKE, wait until it says to press the center button and then press it to run the program
    - On EV3, wait until it says waiting for connection and then start the EV3 program

Other Notes:
    - Most variables are underlined constants, this is for optimization. They do not affect how the program runs.
    - Some variables are micropython constants, these are for optimization. They do not affect how the program runs.
    - The EV3 visualization server is a simple TCP server that receives data from the EV3 and displays it in a window.
    - The SPIKE visualization server is a BLE server that receives data from the SPIKE and displays it in a window.
'''
_HUB_TYPE = "Spike" # "Spike", "EV3"

_MONITOR_FPS = True
_VISUALIZE = False
_VISUALIZE_IP = "169.254.12.184"
_VISUALIZE_PORT = 65432
_SEND_EVERY_X_FRAMES = 3

console_banner_text = """
            ____        _____       __
        / __ \__  __/ ___/____  / /___ _____  ____  ___  _____
        / /_/ / / / /\__ \/ __ \/ / __ `/ __ \/ __ \/ _ \/ ___/
        / ____/ /_/ /___/ / /_/ / / /_/ / / / / / / /  __/ /
        /_/    \__, //____/ .___/_/\__,_/_/ /_/_/ /_/\___/_/
            /____/     /_/   
"""
top_line =    "======================================================================" if _HUB_TYPE == "EV3" else "======================================================================="
buttom_line = "======================= PySplanner EV3 Version =======================" if _HUB_TYPE == "EV3" else "====================== PySplanner SPIKE Version ======================="

print(top_line)
print(console_banner_text)
print(buttom_line)

ev3_banner_text = """\n
                        ____
                    / __ \__  __
                    / /_/ / / / /
                    / ____/ /_/ /
                    /_/    \__, /
                        /____/
    _____       __
    / ___/____  / /___ _____  ____  ___  _____
    \__ \/ __ \/ / __ `/ __ \/ __ \/ _ \/ ___/
    ___/ / /_/ / / /_/ / / / / / / /  __/ /
    /____/ .___/_/\__,_/_/ /_/_/ /_/\___/_/
        /_/
"""

# Parameters for setting up the robot
from pybricks.parameters import Button, Color, Port

if _HUB_TYPE == "Spike":
    from pybricks.hubs import PrimeHub # Spike hub class

    hub = PrimeHub() # Initialize hub
    hub.system.set_stop_button(Button.BLUETOOTH) # Make sure the center button wont quit the program
    display = hub.display # Initialize display
    display.off() # Turn off display

    # Spike specific imports
    from pybricks.parameters import Button, Color, Port, Axis, Direction
    from pybricks.tools import multitask as thread_handler
    from pybricks.pupdevices import Motor
    from pybricks.tools import StopWatch
    import urandom as random
    import ustruct as struct
    from usys import stdout
    import umath as math
else:
    from pybricks.hubs import EV3Brick # EV3 hub class

    hub = EV3Brick() # Initialize hub
    display = hub.screen # Initialize display
    display.clear() # Clear display

    # EV3 specific imports
    from threading import Thread as thread_handler
    from pybricks.media.ev3dev import Font
    from pybricks.ev3devices import Motor
    import random
    import struct
    import socket
    import math
    import time

    # Set font for displaying the banner
    BANNER_FONT = Font('Lucida', 8, bold=False)
    display.set_font(BANNER_FONT)

    display.print(ev3_banner_text) # Draw banner text

    # Set font for displaying run numbers and other text
    FONT = Font('Lucida', 32, bold=True)
    display.set_font(FONT)

print("Loading...")

# Non-specific imports
from micropython import const, opt_level
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Micropython optimization
opt_level(3)

# Variables needed for classes and functions
_WHEEL_DIAMETER = const(42) if _HUB_TYPE == "EV3" else 62.4 # Wheel diameter in mm
_AXLE_TRACK = const(93) if _HUB_TYPE == "EV3" else const(90)  # Distance between wheels in mm
_WHEEL_RADIUS = _WHEEL_DIAMETER / 2  # Calculate the wheel radius
_WHEEL_CIRCUMFERENCE = math.pi * _WHEEL_DIAMETER  # Calculate the wheel circumference
if _HUB_TYPE != "Spike":
    _LEFT_MOTOR = Motor(Port.A)
    _RIGHT_MOTOR = Motor(Port.D)
else:
    _LEFT_MOTOR = Motor(Port.A, Direction.COUNTERCLOCKWISE)
    _RIGHT_MOTOR = Motor(Port.E, Direction.CLOCKWISE)
_ROBOT = DriveBase(_LEFT_MOTOR, _RIGHT_MOTOR, _WHEEL_DIAMETER, _AXLE_TRACK)

quit_program = False

class ThreadedFunction:
    class MissingArgumentVariableError(Exception):
        pass

    def __init__(self, function: callable, args_variable_name: str, args: tuple = ()):
        if not callable(function):
            raise TypeError("ThreadedFunction received non-callable object: " + str(function))
        if not isinstance(args, tuple):
            args = (args,) # Only one argument (Python breaks if you pass a tuple of length 1)
        if not isinstance(args_variable_name, str):
            raise TypeError("args_variable_name must be a string, got " + type(args_variable_name))

        self.function = function
        self.arg_variable = args_variable_name

        if self.arg_variable not in globals():
            raise self.MissingArgumentVariableError("Function argument variable not found. Define it as follows:\n" + self.arg_variable + " = ()")
        else:
            globals()[self.arg_variable] = args

def RunInThread(*funcs: ThreadedFunction):
    '''
    Runs function(s) in separate thread(s)

    Parameters:
        funcs (ThreadedFunction): Function(s) to run in separate thread(s)

    Returns:
        None

    Example:
        RunInThread(ThreadedFunction(func, arg_variable_name (arg1, arg2)), ThreadedFunction(other_func, arg_variable_name (arg3, arg4)))
    '''
    funcs_to_run = []

    if isinstance(funcs, ThreadedFunction):
        funcs_to_run.append(funcs)
    elif isinstance(funcs, tuple) and all(isinstance(func, ThreadedFunction) for func in funcs):
        for func in funcs:
            if type(func) != ThreadedFunction:
                raise TypeError("Received non-ThreadedFunction object in RunInThread: " + str(func))
            funcs_to_run.append(func.function)
    else:
        raise TypeError("Received non-ThreadedFunction (or tuple) object in RunInThread: " + str(funcs))

    if _HUB_TYPE == "Spike":
        thread_handler(*funcs_to_run) if type(funcs_to_run) == tuple else thread_handler(funcs_to_run)
    else:
        for func in funcs_to_run:
            thread_handler(target=func).start()

# Functions and classes
_blink_thread_args = ()
class Light:
    '''
    Handles lighting on EV3 and SPIKE

    Methods:
        blink(color, durations)
        on(color)
        off()

    Example:
        light = Light()
        light.blink(Color.ORANGE, [0.5, 0.5])
        light.on(Color.GREEN)
        light.off()
    '''

    def __init__(self):
        self.light = hub.light
        self.blinking = False
        self.stop_blinking = False
        pass

    def blink(self, color: Color, durations: tuple[float, float]):
        '''
        Blinks the button light with a specified color and durations.
        If the light does not start blinking, the color is not supported.

        Parameters:
            color (Color): Color to blink
            durations (tuple[float, float]): Duration of on and off phases in seconds

        Returns:
            None

        Example:
            light.blink(Color.ORANGE, [0.5, 0.5])
        '''
        # Stop the current blink thread
        self.end_blink()
        
        # Set up the new blink thread
        RunInThread(ThreadedFunction(self._blink_thread, "_blink_thread_args", (color, durations[0], durations[1])))

    def end_blink(self):
        if not self.blinking: return
        self.stop_blinking = True
        while self.blinking:
            pass
        self.stop_blinking = False
        self.light.off()

    def _blink_thread(self):
        color, duration_on, duration_off = _blink_thread_args
        self.blinking = True

        on_dur = duration_on * 1000
        off_dur = duration_off * 1000
        while not self.stop_blinking:
            self.light.on(color)
            wait(on_dur)
            self.light.off()
            wait(off_dur)

        self.blinking = False
        
    def on(self, color: Color):
        '''
        Turns the button light on with a specified color.
        If the light does not turn on, the color is not supported.

        Parameters:
            color (Color): Color to turn the light on

        Returns:
            None

        Example:
            light.on(Color.GREEN)
        '''
        # End the previous blink
        self.end_blink()

        self.light.on(color)
    
    def off(self):
        '''
        Turns the button light off.

        Parameters:
            None

        Returns:
            None

        Example:
            light.off()
        '''
        # End the previous blink
        self.end_blink()

        self.light.off()

# Light during loading
light = Light()
light.blink(Color.ORANGE, [0.2, 0.5])

class VisualizationServer:
    def __init__(self):
        self.previously_paused = False
        if _HUB_TYPE == "Spike":
            self.server = stdout
        else:
            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server.connect((_VISUALIZE_IP, _VISUALIZE_PORT))

    def _send_bytes(self, data : bytes):
        if _HUB_TYPE == "Spike":
            self.server.write(data)
        else:
            self.server.sendall(data)
    
    def _read_bytes(self, chunk_size : int = 512) -> bytes:
        if _HUB_TYPE == "Spike":
            return self.server.buffer.read(chunk_size)
        else:
            return self.server.recv(chunk_size)

    def send_initial_data(self, data : list[list[list[float, float]], int, int]):
        chunk_size = 20
        serialized_data = str(data).encode() if _HUB_TYPE == "EV3" else bytearray(str(data), 'utf-8')
        total_sent = 0
        while total_sent < len(serialized_data):
            chunk = serialized_data[total_sent:total_sent + chunk_size]
            self._send_bytes(b'#' + chunk) if _HUB_TYPE == "Spike" else self._send_bytes(chunk)
            total_sent += len(chunk)
        
        while self._read_bytes(2) != b"ok":
            pass

    def send_data(self, x : float, y : float, heading : float, targetSpeed : float, curvature : float, goalIndex : int):
        self._send_bytes(struct.pack('6f', x, y, heading, targetSpeed, curvature, float(goalIndex)))

    def start(self):
        if self.previously_paused:
            self._send_bytes(b"restart")

    def stop(self):
        self.previously_paused = True
        self._send_bytes(b"stop")

def RobotAngle():
    '''
    Returns the angle of the robot in degrees.

    Parameters:
        None

    Returns:
        float | int: Angle of the robot in degrees
        
    Example:
        angle = RobotAngle()
    '''

    if _HUB_TYPE == "Spike":
        return hub.imu.heading() # Internal SPIKE IMU (pybricks.hubs.PrimeHub.imu)
    else:
        return _ROBOT.angle() # Calculated angle from robot movement (pybricks.robotics.DriveBase as _ROBOT)

def WheelDistance(side = "left"):
    '''
    Gets the distance travelled of the wheel on the specified side
    
    Parameters:
        side (str["left" | "right"]): Side of the wheel
        
    Returns:
        float | int: Distance travelled by the wheel in mm

    Exanple:
        distance = WheelDistance("left")
        distance = WheelDistance("right)
    '''

    return ((_LEFT_MOTOR.angle() if side == "left" else _RIGHT_MOTOR.angle()) / 360) * _WHEEL_CIRCUMFERENCE

last_displayed_number = None
def DisplayNumber(number : int):
    '''
    Displays an integer on the screen or light matrix.

    Parameters:
        number (int): Number to display

    Returns:
        None

    Example:
        DisplayNumber(10)
    '''

    global last_displayed_number
    if number != last_displayed_number:
        if _HUB_TYPE == "Spike":
            display.number(number) # Spike has a built-in number display (pybricks.hubs.PrimeHub.display)
        else:
            display.clear()
            text_w, text_h = FONT.text_width(str(number)), FONT.text_height(str(number))
            display.draw_text(178 / 2 - text_w / 2, 128 / 2 - text_h / 2, str(number)) # Manually draw number on the screen (pybricks.hubs.EV3Brick.screen as display)
        last_displayed_number = number

class TableRun:
    '''
    Class to store information about a table run.
    
    Attributes:
        name (str): Name of the run
        description (str | None): Description of the run
        table_side (str): Side of the table the run is on
        path (list[list[int, float]]): Path of the run in 2D coordinate lists (e.g. [[x1, y1], [x2, y2]])
        pure_pursuit_params (dict[str, int | float | str]): Dictionary of pure pursuit parameters, see below for more info

    Pure Pursuit Parameters:
        start_speed_factor (int | float): Factor to control initial speed
        finished_distance (int | float): Distance to consider the robot finished in mm
        curvature_points (int): Number of points to use for curvature calculation
        turn_vel_limit (int): Maximum turn velocity limit
        curvature_threshold (int | float): Threshold for taking action based on curvature
        points_to_search (int): Number of points to check in pure pursuit
        lookahead_points (int ): Number of points to check in pure pursuit
        slowdown_dis (int | float): Distance to start slowing down in mm
        curvature_factor (int | float): Factor to influence speed based on curvature
        max_speed (int): Maximum speed in mm/s
        kp_turn (int | float): Proportional gain for turning

    Methods:
        None
    '''

    REQUIRED_PARAMS = {
        # "param_name": [type, type2, ..., min, max]
        'start_speed_factor': [int, float, 1, 100],     # Factor to control initial speed
        'finished_distance': [int, float, 0.1, 10],     # Distance to consider the robot finished in mm
        'curvature_points': [int, 1, 10],               # Number of points to use for curvature calculation
        'turn_vel_limit': [int, 1, 360],                # Maximum turn velocity limit
        'curvature_threshold': [int, float, 0, 2],      # Threshold for taking action based on curvature
        'points_to_search': [int, 1, 50],               # Number of points to check in pure pursuit
        'lookahead_points': [int, 1, 10],               # Amount of points to look ahead in the path 
        'slowdown_dis': [int, float, 0, float("inf")],  # Distance to start slowing down in mm
        'curvature_factor': [int, float, 0.1, 10],      # Factor to influence speed based on curvature
        'max_speed': [int, 25, 300],                    # Maximum speed in mm/s
        'kp_turn': [int, float, 0.1, 10]                # Proportional gain for turning
    }

    def __init__(self, name : str, description : str, table_side : str, path : list[list[int, float]], pure_pursuit_params : dict[str, int | float | str]):
        # Type checks for other arguments
        if not isinstance(name, str):
            raise TypeError("'name' must be a string, got " + type(name).__name__)
        if description is not None and not isinstance(description, str):
            raise TypeError("'description' must be a string or None, got " + type(description).__name__)
        if table_side not in ["left", "right"]:
            raise ValueError("'table_side' must be either 'left' or 'right', got '" + table_side + "'")
        if not (isinstance(path, list) and all(isinstance(p, list) and len(p) == 2 and all(isinstance(coord, (int, float)) for coord in p) for p in path)):
            raise TypeError("'path' must be a list of 2D coordinate lists (e.g. [[x1, y1], [x2, y2]])")

        self.name = name
        self.description = description
        self.table_side = table_side
        self.path = path
        self.pure_pursuit_params = pure_pursuit_params

        # Ensure all required pure_pursuit_params are present and have the correct types
        missing_params = set(self.REQUIRED_PARAMS.keys()) - set(pure_pursuit_params.keys())
        if missing_params:
            raise ValueError("Missing required pure pursuit parameters: " + ', '.join(missing_params))

        for param, param_data in self.REQUIRED_PARAMS.items():
            expected_types = param_data[:-2] 
            min_value = param_data[-2]
            max_value = param_data[-1]

            found_match = False
            for accepted_type in expected_types:
                if isinstance(pure_pursuit_params.get(param), accepted_type):
                    found_match = True
                    break

            if not found_match:
                raise TypeError("Parameter '" + param + "' must be one of these types: " + str([t.__name__ for t in expected_types]) + ", got " + type(pure_pursuit_params.get(param)).__name__)

            if min_value is not None and max_value is not None:
                if not min_value <= pure_pursuit_params.get(param) <= max_value:
                    raise ValueError("Parameter '" + param + "' must be between " + str(min_value) + " and " + str(max_value) + ", got " + str(pure_pursuit_params.get(param)))

_run_program_args = ()

class CustomRunSelector:
    '''
    Class to select a table run. Handles the button input and runs the program.

    Attributes:
        runs (list[TableRun]): List of table runs
        run_index (int): Index of the current run
        last_left_pressed (bool): Whether the left button was pressed last frame
        last_right_pressed (bool): Whether the right button was pressed last frame
        last_center_pressed (bool): Whether the center button was pressed last frame
        running_program (bool): Whether the program is running

    Methods:
        None

    Example:
        selector = CustomRunSelector(runs)
    '''

    def __init__(self, runs : list[TableRun], visualization : VisualizationServer | None = None):
        self.runs = runs
        self.visualization = visualization
        self.run_index = 0
        self.last_left_pressed = False
        self.last_right_pressed = False
        self.last_center_pressed = False
        self.running_program = False
        self.exited = False
        self.quit_program = False

    def get_status(self):
        return self.exited == False
    
    def _run_program(self, run : TableRun | None = None) -> None:
        '''
        Runs a table run.
        This function is optimized to be ran with the PySplanner thread runner.

        Parameters:
            run (TableRun): Table run to run

        Returns:
            None

        Example:
            RunProgram(run)
        '''
        run = _run_program_args[0] if run is None else run

        print("Running: " + run.name)
        controller = DriveController(run)

        if self.visualization is not None:
            path = controller.path
            lookahead_dis = 30
            curvature_points = controller.curvature_points

            self.visualization.start()
            self.visualization.send_initial_data([path, lookahead_dis, curvature_points])

        frames = 0
        if _MONITOR_FPS:
            sw = StopWatch() if _HUB_TYPE == "Spike" else time.time()

        while not controller.status and not quit_program:
            controller.UpdateCoordinatesSpike() if _HUB_TYPE == "Spike" else controller.UpdateCoordinatesEV3()
            controller.RunPurePursuit()
            _ROBOT.drive(controller.target_speed, controller.turn_rate)

            if self.visualization is not None and frames % _SEND_EVERY_X_FRAMES == 0:
                self.visualization.send_data(controller.xpos, controller.ypos, controller.heading, controller.target_speed, controller.curvature, controller.goal_index)
            
            frames += 1

        _ROBOT.stop()        

        print("Finished running: " + run.name)

        if _MONITOR_FPS:
            end_time = sw.time() if _HUB_TYPE == "Spike" else time.time()
            total_time = end_time if _HUB_TYPE == "Spike" else end_time - sw
            
            print("Total time: " + str(round(total_time, 2)) + "s")
            print("FPS: " + str(round(frames / total_time, 2)))

        if self.visualization is not None:
            self.visualization.stop()

        del controller
        self.quit_program = True
    
    def _run_controller(self, run : TableRun):
        self.quit_program = False
        RunInThread(ThreadedFunction(self._run_program, "_run_program_args", (run)))
        while not self.quit_program:
            pressed = hub.buttons.pressed()
            center_pressed = Button.CENTER in pressed
            if center_pressed and not self.last_center_pressed:
                self.running_program = False
                self.last_center_pressed = True
                self.quit_program = True
            wait(0.2) # 5 FPS

    def update(self):
        if not self.running_program and not self.exited:
            pressed = hub.buttons.pressed()
            left_pressed = Button.LEFT in pressed
            right_pressed = Button.RIGHT in pressed
            center_pressed = Button.CENTER in pressed

            if left_pressed:
                if not self.last_left_pressed:
                    self.run_index = self.run_index - 1 if self.run_index > 0 else len(self.runs) - 1
                    self.last_left_pressed = True
            else:
                self.last_left_pressed = False
                
            if right_pressed:
                if not self.last_right_pressed:
                    self.run_index = self.run_index + 1 if self.run_index < len(self.runs) - 1 else 0
                    self.last_right_pressed = True
            else:
                self.last_right_pressed = False

            if center_pressed:
                if not self.last_center_pressed:
                    self.running_program = True
                    self.last_center_pressed = True
                    display.off() if _HUB_TYPE == "Spike" else display.clear()
                    #light.blink(Color.GREEN, [0.05, 0.05])
                    #wait(0.2)
                    #light._end_blink()
                    self._run_controller(runs[self.run_index])
            else:
                self.last_center_pressed = False
            
            DisplayNumber(self.run_index)

class DriveController:
    def __init__(self, run : TableRun):
        # Set args to class variables
        self.path = run.path

        # Assume the robot is started at the first point of the path
        self.xpos = self.path[0][0]
        self.ypos = self.path[0][1] 
        self.starting_angle = self._calculate_angle((self.path[0][0], self.path[0][1]), (self.path[1][0], self.path[1][1]))
        self.heading = self.starting_angle

        # Pure pursuit variables
        self.start_speed_factor = run.pure_pursuit_params['start_speed_factor']
        self.finished_distance = run.pure_pursuit_params['finished_distance']
        self.curvature_points = run.pure_pursuit_params['curvature_points']
        self.turn_vel_limit = run.pure_pursuit_params['turn_vel_limit']
        self.curvature_threshold = run.pure_pursuit_params['curvature_threshold']
        self.lookahead_points = run.pure_pursuit_params['lookahead_points']
        self.points_to_search = run.pure_pursuit_params['points_to_search']
        self.slowdown_dis = run.pure_pursuit_params['slowdown_dis']
        self.curvature_factor = run.pure_pursuit_params['curvature_factor']
        self.max_speed = run.pure_pursuit_params['max_speed'] 
        self.kp_turn = run.pure_pursuit_params['kp_turn']

        # Initialize other vars
        self.last_found_index = 0
        self.last_distance_left = 0
        self.last_distance_right = 0
        self.last_heading = 0
        self.turn_rate = 0
        self.target_speed = 0
        self.curvature = 0
        self.goal_index = 0
        self.status = False

    def _calculate_angle(self, pt1, pt2):
        '''Calculate the angle between two points

        Parameters:
            pt1 (list[float, float]): First point
            pt2 (list[float, float]): Second point

        Returns:
            float: Angle between the two points in degrees

        Example:
            angle = _calculate_angle([0, 0], [1, 1])
        '''

        # Calculate differences in coordinates
        delta_x = pt2[0] - pt1[0]
        delta_y = pt2[1] - pt1[1]
        
        # Calculate the angle using atan2
        angle_rad = math.atan2(delta_y, delta_x)
        
        # Convert the angle from radians to degrees
        angle_deg = math.degrees(angle_rad)
        
        # Adjust the angle to be in the range [0, 360)
        if angle_deg < 0:
            angle_deg += 360
        
        return round(angle_deg)
    
    def _calculate_turn_rate(self, turnVel, linearVel):
        '''Calculate the turn rate of the robot
        
        Parameters:
            turnVel (float): Turn velocity of the robot
            linearVel (float): Linear velocity of the robot
            
        Returns:
            float: Turn rate of the robot in degrees per second
            
        Example:
            turn_rate = _calculate_turn_rate(10, 20)
        '''

        # Calculate left and right motor speeds
        left_speed = linearVel - turnVel
        right_speed = linearVel + turnVel
        
        # Convert motor speeds to angular velocity (rad/s)
        left_ang_vel = left_speed / _WHEEL_RADIUS
        right_ang_vel = right_speed / _WHEEL_RADIUS
        
        # Calculate angular velocity (rad/s) for the turn
        ang_vel_robot = (right_ang_vel - left_ang_vel) * _WHEEL_RADIUS / _AXLE_TRACK
        
        # Convert angular velocity to degrees per second
        turn_rate_deg_per_sec = math.degrees(ang_vel_robot)
        
        return turn_rate_deg_per_sec

    def UpdateCoordinatesEV3(self):
        '''
        Update the robot's coordinates based on heading and distance traveled, accounting for curved paths when the robot turns.
        (Only for use on the EV3)

        Parameters:
            None

        Returns:
            None
            
        Example:
            UpdateCoordinates()
        '''

        # Get robot angle and motor angles
        self.heading = RobotAngle() + self.starting_angle
        angle_change = self.heading - self.last_heading

        # Calculate distances
        distance_left = WheelDistance("left")
        distance_right = WheelDistance("right")
        linear_distance = ((distance_right - self.last_distance_right) + (distance_left - self.last_distance_left)) / 2

        # If angle change is very small, consider it as straight movement
        if abs(angle_change) < 1e-6:
            cos_heading = math.cos(math.radians(self.heading))
            sin_heading = math.sin(math.radians(self.heading))
            self.xpos += cos_heading * linear_distance
            self.ypos += sin_heading * linear_distance
        else:
            # Update position based on the turn
            radius = linear_distance / math.radians(angle_change)
            cx = self.xpos - radius * math.sin(math.radians(self.last_heading))
            cy = self.ypos + radius * math.cos(math.radians(self.last_heading))
            self.xpos = cx + radius * math.sin(math.radians(self.heading))
            self.ypos = cy - radius * math.cos(math.radians(self.heading))

        # Update last known values
        self.last_distance_left = distance_left
        self.last_distance_right = distance_right
        self.last_heading = self.heading

    def UpdateCoordinatesSpike(self):
        '''
        Update the robot's coordinates based on heading and distance traveled, accounting for curved paths when the robot turns.
        (Only for use on the SPIKE)

        Parameters:
            None

        Returns:
            None
            
        Example:
            UpdateCoordinates()
        '''

        # Get robot angle and motor angles
        left_total_distance = WheelDistance("left")
        right_total_distance = WheelDistance("right")
        self.heading = RobotAngle()
        
        # Get linear distance traveled and angle chance
        left_distance_traveled = left_total_distance - self.last_distance_left
        right_distance_traveled = right_total_distance - self.last_distance_right
        linear = (left_distance_traveled + right_distance_traveled) / 2
        angle_change = self.heading - self.last_heading

        # Check if the robot moved or changed angles
        if angle_change == 0 or linear == 0:
            # Calculate coordinates in a straight line
            self.xpos += math.sin(math.radians(self.heading)) * linear
            self.ypos += math.cos(math.radians(self.heading)) * linear
        else:
            # Calculate the radius of the robot's turning path
            radius = linear / math.radians(angle_change)

            # Calculate the center of the robot's turning circle
            center_x = self.xpos - radius * math.sin(math.radians(self.last_heading))
            center_y = self.ypos + radius * math.cos(math.radians(self.last_heading))

            # Update the robot's position based on the arc traveled along the turning circle
            self.xpos = center_x + radius * math.sin(math.radians(self.heading))
            self.ypos = center_y - radius * math.cos(math.radians(self.heading))

        # Update the last distances
        self.last_distance_left = left_total_distance
        self.last_distance_right = right_total_distance
        self.last_heading = self.heading

    def _calculate_curvature(self, points):
        '''
        Calculate the curvature between points in a path

        Parameters:
            points (list[list[float, float]]): List of points in the path

        Returns:
            float: Curvature of the path

        Example:
            curvature = _calculate_curvature([[0, 0], [1, 1]])
        '''

        # Ensure there are at least 3 points to calculate curvature
        if len(points) < 3:
            return 0
        
        angles = []
        for i in range(1, len(points) - 1):
            # Calculate vectors
            vector1 = [points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]]
            vector2 = [points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1]]
            
            # Calculate the dot product
            dot_product = vector1[0] * vector2[0] + vector1[1] * vector2[1]
            
            # Calculate norms (magnitudes)
            norm1 = math.sqrt(vector1[0] ** 2 + vector1[1] ** 2)
            norm2 = math.sqrt(vector2[0] ** 2 + vector2[1] ** 2)

            # Avoid division by zero
            if norm1 == 0 or norm2 == 0:
                angle = 0
            else:
                # Calculate the angle between vectors
                cosine_value = dot_product / (norm1 * norm2)
                cosine_value = max(-1, min(1, cosine_value))  # Clamp to [-1, 1] to prevent domain errors
                angle = math.acos(cosine_value)
            
            # Append the absolute value of the angle to the list
            angles.append(abs(angle))

        # Sum up all absolute angles to get the total curvature
        total_curvature = sum(angles)

        return total_curvature
    
    def _calculate_distance(self, pt1, pt2):
        '''
        Calculate the distance between two points

        Parameters:
            pt1 (list[float, float]): First point
            pt2 (list[float, float]): Second point

        Returns:
            float: Distance between the two points

        Example:
            distance = _calculate_distance([0, 0], [1, 1])
        '''

        return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)

    def _sign(self, x):
        '''
        Get the sign of a number

        Parameters:
            x (float): Number to get the sign of

        Returns:
            int: Sign of the number

        Example:
            sign = _sign(1)
        '''

        return 1 if x > 0 else (-1 if x < 0 else 0)
    
    def RunPurePursuit(self):
        '''
        Pure pursuit algorithm to follow a path by steering toward a lookahead point.
        '''

        # Precompute values
        heading_rad = math.radians(self.heading)
        cos_heading = math.cos(heading_rad)
        sin_heading = math.sin(heading_rad)

        # Initialize variables for the closest point search
        min_dist_sq = float('inf')
        closest_index = self.last_found_index
        path_length = len(self.path)

        # Efficiently find the closest point and lookahead point using precomputed range
        for i in range(self.last_found_index, min(self.last_found_index + self.points_to_search, path_length)):
            x, y = self.path[i]
            dx, dy = x - self.xpos, y - self.ypos
            dist_sq = dx * dx + dy * dy
            if dist_sq < min_dist_sq:
                min_dist_sq, closest_index = dist_sq, i

        # Update the last found index to the closest point for efficiency
        self.last_found_index = closest_index

        # Define a lookahead point (a few steps ahead of the closest point)
        lookahead_index = min(closest_index + self.lookahead_points, path_length - 1)
        lookahead_point = self.path[lookahead_index]

        # Calculate the angle from the robot to the lookahead point
        dx = lookahead_point[0] - self.xpos
        dy = lookahead_point[1] - self.ypos
        target_angle = math.degrees(math.atan2(dy, dx))

        # Calculate the turn error
        turn_error = target_angle - self.heading
        if turn_error > 180:
            turn_error -= 360
        elif turn_error < -180:
            turn_error += 360

        # Compute turn velocity with clamping
        turn_vel = turn_error * self.kp_turn
        turn_vel = max(-self.turn_vel_limit, min(turn_vel, self.turn_vel_limit))

        # Slow down as the robot approaches the end of the path
        final_dist = self._calculate_distance(self.path[-1], (self.xpos, self.ypos))
        slow_end_factor = min((final_dist / self.slowdown_dis) ** 1.5, 1.0)

        # Adjust speed based on curvature near the lookahead point
        curvature = self._calculate_curvature(self.path[max(closest_index - 2, 0): lookahead_index])
        curvature_factor = 1.0 / (1.0 + self.curvature_factor * max(0, curvature - self.curvature_threshold))

        # Calculate target speed
        self.target_speed = min(self.max_speed * slow_end_factor * curvature_factor, self.max_speed)

        # Set the turn rate and speed for the robot
        self.turn_rate = self._calculate_turn_rate(turn_vel, self.target_speed)

        # Update completion status if close to the final point
        self.status = final_dist < self.finished_distance

runs = [
    TableRun("Mission X", "Mission X", "left", [[44.0, 489.0], [48.94, 467.63], [54.03, 446.35], [59.44, 425.23], [65.33, 404.34], [71.86, 383.79], [79.17, 363.63], [87.45, 343.96], [96.83, 324.85], [107.49, 306.39], [119.54, 288.65], [132.93, 271.69], [147.52, 255.54], [163.18, 240.25], [179.78, 225.86], [197.19, 212.41], [215.26, 199.95], [233.88, 188.52], [252.91, 178.16], [272.2, 168.91], [291.67, 160.78], [311.31, 153.63], [331.14, 147.28], [351.19, 141.58], [371.49, 136.33], [392.06, 131.37], [412.93, 126.54], [434.12, 121.64], [455.66, 116.52], [477.58, 110.99], [499.9, 104.89], [522.59, 98.19], [545.53, 91.19], [568.55, 84.24], [591.5, 77.71], [614.23, 71.94], [636.58, 67.3], [658.4, 64.13], [679.53, 62.8], [699.81, 63.66], [719.1, 67.07], [737.23, 73.37], [754.06, 82.93], [769.52, 95.72], [783.67, 111.23], [796.57, 128.9], [808.31, 148.17], [818.94, 168.49], [828.55, 189.3], [837.18, 210.08], [844.87, 230.7], [851.62, 251.22], [857.42, 271.72], [862.28, 292.28], [866.19, 312.96], [869.17, 333.84], [871.29, 354.92], [872.65, 376.18], [873.36, 397.59], [873.54, 419.12], [873.28, 440.75], [872.7, 462.46], [871.9, 484.22], [871.0, 506.0]], {
        'start_speed_factor': 10,
        'finished_distance': 10,
        'curvature_points': 5,
        'turn_vel_limit': 100,
        'curvature_threshold': 0.03,
        "points_to_search": 5,
        'lookahead_points': 3,
        'slowdown_dis': 30,
        'curvature_factor': 1.4,
        'max_speed': 175,
        'kp_turn': 1.9
    }),
    TableRun("Mission Y", "Mission Y", "left", [[10, 0], [0, 10]], {
        'start_speed_factor': 10,
        'finished_distance': 0.1,
        'curvature_points': 10,
        'turn_vel_limit': 360,
        'curvature_threshold': 0.1,
        'points_to_search': 5,
        'lookahead_points': 3,
        'slowdown_dis': float("inf"),
        'curvature_factor': 0.1,
        'max_speed': 300,
        'kp_turn': 0.1
    }),
    TableRun("Mission Z", "Mission Z", "left", [[10, 0], [0, 10]], {
        'start_speed_factor': 10,
        'finished_distance': 0.1,
        'curvature_points': 10,
        'turn_vel_limit': 360,
        'curvature_threshold': 0.1,
        'points_to_search': 5,
        'lookahead_points': 3,
        'slowdown_dis': float("inf"),
        'curvature_factor': 0.1,
        'max_speed': 300,
        'kp_turn': 0.1
    }),
]

if _VISUALIZE:
    print("Initializing visualization server...")
    visualization = VisualizationServer()
    print("Connected to visualization server!")
else:
    visualization = None

wait(1500) # Wait for threads to initialize and display the banner
light.off()
try:
    selector = CustomRunSelector(runs, visualization) # Start the run selector
    print("Initialized the run selector! Select a run to begin.")
except Exception as e:
    light.blink(Color.RED, [0.5, 0.5]) # Turn on red light to indicate error
    print("Error when initializing the run selector:\n" + str(e))
    input("Press Enter to exit...")
    quit()

while True:
    selector.update()
    wait(0.1) # 20 FPS
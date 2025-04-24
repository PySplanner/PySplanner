#!/usr/bin/env pybricks-micropython

# PySplanner - A Pure Pursuit Robot Controller
# Licensed under the GNU General Public License v3.0
# © 2024 DylDev (https://github.com/DylDevs)
# © 2024 techcatgato (https://github.com/techcatgato)
# © 2024 Skrauzy (https://github.com/PumpkinDoesCoding)
# © 2024 Sketchy (https://github.com/3xSketchy)
# NOTE: Comments and some QOL features have been removed to save on file size

PATH_PLANNER_DATA = "{INSERT_PATH_PLANNER_DATA}"

_HUB_TYPE = PATH_PLANNER_DATA["hub_type"]
_VISUALIZE = True
_VISUALIZE_IP = "169.254.219.105"
_VISUALIZE_PORT = 65432
_SEND_EVERY_X_FRAMES = 3

from pybricks.parameters import Button, Color, Port
if _HUB_TYPE == "Spike":
    from pybricks.hubs import PrimeHub

    hub = PrimeHub()
    hub.system.set_stop_button(Button.BLUETOOTH)
    display = hub.display
    display.off()

    from pybricks.tools import multitask as thread_handler
    from pybricks.pupdevices import Motor
    import ustruct as struct
    from usys import stdout
    import umath as math
else:
    from pybricks.hubs import EV3Brick

    hub = EV3Brick()
    display = hub.screen
    display.clear()

    from threading import Thread as thread_handler
    from pybricks.media.ev3dev import Font
    from pybricks.ev3devices import Motor
    import socket
    import struct
    import math

    FONT = Font('Lucida', 32, bold=True)
    display.set_font(FONT)

from micropython import const, opt_level
from pybricks.robotics import DriveBase
from pybricks.tools import wait

opt_level(3)

def GetPort(port: str["A", "B", "C", "D", "E", "F"]) -> Port:
    if port == "A": return Port.A
    if port == "B": return Port.B
    if port == "C": return Port.C
    if port == "D": return Port.D
    if port == "E": return Port.E
    if port == "F": return Port.F

    raise ValueError("Invalid port: " + port)

# Variables needed for classes and functions
_WHEEL_DIAMETER = PATH_PLANNER_DATA["drive_base"]["wheel_diameter"]
_AXLE_TRACK = PATH_PLANNER_DATA["drive_base"]["axle_track"]
_WHEEL_RADIUS = _WHEEL_DIAMETER / 2
_WHEEL_CIRCUMFERENCE = math.pi * _WHEEL_DIAMETER
_LEFT_MOTOR = Motor(GetPort(PATH_PLANNER_DATA["drive_base"]["left_motor"]))
_RIGHT_MOTOR = Motor(GetPort(PATH_PLANNER_DATA["drive_base"]["right_motor"]))
_ROBOT = DriveBase(_LEFT_MOTOR, _RIGHT_MOTOR, _WHEEL_DIAMETER, _AXLE_TRACK)

quit_program = False
class ThreadedFunction:
    class MissingArgumentVariableError(Exception):
        pass

    def __init__(self, function: callable, args_variable_name: str, args: tuple = ()):
        if not callable(function):
            raise TypeError("ThreadedFunction received non-callable object: " + str(function))
        if not isinstance(args, tuple):
            args = (args,)
        if not isinstance(args_variable_name, str):
            raise TypeError("args_variable_name must be a string, got " + type(args_variable_name))

        self.function = function
        self.arg_variable = args_variable_name

        if self.arg_variable not in globals():
            raise self.MissingArgumentVariableError("Function argument variable not found. Define it as follows:\n" + self.arg_variable + " = ()")
        else:
            globals()[self.arg_variable] = args

def RunInThread(*funcs: ThreadedFunction):
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
    def __init__(self):
        self.light = hub.light
        self.blinking = False
        self.stop_blinking = False
        pass

    def blink(self, color: Color, durations: tuple[float, float]):
        self.end_blink()
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
        self.end_blink()
        self.light.on(color)
    
    def off(self):
        self.end_blink()
        self.light.off()

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
    if _HUB_TYPE == "Spike":
        return hub.imu.heading()
    else:
        return _ROBOT.angle()

def WheelDistance(side = "left"):
    return ((_LEFT_MOTOR.angle() if side == "left" else _RIGHT_MOTOR.angle()) / 360) * _WHEEL_CIRCUMFERENCE

last_displayed_number = None
def DisplayNumber(number : int):
    global last_displayed_number
    if number != last_displayed_number:
        if _HUB_TYPE == "Spike":
            display.number(number)
        else:
            display.clear()
            text_w, text_h = FONT.text_width(str(number)), FONT.text_height(str(number))
            display.draw_text(178 / 2 - text_w / 2, 128 / 2 - text_h / 2, str(number))
        last_displayed_number = number

class TableRun:
    def __init__(self, name : str, description : str, table_side : str, path : list[list[int, float]]):
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

_run_program_args = ()
class CustomRunSelector:
    def __init__(self, runs : list[TableRun], visualization : VisualizationServer | None = None):
        self.runs = runs
        self.run_index = 0
        self.last_left_pressed = False
        self.last_right_pressed = False
        self.last_center_pressed = False
        self.running_program = False
        self.exited = False
        self.quit_program = False
        self.visualization = visualization

    def get_status(self):
        return self.exited == False
    
    def _run_program(self, run : TableRun | None = None) -> None:
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
        while not controller.status and not quit_program:
            controller.UpdateCoordinatesSpike() if _HUB_TYPE == "Spike" else controller.UpdateCoordinatesEV3()
            controller.RunPurePursuit()
            _ROBOT.drive(controller.target_speed, controller.turn_rate)

            frames += 1
            if self.visualization is not None and frames == _SEND_EVERY_X_FRAMES:
                self.visualization.send_data(controller.xpos, controller.ypos, controller.heading, controller.target_speed, controller.curvature, controller.goal_index)
                frames = 0

        _ROBOT.stop()
        print("Finished running: " + run.name)

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
                    self._run_controller(runs[self.run_index])
            else:
                self.last_center_pressed = False
            
            DisplayNumber(self.run_index)

class DriveController:
    def __init__(self, run : TableRun):
        self.path = run.path
        self.xpos = self.path[0][0]
        self.ypos = self.path[0][1] 
        self.starting_angle = self._calculate_angle((self.path[0][0], self.path[0][1]), (self.path[1][0], self.path[1][1]))
        self.heading = self.starting_angle
        self.start_speed_factor = 10
        self.finished_distance = 10
        self.curvature_points = 5
        self.turn_vel_limit = 100
        self.curvature_threshold = 0.03
        self.points_to_search = 5
        self.lookahead_points = 3
        self.slowdown_dis = 30
        self.curvature_factor = 1.4
        self.max_speed = 175
        self.kp_turn = 1.9
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
        delta_x = pt2[0] - pt1[0]
        delta_y = pt2[1] - pt1[1]
        angle_rad = math.atan2(delta_y, delta_x)
        angle_deg = math.degrees(angle_rad)
        
        if angle_deg < 0:
            angle_deg += 360
        
        return round(angle_deg)
    
    def _calculate_turn_rate(self, turnVel, linearVel):
        left_speed = linearVel - turnVel
        right_speed = linearVel + turnVel
        left_ang_vel = left_speed / _WHEEL_RADIUS
        right_ang_vel = right_speed / _WHEEL_RADIUS
        ang_vel_robot = (right_ang_vel - left_ang_vel) * _WHEEL_RADIUS / _AXLE_TRACK
        turn_rate_deg_per_sec = math.degrees(ang_vel_robot)
        
        return turn_rate_deg_per_sec

    def UpdateCoordinatesEV3(self):
        self.heading = RobotAngle() + self.starting_angle
        angle_change = self.heading - self.last_heading

        distance_left = WheelDistance("left")
        distance_right = WheelDistance("right")
        linear_distance = ((distance_right - self.last_distance_right) + (distance_left - self.last_distance_left)) / 2

        if abs(angle_change) < 1e-6:
            cos_heading = math.cos(math.radians(self.heading))
            sin_heading = math.sin(math.radians(self.heading))
            self.xpos += cos_heading * linear_distance
            self.ypos += sin_heading * linear_distance
        else:
            radius = linear_distance / math.radians(angle_change)
            cx = self.xpos - radius * math.sin(math.radians(self.last_heading))
            cy = self.ypos + radius * math.cos(math.radians(self.last_heading))
            self.xpos = cx + radius * math.sin(math.radians(self.heading))
            self.ypos = cy - radius * math.cos(math.radians(self.heading))

        self.last_distance_left = distance_left
        self.last_distance_right = distance_right
        self.last_heading = self.heading

    def UpdateCoordinatesSpike(self):
        left_total_distance = WheelDistance("left")
        right_total_distance = WheelDistance("right")
        self.heading = RobotAngle()
        
        left_distance_traveled = left_total_distance - self.last_distance_left
        right_distance_traveled = right_total_distance - self.last_distance_right
        linear = (left_distance_traveled + right_distance_traveled) / 2
        angle_change = self.heading - self.last_heading

        if abs(angle_change) < 1e-6 or linear == 0:
            self.xpos += math.sin(math.radians(self.heading)) * linear
            self.ypos += math.cos(math.radians(self.heading)) * linear
        else:
            radius = (linear * (360 / abs(angle_change))) / (2 * math.pi)

            # THis is the old stuff, don't remove it because the new stuff isn't 100% tested
            #center_x = self.xpos - radius * math.sin(math.radians(self.last_heading))
            #center_y = self.ypos + radius * math.cos(math.radians(self.last_heading))
            #self.xpos = center_x + radius * math.sin(math.radians(self.heading))
            #self.ypos = center_y - radius * math.cos(math.radians(self.heading))

            rX = radius - (math.cos(math.radians(abs(angle_change))) * radius)
            rY = math.sin(math.radians(abs(angle_change))) * radius
            rD = math.sqrt(rX ** 2 + rY ** 2)
            rA = math.radians(self.last_heading) + (math.asin(rX / rD) * (angle_change / abs(angle_change)))
            self.xpos += math.sin(rA) * rD
            self.ypos += math.cos(rA) * rD

        self.last_distance_left = left_total_distance
        self.last_distance_right = right_total_distance
        self.last_heading = self.heading

    def _calculate_curvature(self, points):
        if len(points) < 3:
            return 0
        
        angles = []
        for i in range(1, len(points) - 1):
            vector1 = [points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]]
            vector2 = [points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1]]
            dot_product = vector1[0] * vector2[0] + vector1[1] * vector2[1]
            norm1 = math.sqrt(vector1[0] ** 2 + vector1[1] ** 2)
            norm2 = math.sqrt(vector2[0] ** 2 + vector2[1] ** 2)

            if norm1 == 0 or norm2 == 0:
                angle = 0
            else:
                cosine_value = dot_product / (norm1 * norm2)
                cosine_value = max(-1, min(1, cosine_value))
                angle = math.acos(cosine_value)

            angles.append(abs(angle))

        total_curvature = sum(angles)

        return total_curvature
    
    def _calculate_distance(self, pt1, pt2):
        return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)

    def _sign(self, x):
        return 1 if x > 0 else (-1 if x < 0 else 0)
    
    def RunPurePursuit(self):
        min_dist_sq = float('inf')
        closest_index = self.last_found_index
        path_length = len(self.path)

        for i in range(self.last_found_index, min(self.last_found_index + self.points_to_search, path_length)):
            x, y = self.path[i]
            dx, dy = x - self.xpos, y - self.ypos
            dist_sq = dx * dx + dy * dy
            if dist_sq < min_dist_sq:
                min_dist_sq, closest_index = dist_sq, i

        self.last_found_index = closest_index

        lookahead_index = min(closest_index + self.lookahead_points, path_length - 1)
        lookahead_point = self.path[lookahead_index]
        dx = lookahead_point[0] - self.xpos
        dy = lookahead_point[1] - self.ypos
        target_angle = math.degrees(math.atan2(dy, dx))

        turn_error = target_angle - self.heading
        if turn_error > 180:
            turn_error -= 360
        elif turn_error < -180:
            turn_error += 360

        turn_vel = turn_error * self.kp_turn
        turn_vel = max(-self.turn_vel_limit, min(turn_vel, self.turn_vel_limit))

        final_dist = self._calculate_distance(self.path[-1], (self.xpos, self.ypos))
        slow_end_factor = min((final_dist / self.slowdown_dis) ** 1.5, 1.0)
        curvature = self._calculate_curvature(self.path[max(closest_index - 2, 0): lookahead_index])
        curvature_factor = 1.0 / (1.0 + self.curvature_factor * max(0, curvature - self.curvature_threshold))

        self.target_speed = min(self.max_speed * slow_end_factor * curvature_factor, self.max_speed)
        self.turn_rate = self._calculate_turn_rate(turn_vel, self.target_speed)
        self.status = final_dist < self.finished_distance

runs = []
for run in PATH_PLANNER_DATA["runs"]:
    runs.append(TableRun(run["name"], "", "left", run["points"]))

if _VISUALIZE:
    print("Initializing visualization server...")
    visualization = VisualizationServer()
    print("Connected to visualization server!")
else:
    visualization = None

wait(1500)
light.off()
try:
    selector = CustomRunSelector(runs, visualization)
    print("Initialized the run selector! Select a run to begin.")
except Exception as e:
    light.blink(Color.RED, [0.5, 0.5])
    print("Error when initializing the run selector:\n" + str(e))
    input("Press Enter to exit...")
    quit()

while True:
    selector.update()
    wait(0.1)

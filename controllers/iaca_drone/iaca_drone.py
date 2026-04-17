from controller import Robot
import json
import math
import numpy as np
import os
import sys
from math import cos, sin

from pid_controller import pid_velocity_fixed_height_controller

current_dir = os.path.dirname(__file__)
shared_path = os.path.abspath(os.path.join(current_dir, "..", "shared"))
sys.path.append(shared_path)

import controllers.shared.equations as eq


FLYING_ATTITUDE = 2.0
## Known working values
# DELTA_V_MAX = 0.05
# ALPHA_VELOCITY = 0.92
# MAX_WORLD_SPEED = 1.0
# BOUNDARY_STRENGTH = 0.5
# BOUNDARY_MARGIN = 4.0


## Unknown values im testing to try and improve drone speed / turning
BOUNDARY_STRENGTH = .9
BOUNDARY_MARGIN = 5.0

DELTA_V_MAX = 0.08
ALPHA_VELOCITY = 0.88
MAX_WORLD_SPEED = 1.5


D_MAX = math.sqrt(2.0)  # max diagonal distance between neighboring cells in grid units

WORLD_X_MIN = -30.0
WORLD_X_MAX = 30.0
WORLD_Y_MIN = -30.0
WORLD_Y_MAX = 35.0


robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Motors
m1_motor = robot.getDevice("m1_motor")
m1_motor.setPosition(float("inf"))
m1_motor.setVelocity(-1)

m2_motor = robot.getDevice("m2_motor")
m2_motor.setPosition(float("inf"))
m2_motor.setVelocity(1)

m3_motor = robot.getDevice("m3_motor")
m3_motor.setPosition(float("inf"))
m3_motor.setVelocity(-1)

m4_motor = robot.getDevice("m4_motor")
m4_motor.setPosition(float("inf"))
m4_motor.setVelocity(1)

# Sensors
imu = robot.getDevice("inertial_unit")
imu.enable(timestep)

gps = robot.getDevice("gps")
gps.enable(timestep)

gyro = robot.getDevice("gyro")
gyro.enable(timestep)

camera = robot.getDevice("camera")
camera.enable(timestep)

range_front = robot.getDevice("range_front")
range_front.enable(timestep)

range_left = robot.getDevice("range_left")
range_left.enable(timestep)

range_back = robot.getDevice("range_back")
range_back.enable(timestep)

range_right = robot.getDevice("range_right")
range_right.enable(timestep)

receiver = robot.getDevice("cmd_receiver")
receiver.enable(timestep)

PID_crazyflie = pid_velocity_fixed_height_controller()

past_x_global = 0.0
past_y_global = 0.0
past_time = 0.0
first_time = True

# Drone state
v_world = np.zeros(2, dtype=float)       # current filtered world-frame velocity
yaw_desired = 0.0
height_desired = FLYING_ATTITUDE
last_neighbors = {}                       # most recent neighbor dict from supervisor
startup = True

print("Crazyflie iaca_drone started")


def clamp_vector_norm(v, max_norm):
    norm = np.linalg.norm(v)
    if norm == 0.0 or norm <= max_norm:
        return v
    return (v / norm) * max_norm

def boundary_force(drone_pos, x_min, x_max, y_min, y_max, margin, strength):
    """
    Returns a force pushing drone back toward map center when near or outside bounds.
    Force scales linearly from 0 at the margin boundary to full strength at the edge.
    """
    x, y = drone_pos
    fx, fy = 0.0, 0.0

    # X boundaries
    if x < x_min + margin:
        t = 1.0 - max(x - x_min, 0.0) / margin
        fx += strength * t
    elif x > x_max - margin:
        t = 1.0 - max(x_max - x, 0.0) / margin
        fx -= strength * t

    # Y boundaries
    if y < y_min + margin:
        t = 1.0 - max(y - y_min, 0.0) / margin
        fy += strength * t
    elif y > y_max - margin:
        t = 1.0 - max(y_max - y, 0.0) / margin
        fy -= strength * t

    return np.array([fx, fy], dtype=float)


while robot.step(timestep) != -1:
    current_time = robot.getTime()

    if first_time:
        past_x_global = gps.getValues()[0]
        past_y_global = gps.getValues()[1]
        past_time = current_time
        first_time = False
        continue

    dt = current_time - past_time
    if dt <= 0.0:
        continue
    dt = max(dt, 1e-3)

    # Read newest command packet
    while receiver.getQueueLength() > 0:
        data = receiver.getString()
        command = json.loads(data)
        last_neighbors = command.get("neighbors", {})
        yaw_desired = float(command.get("yaw_desired", yaw_desired))
        height_desired = float(command.get("height_desired", height_desired))
        startup = bool(command.get("startup", True))
        receiver.nextPacket()

    # Sensor readings
    roll = imu.getRollPitchYaw()[0]
    pitch = imu.getRollPitchYaw()[1]
    yaw = imu.getRollPitchYaw()[2]
    yaw_rate = gyro.getValues()[2]

    x_global = gps.getValues()[0]
    y_global = gps.getValues()[1]
    altitude = gps.getValues()[2]

    # Compute APF force and update velocity in world frame
    if startup or len(last_neighbors) == 0:
        v_world = np.zeros(2, dtype=float)
    else:
        drone_pos = np.array([x_global, y_global], dtype=float)

        # Rebuild neighbor list in the format get_total_attracting_force expects
        neighbors = []
        for key, val in last_neighbors.items():
            cell_center = (val["wx"], val["wy"])
            Q_k = float(val["q"])
            neighbors.append((cell_center, Q_k))

        f_total = eq.get_total_attracting_force(
            neighbors=neighbors,
            drone_pos=drone_pos,
            v_old=v_world,
            D_max=D_MAX
        )

        f_boundary = boundary_force(
            drone_pos,
            WORLD_X_MIN, WORLD_X_MAX,
            WORLD_Y_MIN, WORLD_Y_MAX,
            margin=BOUNDARY_MARGIN,
            strength=BOUNDARY_STRENGTH
        )
        f_total = f_total + f_boundary

        f_norm = np.linalg.norm(f_total)

        if f_norm < 1e-9:
            # Bleed velocity slowly, don't snap to zero
            v_world = v_world * 0.95
        else:
            # Scale down DELTA_V_MAX if force direction opposes current motion
            v_world_norm = np.linalg.norm(v_world)
            if v_world_norm > 1e-6:
                cos_angle = np.dot(f_total / f_norm, v_world / v_world_norm)
                # cos_angle = 1 means same direction, -1 means opposite
                # Scale increment down to 20% when fully opposing, full when aligned
                turn_scale = 0.2 + 0.8 * ((cos_angle + 1.0) / 2.0)
                effective_delta = DELTA_V_MAX * turn_scale
            else:
                effective_delta = DELTA_V_MAX

            v_new = eq.update_drone_velocity(
                v_current=v_world,
                F_a=f_total,
                v_max=effective_delta
            )
            v_world = eq.stability_aware_velocity_adjustment(
                v_t_minus_1=v_world,
                v_new=v_new,
                alpha=ALPHA_VELOCITY
            )
            v_world = clamp_vector_norm(v_world, MAX_WORLD_SPEED)

    v_body = eq.global_to_body_velocity(v_world, yaw)
    forward_desired = v_body[0]
    sideways_desired = v_body[1]

    # PID controller
    motor_power = PID_crazyflie.pid(
        dt,
        forward_desired,
        sideways_desired,
        yaw_desired,
        height_desired,
        roll,
        pitch,
        yaw_rate,
        altitude,
        # actual body-frame velocity from GPS diff
        (x_global - past_x_global) / dt * cos(yaw) + (y_global - past_y_global) / dt * sin(yaw),
        -(x_global - past_x_global) / dt * sin(yaw) + (y_global - past_y_global) / dt * cos(yaw),
    )

    m1_motor.setVelocity(-motor_power[0])
    m2_motor.setVelocity(motor_power[1])
    m3_motor.setVelocity(-motor_power[2])
    m4_motor.setVelocity(motor_power[3])

    past_time = current_time
    past_x_global = x_global
    past_y_global = y_global

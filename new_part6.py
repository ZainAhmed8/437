import numpy as np
import time
import math

from picarx import Picarx
np.set_printoptions(threshold=np.inf)

px = Picarx()

MOVE_SPEED = 10 #speed of car
MOVE_STEP = 5 #forward move
TURN_STEP = 5 #increment of turn
TURN_RANGE = 30 #max steering range

MAP_SIZE = 100  # 100 cm x 100 cm grid
occupancy_grid = np.zeros((MAP_SIZE, MAP_SIZE), dtype=int)

car_x, car_y = MAP_SIZE // 2, MAP_SIZE // 2
car_theta = 0.0  

def scan_environment(px, move_speed=MOVE_SPEED, move_step=MOVE_STEP, turn_range=TURN_RANGE, turn_step=TURN_STEP):
    """
    Scans the environment by reversing slightly, then moving forward step by step.
    During each forward step, the steering servo turns slightly to simulate a wider scan.
    """
    scan_data = []

    # reverse and create space
    px.backward(move_speed)
    time.sleep(0.3)
    px.stop()

    # move forward while reading
    for angle in range(-turn_range, turn_range + 1, turn_step):
        px.set_dir_servo_angle(angle)
        time.sleep(0.3)

        px.forward(move_speed)
        time.sleep(0.3)
        px.stop()

        dist = px.get_distance()
        if dist < 0:
            dist = 1
        scan_data.append((angle, dist))
        time.sleep(0.5)

        px.backward(move_speed)
        time.sleep(0.3)
        px.stop()

    # reset steering to center
    px.set_dir_servo_angle(0)
    time.sleep(0.1)

    return scan_data


def polar_to_grid(car_x, car_y, car_theta, angle_deg, distance_cm):
    """
    Convert polar coordinate (angle, distance) relative to car into
    occupancy-grid coordinates.
    """
    angle_rad = math.radians(angle_deg)
    theta_rad = car_theta

    x_obj = car_x + distance_cm * math.cos(theta_rad + angle_rad)
    y_obj = car_y + distance_cm * math.sin(theta_rad + angle_rad)
    
    x_idx = min(max(int(round(x_obj)), 0), MAP_SIZE - 1)
    y_idx = min(max(int(round(y_obj)), 0), MAP_SIZE - 1)

    
    return x_idx, y_idx


def update_occupancy_grid(occupancy_grid, scan_points, car_x, car_y, car_theta):
    """
    Updates the occupancy grid based on a list of (angle_deg, distance_cm) readings.
    We also fill in the line between consecutive scan points if needed.
    """
    for i in range(len(scan_points) - 1):
        angle1, dist1 = scan_points[i]
        angle2, dist2 = scan_points[i+1]
        
        # Convert to grid coords
        x1, y1 = polar_to_grid(car_x, car_y, car_theta, angle1, dist1)
        x2, y2 = polar_to_grid(car_x, car_y, car_theta, angle2, dist2)
        
        # Mark these endpoints in the grid
        mark_point(occupancy_grid, x1, y1)
        mark_point(occupancy_grid, x2, y2)
        
        # Interpolate the line between the two points
        fill_line(occupancy_grid, x1, y1, x2, y2)

def mark_point(grid, x, y):
    if 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]:
        grid[x, y] = 1
        print(f"Marked: ({x}, {y})")

def fill_line(grid, x1, y1, x2, y2):
    """Mark a line of points between (x1, y1) and (x2, y2)."""
    # A simple Bresenham's line or approximate method:
    points = bresenham_line(x1, y1, x2, y2)
    for (x, y) in points:
        mark_point(grid, x, y)

def bresenham_line(x1, y1, x2, y2):
    """Return a list of points between (x1, y1) and (x2, y2) using Bresenham's line algorithm."""
    points = []
    dx = abs(x2 - x1)
    sx = 1 if x1 < x2 else -1
    dy = -abs(y2 - y1)
    sy = 1 if y1 < y2 else -1
    err = dx + dy
    x, y = x1, y1
    
    while True:
        points.append((x, y))
        if x == x2 and y == y2:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    return points

def move_car(px, distance_cm, speed=MOVE_SPEED):
    """Moves the car forward a specified distance (cm)."""
    move_time = distance_cm / speed
    px.forward(speed)
    time.sleep(move_time)
    px.stop()

def update_car_position(car_x, car_y, car_theta, distance_cm):
    """Updates the car's global position after moving forward."""
    new_x = car_x + distance_cm * math.cos(car_theta)
    new_y = car_y + distance_cm * math.sin(car_theta)
    return new_x, new_y


def update_pose(car_x, car_y, car_theta, linear_velocity, angular_velocity, dt):
    """
    Update car pose given linear velocity (cm/s), angular velocity (rad/s), and time interval dt.
    """
    new_x = car_x + linear_velocity * dt * math.cos(car_theta)
    new_y = car_y + linear_velocity * dt * math.sin(car_theta)
    new_theta = car_theta + angular_velocity * dt
    return new_x, new_y, new_theta



def main_loop():
    global car_x, car_y, car_theta, occupancy_grid

    last_time = time.time()
    n = 0

    current_time = time.time()
    dt = current_time - last_time  # Calculate time step
    last_time = current_time

    # Get current steering angle and estimate angular velocity
    steering_angle = px.dir_current_angle  # Current servo angle
    angular_velocity = math.radians(steering_angle) / 2  # Estimate

    # Perform scanning and movement
    scan_points = scan_environment(px)

    # Update the car’s pose
    car_x, car_y, car_theta = update_pose(car_x, car_y, car_theta, MOVE_SPEED, angular_velocity, dt)

    update_occupancy_grid(occupancy_grid, scan_points, car_x, car_y, car_theta)
    # Print position updates
    print(f"Car Position: x={car_x:.2f}, y={car_y:.2f}, theta={math.degrees(car_theta):.2f}°")
    print("Scanned Data:", scan_points)

    time.sleep(1)

    px.set_dir_servo_angle(15)
    px.forward(MOVE_SPEED)
    time.sleep(5)
    px.stop()

    # Get current steering angle and estimate angular velocity
    steering_angle = px.dir_current_angle  # Current servo angle
    angular_velocity = math.radians(steering_angle) / 2  # Estimate

    car_x, car_y, car_theta = update_pose(car_x, car_y, car_theta, MOVE_SPEED, angular_velocity, dt)


    scan_points = scan_environment(px)
    update_occupancy_grid(occupancy_grid, scan_points, car_x, car_y, car_theta)


    print(occupancy_grid)



if __name__ == "__main__":
    try:
        main_loop()
    except KeyboardInterrupt:
        print("\nExiting...")
        px.stop()




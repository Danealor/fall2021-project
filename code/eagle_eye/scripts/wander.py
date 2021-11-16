#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import math
import numpy as np
from matplotlib import pyplot as plt
from tf.transformations import euler_from_quaternion


### Occupancy Grid ###

class Grid(object):

    def __init__(self, dtype=bool,
        cell_width=1., cell_height=1., 
        init_cols=1000, init_rows=1000, 
        init_shift_x=0., init_shift_y=0.):

        self.matrix = np.zeros((init_rows, init_cols), dtype=dtype)
        self._default_val = np.zeros(1, dtype=dtype)[0]

        # Height is negative because we flip vertically
        self.cell_dims = np.array([cell_width, -cell_height], dtype=float)

        self.shift = np.array([init_shift_x, init_shift_y], dtype=float)

    def index(self, coords):
        "Return row,col indices from x,y coordinates."
        idx = ((coords - self.shift) // self.cell_dims).astype(int) # (x,y) => (j,i)
        return idx[...,::-1] # (j,i) => (i,j)

    _corner_tf = np.array([[0,1],[0,0],[1,1],[1,0]])
    def corners(self, idx):
        "Return list of x,y coordinates of the corners of the cells at index i,j"
         # (i,j) => (j,i) => (x,y)
        origin = idx[...,::-1] * self.cell_dims + self.shift
        corner_offsets = self.cell_dims * self._corner_tf

        return origin[...,None,:] +  corner_offsets # Expand to (..., 4, 2)

    def centers(self, idx):
        "Return list of x,y coordinates of the centers of the cells at index i,j"
         # (i,j) => (j,i) => (x,y)
        origin = idx[...,::-1] * self.cell_dims + self.shift
        center_offset = self.cell_dims * [0.5,0.5]

        return origin + center_offset


    def surrounding(self, coords, dist):
        "Return list of row,col indices to cells surrounding a point up to dist in any direction"
        surrounding_tf = self._corner_tf * 2 - 1
        corner_offsets = dist * surrounding_tf
        corners = coords[...,None,:] + corner_offsets # Expand to (..., 4, 2)
        
        corner_idx = self.index(corners)
        min_idx = np.min(corner_idx, axis=-2) # Collapse to
        max_idx = np.max(corner_idx, axis=-2) # (..., 2)
        diff_idx = max_idx - min_idx
        
        i = np.linspace(min_idx[...,0], max_idx[...,0], np.max(diff_idx[...,0]), axis=-1, dtype=int)
        j = np.linspace(min_idx[...,1], max_idx[...,1], np.max(diff_idx[...,1]), axis=-1, dtype=int)
        iv, jv = np.meshgrid(i, j)

        return np.stack([iv, jv], axis=-1)

    def enveloped(self, idx, origin, radius):
        "Return arrays of True/False if cells are fully enveloped inside radius from origin."
        corners = self.corners(idx) # Returns (..., 4, 2)
        corner_distances = np.linalg.norm(corners - origin, axis=-1) # Collapses to (..., 4)
        max_distances = np.max(corner_distances, axis=-1) # Collapses to (...)
        return max_distances < radius

    def obscured(self, obstacle_coords, origin_coord, max_dist):
        "Return all cells obscured by obstacles at a coordinates up to a max distance."

        # Find start and end coordinates
        start = obstacle_coords
        origin = origin_coord
        short_diff = start - origin
        short_dist = np.linalg.norm(short_diff,axis=-1)
        long_diff = short_diff * (max_dist / short_dist)[:,None]
        end = origin + long_diff
        
        # Find start and end cells
        start_cell = self.index(start)
        end_cell = self.index(end)
        cell_diff = end_cell - start_cell
        cell_diff_max = np.max(np.abs(cell_diff))

        # Get all edge crossing of outgoing ray
        unit = np.sign(cell_diff)
        cells = np.linspace(start_cell + unit, end_cell, cell_diff_max, dtype=int, axis=1)
        corners = self.corners(cells)

        # Find the corner index representing the edge boundaries crossed 
        angle = np.arctan2(short_diff[...,1], short_diff[...,0])
        quadrant = (np.floor(angle / (np.pi / 2)).astype(int) + 4) % 4
        corner_idx = np.choose(quadrant, [0, 2, 3, 1])

        # Apply corner index to take only respsective edge crossings
        corner = np.take_along_axis(corners, corner_idx[...,None,None,None], axis=-2)[...,0,:]

        # Split into vertical and horizontal edge crossings
        x, y = np.moveaxis(corner, -1, 0)

        # Find coordinates of each edge crossing
        y_edge = x / short_diff[...,0,None] * short_diff[...,1,None]
        x_edge = y / short_diff[...,1,None] * short_diff[...,0,None]
        vert = np.stack((x, y_edge), axis=-1)
        horiz = np.stack((x_edge, y), axis=-1)

        # Get cell centers of the cell belonging to the edge just crossed into
        half_cell = np.sign(short_diff) * self.cell_dims / 2
        choose_x = [1,0]
        choose_y = [0,1]
        vert_centers = vert + (half_cell * choose_x)[...,None,:]
        horiz_centers = horiz + (half_cell * choose_y)[...,None,:]

        # Convert cell centers to indices and combine
        vert_cells = self.index(vert_centers)
        horiz_cells = self.index(horiz_centers)
        obscured_cells = np.concatenate([vert_cells, horiz_cells], axis=-2)

        return obscured_cells

    def validate_entry(self, idx):
        "Check if the indices are within the matrix and expand if necessary, returning new indices."
        if not idx.size:
            return idx # No elements
        
        min_idx = np.min(idx, axis=-2)
        max_idx = np.max(idx, axis=-2)
        transform = np.zeros_like(self.matrix.shape)

        while (min_idx + transform)[0] < 0:
            # Expand up
            transform[0] += self.matrix.shape[0]
            g = np.zeros_like(self.matrix)
            self.matrix = np.concatenate([g, self.matrix], axis=0)
        while (max_idx + transform)[0] >= self.matrix.shape[0]:
            # Expand down
            g = np.zeros_like(self.matrix)
            self.matrix = np.concatenate([self.matrix, g], axis=0)
        while (min_idx + transform)[1] < 0:
            # Expand left
            transform[1] += self.matrix.shape[1]
            g = np.zeros_like(self.matrix)
            self.matrix = np.concatenate([g, self.matrix], axis=1)
        while (max_idx + transform)[1] >= self.matrix.shape[1]:
            # Expand right
            g = np.zeros_like(self.matrix)
            self.matrix = np.concatenate([self.matrix, g], axis=1)

        # Update shift (i,j) => (j,i) => (x,y)
        self.shift -= transform[::-1] * self.cell_dims

        # Update indices
        idx += transform

        return idx

    def __getitem__(self, key):
        idx = self.index(key) # (x,y) => (i,j)

        # Find invalid entries
        invalid = np.max((idx < 0) | (idx >= self.matrix.shape), axis=-1)

        # Zero out the invalid indices
        idx[invalid] = (0, 0)

        return np.where(invalid, self._default_val, self.matrix[idx[...,0], idx[...,1]])

    def __setitem__(self, key, value):
        idx = self.index(key) # (x,y) => (i,j)
        idx = self.validate_entry(idx)
        self.matrix[idx[...,0], idx[...,1]] = value

    def extents(self):
        "Returns [[xmin, xmax], [ymin, ymax]]"
        start = self.shift
        end = self.matrix.shape[::-1] * self.cell_dims + self.shift
        corners = np.stack([start, end])
        botleft = np.min(corners, axis=0)
        topright = np.max(corners, axis=0)
        extent = np.stack([botleft, topright],axis=1)
        return extent

    def plot(self, grid=True, **kwargs):
        "Shows a pyplot of the grid"
        fig, ax = plt.subplots()
        ax.imshow(self.matrix, extent=list(self.extents().flat), **kwargs)
        ax.xaxis.set_minor_locator(plt.MultipleLocator(abs(self.cell_dims[0])))
        ax.yaxis.set_minor_locator(plt.MultipleLocator(abs(self.cell_dims[1])))
        if grid:
            ax.grid(True,which='both',linestyle='-',linewidth=1)


### Scandata Parsing ###

def parse_scandata(msg, low='none', high='none'):
    """Parses scandata from raw LaserScan message.
    
    Options for out-of-range value evaluations are:
    'none': Do nothing
    'remove': Remove the value entries
    'clamp': Replace the distance value with the respective limit value

    :param low: How to evaluate underrange values.
    :param high: How to evaluate overrange values.
    """
    # Convert from radians to degrees
    start_deg = msg.angle_min * 180. / math.pi
    end_deg = msg.angle_max * 180. / math.pi

    # Extract data to x (degrees) and y (meters)
    x = np.linspace(start_deg, end_deg, len(msg.ranges))
    y = np.array(msg.ranges)
    data = np.stack([x,y])

    # Evaluate upper end of data
    if high == 'clamp':
        data[1, np.isinf(data[1,:])] = msg.range_max
    elif high == 'remove':
        data = data[:, ~np.isinf(data[1,:])]

    # Evaluate lower end of data
    if low != 'none':
        data_shift_left = np.roll(data, -1, axis=1)
        data_shift_right = np.roll(data, 1, axis=1)

        close = data[1,:] < msg.range_min * 2
        jump_right = data_shift_left[1,:] / data[1,:] > 2
        jump_left = data_shift_right[1,:] / data[1,:] > 2

        edge_left = close & jump_right
        edge_right = close & jump_left

        remove = np.repeat(False, len(edge_left))
        removing = False
        for i in range(len(remove)):
            if edge_right[i] or close[i] or data[1,i] > msg.range_min * 8:
                removing = False
            remove[i] = removing
            if edge_left[i]:
                removing = True

        if removing:
            for i in range(len(remove)):
                if edge_right[i] or close[i] or data[1,i] > msg.range_min * 8:
                    break
                remove[i] = True
    
        if low == 'clamp':
            data[1, remove] = msg.range_min
        elif low == 'remove':
            data = data[:, ~remove]

    return data

def find_min(data, angle_deg, collapse_range_deg=80):
    "Find the local minima in the data closest to a specified angle"

    # Error tolerance
    tolerance = {'rtol': 0.02, 'atol': 0.002}
    
    # Calculate angles
    angle_start = (angle_deg - collapse_range_deg/2) % 360
    angle_end = (angle_deg + collapse_range_deg/2) % 360

    # Convert angles to indices
    idx_start, idx_end = np.searchsorted(data[0,:], (angle_start, angle_end))

    # Evaluate range with modular indexing
    collapse_range = data[:,idx_start:idx_end] if idx_start <= idx_end \
        else np.concatenate([data[:,idx_start:None],data[:,None:idx_end]],axis=1)

    # Try to collapse to a minimum within the range
    idx_min = np.argmin(collapse_range[1,:])
    val_min = collapse_range[1,idx_min]

    # If there's multiple minimums, average out (important for underrange)
    data_min_idx = np.isclose(collapse_range[1,:], val_min, **tolerance)
    if np.count_nonzero(data_min_idx) > 1:
        #print("Averaging!")
        averaging_range = collapse_range[:,data_min_idx]
        #print(averaging_range[0])

        # Establish semicircle basis for averaging
        first_angle = averaging_range[0,0]
        last_angle = averaging_range[0,-1]
        range_angle = angle_diff(last_angle, first_angle)
        mid_angle = (first_angle + range_angle/2) % 360
        averaging_range[0] = np.mod(averaging_range[0] - mid_angle + 180, 360) + mid_angle - 180
        #print(averaging_range[0])

        return tuple(np.average(averaging_range,axis=1))

    step = 0
    # Is our minumum on the left edge?
    if np.isclose(val_min, data[1,idx_start], **tolerance):
        idx_min = idx_start # Translate to global index
        step = -1 # Walk left
    # Is our minumum on the right edge?
    elif np.isclose(val_min, data[1,idx_end], **tolerance):
        idx_min = idx_end # Translate to global index
        step = 1 # Walk right
    else:
        # No edge minimum, just return our collapsed minimum
        return tuple(collapse_range[:,idx_min])

    # Initialize one step forward
    idx = (idx_min + step) % data.shape[1]

    # Keep going until we loop back around
    idx_start = idx_min
    while idx != idx_start:
        # Walk
        idx = (idx + step) % data.shape[1]
        val = data[1,idx]

        # Check slope
        if val < val_min:
            val_min = val
            idx_min = idx
        elif not np.isclose(val, val_min, **tolerance):
            # Going back up, we're done here
            break
    
    # Return x,y
    return tuple(data[:,idx_min])

def find_obstacle(data, angle_deg, clearance_width, max=math.inf):
    x = data[1] * np.sin((data[0] - angle_deg) / 180. * math.pi)
    y = data[1] * np.cos((data[0] - angle_deg) / 180. * math.pi)
    return np.min(y[(abs(x) < clearance_width/2) & (y > 0)], initial=max)

def find_opening(data, angle_deg, dist_thresh, dir='pos'):
    "Finds the first wall opening in the laser scan."
    x = data[1] * np.sin((data[0] - angle_deg) / 180. * math.pi)
    y = data[1] * np.cos((data[0] - angle_deg) / 180. * math.pi)
    xy = np.stack([x,y])
    coeff = 1 if dir == 'pos' else -1
    openings = xy[:, (x >= 0) & (y >= dist_thresh)]
    if not openings.any():
        return None, None
    first_opening = np.argmin(openings[0])
    return tuple(openings[:, first_opening])

def find_first(condition, start_index=0, dir='right'):
    if np.alltrue(~condition):
        return None # Not found
    if dir == 'right':
        shift = -start_index
        coeff = 1
    elif dir == 'left':
        condition = condition[::-1]
        shift = start_index + 1
        coeff = -1
    else:
        raise ValueError("'dir' must be either 'left' or 'right'.")

    rolled = np.roll(condition, shift)
    res_rel = np.argmax(rolled)
    return (start_index + coeff * res_rel) % len(condition)

def get_wall_width(data, angle_deg):
    "Finds the width of the wall found at a certain angle."
    idx_wall = np.searchsorted(data[0], angle_deg)
    x = data[1] * np.sin((data[0] - angle_deg) / 180. * math.pi)
    y = data[1] * np.cos((data[0] - angle_deg) / 180. * math.pi)
    dist_wall = y[idx_wall]
    filter_wall = (y < 2*dist_wall) & (y > 0)

    # Find left
    idx_left = find_first(~filter_wall, idx_wall, dir='left')
    if idx_left == None:
        return math.inf

    # Find right
    idx_right = find_first(~filter_wall, idx_wall, dir='right')
    if idx_right == None:
        return math.inf
    
    # Roll to start
    rolled = np.roll(x, -idx_left)
    width = idx_right - idx_left
    wall_range = rolled[:width]

    return np.max(wall_range) - np.min(wall_range)


### Odometry Parsing ###

def parse_odom(msg):
    "Parses a nav_msgs.Odometry message to position and angles"
    pose = msg.pose.pose

    pos = pose.position
    pos_np = np.array([pos.x, pos.y, pos.z])

    quat = pose.orientation
    angles = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    angles_deg = np.array(angles) / math.pi * 180.

    return pos_np, angles_deg


### Math Utilities ###

# UNITS: Radians
#FULLTURN = 2 * math.pi
#HALFTURN = math.pi

# UNITS: Degrees
FULLTURN = 360
HALFTURN = 180

def angle_dist(a, b):
    "Calculates the acute distance between two angles in degrees."
    dist = (a - b) % FULLTURN
    return dist if dist <= HALFTURN else FULLTURN - dist

def angle_diff(a, b):
    "Calculates the acute difference between two angles in degrees."
    dist = (a - b) % FULLTURN
    return dist if dist <= HALFTURN else dist - FULLTURN

def decelerate(dist, stopping_dist, max_speed):
    "Calculates the decelration necessary to stop in a certain distance from a maximum velocity."
    ratio = dist / stopping_dist
    speed = np.sqrt(np.abs(dist / stopping_dist)) * max_speed
    return np.sign(ratio) * min(speed, max_speed)

# https://stackoverflow.com/a/11903368
def setdiff2d(a1, a2):
    "Takes the set difference of two 2D arrays along the first dimension."
    a1_rows = a1.view([('', a1.dtype)] * a1.shape[1])
    a2_rows = a2.view([('', a2.dtype)] * a2.shape[1])
    return np.setdiff1d(a1_rows, a2_rows).view(a1.dtype).reshape(-1, a1.shape[1])


### Control Logic ###

# Parameters
FRONT_COLLAPSE_ANGLE_DEG = 40
FRONT_VALID_ANGLE_DEG = 25
LEFT_VALID_ANGLE_DEG = 40
BOT_WIDTH_CLEARANCE = 0.2
LEFT_WALL_DIST_MIN = 0.2
LEFT_WALL_DIST_MAX = 0.5
LEFT_WALL_ANGLE_DEG_MAX = 10
LEFT_WALL_OPENING_DIST_THRESH = 0.5
OPENING_MIN_DRIVE = 0.25
LEFT_WALL_WIDTH_MIN = 1.0
#LEFT_WALL_WIDTH_MIN = 0.5
WALL_APPROAHCH_ANGLE_DEG = 15
UNSTABLE_ANGLE_DEG = 10
OBSTACLE_CLEARANCE = 0.3
OBSTACLE_TURN_DISTANCE = 0.6
BACKUP_DISTANCE = 0.2

MAX_VELOCITY_LINEAR = 0.5 # m/s
STOPPING_DISTANCE_LINEAR = 1.0 # m
"Distance to stop moving forward from max speed"

MAX_VELOCITY_ANGULAR = 1.0 # rad/s
STOPPING_DISTANCE_ANGULAR = 0.35 # rad
"Distance to stop turning from max speed"

CONTROL_RATE = 20 # Hz
STUCK_CHECK_DELAY = 8 # s
STUCK_FREE_TIME = 0.75 # s
STUCK_CHECK_DISTANCE = 0.25 # m

GRID_SIZE_DEFAULT = 10. # m
GRID_RESOLUTION_DEFAULT = 0.1 # m
GRID_WEIGHT_LIMIT = 15

class Driver(object):

    def __init__(self, init_x=0., init_y=0., size=GRID_SIZE_DEFAULT, resolution=GRID_RESOLUTION_DEFAULT, no_drive=False):
        self.pose_initialized = False
        self.scan_initialized = False
        self.goal_dist = 0.
        self.goal_turn = 0.
        self.distance_traveled = 0.
        self.no_drive = no_drive

        # Initialize grid
        self.grid = Grid(dtype=np.int8, 
                         init_shift_x=init_x - size, 
                         init_shift_y=init_y - size, 
                         cell_width=resolution,
                         cell_height=resolution,
                         init_cols=int((2*size) / resolution),
                         init_rows=int((2*size) / resolution))

    def register(self):
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.on_shutdown(self.shutdown)

    def main_loop(self):
        rate = rospy.Rate(CONTROL_RATE)

        stuck_check_time = rospy.Time.now() + rospy.Duration(STUCK_CHECK_DELAY)
        stuck_check_odom = 0
        stuck = False
        stuck_dir = 1

        self.running = True

        while not rospy.is_shutdown():
            if self.running:
                if rospy.Time.now() > stuck_check_time:
                    # Check if stuck
                    stuck = self.distance_traveled - stuck_check_odom < STUCK_CHECK_DISTANCE

                    if stuck:
                        print("We're stuck! Freeing ourselves.")
                        stuck_dir = -stuck_dir # Alternate direction
                        stuck_check_time = rospy.Time.now() + rospy.Duration(STUCK_FREE_TIME)
                    else:
                        stuck_check_time = rospy.Time.now() + rospy.Duration(STUCK_CHECK_DELAY)

                    # Reset stuck check
                    stuck_check_odom = self.distance_traveled

                twist = Twist()
                if stuck:
                    # We're stuck, try to free ourselves
                    twist.linear.x = stuck_dir
                    twist.angular.z = 1
                else:
                    self.exec_drive(twist)
                    self.exec_turn(twist)
                    
                if not self.no_drive:
                    self.cmd_vel_pub.publish(twist)

            rate.sleep()

    def stop(self):
        self.running = False
        self.cmd_vel_pub.publish(Twist())

    ### Setup goals to be executed in the main loop ###
    def drive(self, distance, reverse=False):
        if not self.pose_initialized:
            return # Not initialized!

        self.state_reverse = reverse
        self.goal_dist = self.distance_traveled + distance

        if reverse:
            print(f"GOAL: Reversing {distance:.2f}m.")
        else:
            print(f"GOAL: Driving {distance:.2f}m.")

    def turn(self, angle):
        if not self.pose_initialized:
            return # Not initialized!

        self.goal_turn = (self.yaw + angle) % 360
        print(f"GOAL: Turning {angle:.0f}deg to {self.goal_turn:.0f}deg.")

    ### Execute on our goals through a Twist message ###
    def exec_drive(self, twist):
        if not self.pose_initialized:
            return # Not initialized!

        if self.goal_dist > self.distance_traveled:
            # We want to go forward
            distance_left = self.goal_dist - self.distance_traveled
            twist.linear.x = decelerate(distance_left, STOPPING_DISTANCE_LINEAR, MAX_VELOCITY_LINEAR)
            if self.state_reverse:
                twist.linear.x = -twist.linear.x

    def exec_turn(self, twist):
        if not self.pose_initialized:
            return # Not initialized!

        angle_left_rad = angle_diff(self.goal_turn, self.yaw) / 180. * math.pi
        twist.angular.z = decelerate(angle_left_rad, STOPPING_DISTANCE_ANGULAR, MAX_VELOCITY_ANGULAR)

    ### Receive and parse callbacks from turtlebot ###
    def odom_callback(self, msg):
        if not self.running:
            return

        pos, rot = parse_odom(msg)
        if self.pose_initialized:
            self.distance_traveled += np.linalg.norm(pos - self.pos)
        self.pos = pos
        self.yaw = rot[2]
        self.pose_initialized = True

    def scan_callback(self, msg):
        if not self.running:
            return

        print("-- Scan callback --")
        self.update_grid(msg)
        data = parse_scandata(msg, low='clamp', high='clamp')

        # Find wall to left
        forward_angle = 0.
        left_angle, left_dist = find_min(data, 90)
        left_valid = angle_dist(left_angle, 90) < LEFT_VALID_ANGLE_DEG

        # Check wall width
        if left_valid:
            wall_width = get_wall_width(data, left_angle)
            if wall_width < LEFT_WALL_WIDTH_MIN:
                left_valid = False # Must be a legpost or smth

        # Stabilize next to left wall
        unstable = False
        if left_valid:
            print(f"Found left wall to {left_angle:.0f}deg.")
            forward_angle = left_angle - 90 # infer forward direction
            print(f"Inferred forward to {forward_angle:.0f}deg.")

            # Priority #1: Get within distance range
            if left_dist < LEFT_WALL_DIST_MIN:
                print("Turning away from wall.")
                self.turn(forward_angle - WALL_APPROAHCH_ANGLE_DEG) # Turn right from wall

                if angle_diff(forward_angle, 0) < -UNSTABLE_ANGLE_DEG:
                    # We're turning the wrong way, slow down!
                    unstable = True

            elif left_dist > LEFT_WALL_DIST_MAX:
                print("Turning toward wall.")
                self.turn(forward_angle + WALL_APPROAHCH_ANGLE_DEG) # Turn left toward wall

                if angle_diff(forward_angle, 0) > UNSTABLE_ANGLE_DEG:
                    # We're turning the wrong way, slow down!
                    unstable = True

            # Priority #2: Straighten up parallel to wall
            else:
                print("Straightening up by wall.")
                self.turn(forward_angle) # Straighten up

        # Find wall to front
        front_angle, front_dist = find_min(data, forward_angle, FRONT_COLLAPSE_ANGLE_DEG)
        front_valid = angle_dist(front_angle, forward_angle) < FRONT_VALID_ANGLE_DEG

        # Find obstacle to front
        obs_dist = find_obstacle(data, forward_angle, BOT_WIDTH_CLEARANCE, msg.range_max)

        # Check if there's an opening in the left wall coming up
        if left_valid and left_dist < LEFT_WALL_DIST_MAX:
            opening_dist, _ = find_opening(data, left_angle, LEFT_WALL_OPENING_DIST_THRESH)
            if opening_dist:
                print(f"Found opening in wall {opening_dist:.2f}m away.")
                opening_obs_dist = opening_dist + LEFT_WALL_DIST_MIN
                if opening_obs_dist - OBSTACLE_CLEARANCE < OPENING_MIN_DRIVE:
                    opening_obs_dist = OBSTACLE_CLEARANCE + OPENING_MIN_DRIVE
                if opening_obs_dist < obs_dist:
                    obs_dist = opening_obs_dist

        # Avoid obstacles from the front
        if unstable:
            print("Unstable, slowing down.")
            self.drive(0)
        elif obs_dist < BACKUP_DISTANCE:
            # Too close too wall, just back up and don't turn
            print(f"Backing up from wall {obs_dist:.2f}m away at {forward_angle:.0f}deg.")
            self.drive(BACKUP_DISTANCE - obs_dist, reverse=True)
        elif obs_dist < msg.range_max:
            # Drive up to and stop in front of obstacle
            print(f"Driving up to obstacle in {obs_dist:.2f}m at {forward_angle:.0f}deg.")
            self.drive(obs_dist - OBSTACLE_CLEARANCE)
        else:
            # Otherwise just drive forward, one meter at a time!
            print("Driving forward.")
            self.drive(STOPPING_DISTANCE_LINEAR + 1.0)

        # Turn into corner
        if front_valid and front_dist < OBSTACLE_TURN_DISTANCE:
            # Override priority of wall stabilization with turning into corner
            print(f"Turning into corner at {front_angle:.0f}deg.")
            self.turn(front_angle - 90) # Turn right

        print()

    ### Execute on information received ###
    def update_grid(self, laser_scan):
        data = parse_scandata(laser_scan, low='clamp', high='remove')
        angles = (data[0] + 90 + self.yaw) / 180. * math.pi
        x = data[1] * np.cos(angles)
        y = data[1] * np.sin(angles)
        coords = self.pos[:2] + np.stack([x,y], axis=-1)

        # Mark all points' cells as occupied (-1)
        unknown_cond = self.grid[coords] > -GRID_WEIGHT_LIMIT
        self.grid[coords[unknown_cond]] -= 1

        # Find rest of cells
        occupied_cells = self.grid.index(coords)
        obscured_cells = self.grid.obscured(coords, self.pos[:2], laser_scan.range_max).reshape(-1,2)
        surrounding_cells = self.grid.surrounding(self.pos[:2], laser_scan.range_max)
        enveloped_cond = self.grid.enveloped(surrounding_cells, self.pos[:2], laser_scan.range_max)
        enveloped_cells = surrounding_cells[enveloped_cond]
        open_cells = setdiff2d(enveloped_cells, np.concatenate([occupied_cells,obscured_cells]))

        # Mark all open cells as open (1)
        open_cell_centers = self.grid.centers(open_cells)
        unknown_cond = self.grid[open_cell_centers] < GRID_WEIGHT_LIMIT
        self.grid[open_cell_centers[unknown_cond]] += 1


    def shutdown(self):
		# Gracefully shut down all of our windows/processes
        
        self.stop()
        print("Shutting down the wander node...")

        try:
            print("Grid Size:", self.grid.matrix.shape)
            print("Grid Cells:", self.grid.matrix.size)
            print("Grid Discovered:", np.count_nonzero(self.grid.matrix))
            print("Obstacles Discovered:", np.count_nonzero(self.grid.matrix == -1))

            self.grid.plot(grid=False)
            self.grid.plot(grid=True)
            plt.show()

        except:
            e = sys.exc_info()[1]
            print("There was a problem.")
            print(e)

        return

def parse_args():
    kwargs = {}

    if rospy.has_param('~resolution'):
        kwargs['resolution'] = rospy.get_param('~resolution')
    else:
        rospy.logwarn(f"PARAMS: Resolution not provided; using {GRID_RESOLUTION_DEFAULT:.1f}")

    if rospy.has_param('~size'):
        kwargs['size'] = rospy.get_param('~size')
    else:
        rospy.logwarn(f"PARAMS: Size not provided; using {GRID_SIZE_DEFAULT:.1f}")

    if rospy.has_param('~init_x'):
        kwargs['init_x'] = rospy.get_param('~init_x')
    else:
        rospy.logwarn(f"PARAMS: Initial X not provided; using X=0")

    if rospy.has_param('~init_y'):
        kwargs['init_y'] = rospy.get_param('~init_y')
    else:
        rospy.logwarn(f"PARAMS: Initial Y not provided; using Y=0")

    if rospy.has_param('~no_drive'):
        rospy.loginfo(f"PARAMS: Driving disabled.")
        kwargs['no_drive'] = rospy.get_param('~no_drive')

    return kwargs

def main():
    rospy.init_node('wander_amit', anonymous=True)
    kwargs = parse_args()
    driver = Driver(**kwargs)
    driver.register()
    driver.main_loop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("wander node terminated")
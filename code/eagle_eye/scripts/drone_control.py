#!/usr/bin/env python3
import sys
import rospy
from rospy.impl.tcpros import get_tcpros_handler
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
import clover.srv

import cv2, numpy as np
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation

### Probabilistic Masking (Null Hypothesis test) ###
import scipy.stats


def pval(dist, points):
    "Returns the p-value of points on a symmetric distribution."
    cumul = dist.cdf(points)
    return 2*np.minimum(cumul, 1-cumul)


def pval_lower(dist, points):
    "Returns the lower p-value of points on a distribution."
    return dist.cdf(points)


def pval_upper(dist, points):
    "Returns the upper p-value of points on a distribution."
    return dist.sf(points)


def recenter_hue(hue, mid_hue):
    return np.mod(hue - mid_hue + 90, 180) + mid_hue - 90


RED_FILTER = {
    'hue_mean': 0,
    'hue_std': 5,
    'sat_offs': 0,
    'sat_std': -5,
    'val_offs': 20,
    'val_std': -50,
}


class Filter:
    def __init__(self, hue_mean=0, hue_std=5, sat_offs=0, sat_std=5, val_offs=0, val_std=5):
        self.hue_dist = scipy.stats.norm(hue_mean, hue_std)
        self.sat_dist = scipy.stats.expon(sat_offs, np.abs(sat_std))
        self.val_dist = scipy.stats.expon(val_offs, np.abs(val_std))
        self.sat_inv = sat_std < 0
        self.val_inv = val_std < 0

    def apply_hue(self, hue):
        return pval(self.hue_dist, recenter_hue(hue, self.hue_dist.mean()))

    def apply_sat(self, sat):
        return pval_upper(self.sat_dist, 255 - sat if self.sat_inv else sat)

    def apply_val(self, val):
        return pval_upper(self.val_dist, 255 - val if self.val_inv else val)

    def apply(self, hsv):
        hue, sat, val = np.moveaxis(hsv.astype(float), -1, 0)
        pval_hue = self.apply_hue(hue)
        pval_sat = self.apply_sat(sat)
        pval_val = self.apply_val(val)
        return pval_hue * pval_sat * pval_val

### Coordinate Transformations ###

WIDTH = 320
HEIGHT = 240

def update_img_dims(height, width):
    HEIGHT, WIDTH = height, width

def idx_to_xy(idx):
    return idx[...,::-1] * (1,-1) + (0, HEIGHT)

def xy_to_idx(xy):
    return ((xy - (0, HEIGHT)) * (1,-1))[...,::-1]
    
def to_cv(idx):
    return np.round(idx[...,::-1]).astype(int).clip(-9999, 9999)

def turn_left(xy):
    x, y = np.moveaxis(xy, -1 , 0)
    return np.stack([-y, x], axis=-1)

def to_clover(xy):
    x, y = np.moveaxis(xy, -1 , 0)
    return np.stack([y, -x], axis=-1)

### Computer Vision ###

RESOLUTION = 100 # pixels / meter

# https://docs.opencv.org/3.4/dc/dbb/tutorial_py_calibration.html
def undistort_plumb_bob(camera_matrix, distortion, image_size, alpha=1):
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion, image_size, alpha, image_size)
    def undistort(img):
        img_dst = cv2.undistort(img, camera_matrix, distortion, None, new_camera_matrix)
        #x, y, w, h = roi
        #img_dst = img_dst[y:y+h, x:x+w]
        return img_dst
    return undistort

def parse_telemetry(msg):
    rotation = np.array([msg.pitch, msg.roll, msg.yaw])
    translation = np.array([msg.x, msg.y, msg.z])
    velocity = np.array([msg.vx, msg.vy, msg.vz])
    return rotation, translation, velocity

# https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html
def homogeneous_gnd2cam(R_euler, T_vec, K):
    rotation = Rotation.from_euler('xyz', R_euler)
    R = rotation.as_matrix()
    T = np.concatenate([np.eye(3), T_vec[:,None]], axis=-1)
    M = R @ T
    H = K @ M
    return H

project_hom = np.array(
    [[1,0,0],
     [0,1,0],
     [0,0,0],
     [0,0,1]])
def homogeneous_cam2gnd(R_euler, T_vec, K_inv):
    rotation = Rotation.from_euler('xyz', R_euler)
    R = rotation.as_matrix()
    R_inv = R.T
    T_vec_inv = -R_inv @ T_vec
    M_inv = np.concatenate([R_inv, T_vec_inv[:,None]], axis=-1)
    H_inv = M_inv @ project_hom @ K_inv
    return H_inv

def homogeneous_translation(t_vec):
    T = np.eye(len(t_vec)+1)
    T[:len(t_vec),-1] = t_vec
    return T

corners_coeff = np.array(
    [[0,0],
     [0,1],
     [1,0],
     [1,1]]
)
def roi(image_size, H):
    "Returns bounding box after an image of some size goes through a homogenous transformation"
    corners = corners_coeff * image_size
    corners_hom = cv2.convertPointsToHomogeneous(corners)[:,0,:]
    corners_tf_hom = H @ corners_hom.T
    corners_tf = cv2.convertPointsFromHomogeneous(corners_tf_hom.T)[:,0,:]
    min_pt = np.min(corners_tf, axis=0)
    max_pt = np.max(corners_tf, axis=0)
    origin = min_pt
    size = max_pt - min_pt
    return origin, size

def project_below(img, R_euler, T_vec, K_inv):
    # Create homogeneous transformation from camera to ground
    T_vec_centered = np.zeros_like(T_vec)
    T_vec_centered[2] = -T_vec[2]
    H = homogeneous_cam2gnd(R_euler, T_vec_centered, K_inv)

    # Scale from meters to pixels
    scale_matrix = np.eye(3) * (RESOLUTION, RESOLUTION, 1)
    H_scaled = scale_matrix @ H

    # Apply bounding box but retain center
    min_pt, size = roi(img.shape[:2][::-1], H_scaled)
    max_pt = min_pt + size
    max_dist = np.max(np.abs(np.stack([min_pt, max_pt])),axis=0)
    out_size = np.ceil(max_dist * 2).astype(int)
    T_center = homogeneous_translation(out_size / 2)
    H_scaled_centered = T_center @ H_scaled

    # Apply transformation
    img_below = cv2.warpPerspective(img, H_scaled_centered, out_size)
    return img_below

### Control Logic ###

# Parameters
CONTROL_RATE = 5 # Hz
SPEED_TAKEOFF = 0.5 # m/s
SPEED_FLY = 1.0 # m/s
FLIGHT_HEIGHT = 1.5 # m
TARGET_FLOW_DIST = 1.0 # m
SPEED_STABILITY = 0.1 # m/s
DIST_KEEPOUT_TARGET = 0.75 # m
DIST_KEEPOUT_UNSTABLE = 0.1 # m

class Driver(object):

    def __init__(self):
        self.running = False
        self.state = 'take_off'
        self.img_msg = None
        self.undistort = lambda img: img
        self.camera_initialized = False
        self.bridge = CvBridge()
        self.filter_red = Filter(**RED_FILTER)

    def register(self):
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', clover.srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', clover.srv.Navigate)
        self.navigate_global = rospy.ServiceProxy('navigate_global', clover.srv.NavigateGlobal)
        self.set_position = rospy.ServiceProxy('set_position', clover.srv.SetPosition)
        self.set_velocity = rospy.ServiceProxy('set_velocity', clover.srv.SetVelocity)
        self.set_attitude = rospy.ServiceProxy('set_attitude', clover.srv.SetAttitude)
        self.set_rates = rospy.ServiceProxy('set_rates', clover.srv.SetRates)
        self.land = rospy.ServiceProxy('land', Trigger)

        self.image_sub = rospy.Subscriber('/main_camera/image_raw', 
                                          Image, self.image_callback, queue_size=1)

        self.caminfo_sub = rospy.Subscriber('/main_camera/camera_info', 
                                          CameraInfo, self.caminfo_callback)

        rospy.on_shutdown(self.shutdown)

    def main_loop(self):
        rate = rospy.Rate(CONTROL_RATE)

        self.running = True

        while not rospy.is_shutdown():
            if self.running:
                self.branch_state()

            rate.sleep()

    ### State Machine ###

    def branch_state(self):
        print("State:", self.state)
        if self.state == 'take_off':
            self.state_take_off()
        elif self.state == 'taking_off':
            self.state_taking_off()
        elif self.state == 'find_edge':
            self.state_find_edge()

    def state_take_off(self):
        telemetry = self.get_telemetry()
        success = self.navigate(z=FLIGHT_HEIGHT - telemetry.z, speed=SPEED_TAKEOFF, frame_id='body', auto_arm=True)
        if success:
            self.state = 'taking_off'

    def state_taking_off(self):
        telemetry = self.get_telemetry()
        if np.abs(telemetry.z - FLIGHT_HEIGHT) < 0.1 and np.abs(telemetry.vz) < 0.1:
            self.state = 'find_edge'

    def state_find_edge(self):
        if self.img_msg is None:
            return
        img = self.bridge.imgmsg_to_cv2(self.img_msg,desired_encoding='bgr8')

        # Process image
        img = self.undistort(img)
        img_below = project_below(img, self.rotation, self.translation, self.K_inv)

        # Filter image for boundary
        hsv = cv2.cvtColor(img_below, cv2.COLOR_BGR2HSV)
        filter = self.filter_red.apply(hsv)
        filter_img = (filter * 255).astype(np.uint8)
        edges = cv2.Canny(filter_img, 20, 240)

        edges_idx = np.moveaxis(np.indices(edges.shape),0,-1)[edges>127].reshape(-1,2)
        edges_xy = idx_to_xy(edges_idx)

        if edges_xy.shape[0] < 10:
            # No boundary found
            if np.linalg.norm(self.velocity) <= SPEED_STABILITY:
                # Not moving, just go forward
                self.set_velocity(vx=SPEED_FLY, frame_id='body')
            cv2.imshow("Camera", img)
            cv2.imshow("Below", img_below)
            cv2.waitKey(3)
            return

        # Draw edge points
        img_edges = np.zeros_like(img_below)
        img_edges[tuple(xy_to_idx(edges_xy).astype(int).clip((0,0),(HEIGHT-1,WIDTH-1)).T)] = (0, 0, 255)

        # Find closest point
        center_idx = np.array(img_below.shape[:2])/2
        center_xy = idx_to_xy(center_idx)

        closest_pt_xy = edges_xy[np.argmin(np.linalg.norm(edges_xy - center_xy, axis=-1))] 
        closest_pt_idx = xy_to_idx(closest_pt_xy)

        # Draw line to closest point
        cv2.line(img_below, to_cv(center_idx), to_cv(closest_pt_idx), (0,255,255), 2)
        cv2.line(img_edges, to_cv(center_idx), to_cv(closest_pt_idx), (0,255,255), 1)

        # Control drone
        vector = closest_pt_xy - center_xy
        dist = np.linalg.norm(vector) / RESOLUTION

        # If too close to edge, wait to stabilize
        if dist > DIST_KEEPOUT_UNSTABLE:
            dir = turn_left(vector)
            offset = vector / np.linalg.norm(vector) * (dist - DIST_KEEPOUT_TARGET)
            dir_phy = self.direct(dir, offset)

            # Draw control line
            control_phy = dir_phy + offset
            control_pt_xy = center_xy + control_phy * RESOLUTION
            control_pt_idx = xy_to_idx(control_pt_xy)

            cv2.line(img_below, to_cv(center_idx), to_cv(control_pt_idx), (0,255,0), 2)
            cv2.line(img_edges, to_cv(center_idx), to_cv(control_pt_idx), (0,255,0), 1)

        # Show imagery
        cv2.imshow("Camera", img)
        cv2.imshow("Below", img_below)
        cv2.imshow("Parsed", img_edges)
        cv2.waitKey(3)
        

    def stop(self):
        self.running = False
        self.land()

    ### Control the drone's flight path ###

    def direct(self, direction, offset=(0,0)):
        dir_unit = direction / np.linalg.norm(direction)
        vector = TARGET_FLOW_DIST * dir_unit
        dx, dy = to_clover(vector + offset)
        self.navigate(x=dx, y=dy, speed=SPEED_FLY, frame_id='body')
        return vector

    ### Receive and parse callbacks from the drone ###

    def image_callback(self, msg):
        self.img_msg = msg
        telemetry = self.get_telemetry()
        self.rotation, self.translation, self.velocity = parse_telemetry(telemetry)

    def caminfo_callback(self, msg):
        # Only need to initialize once
        if self.camera_initialized:
            return

        print("-- Initializing Camera --")

        # Image dimensions
        image_size = (msg.width, msg.height)
        update_img_dims(msg.height, msg.width)
        print("Height:", msg.height)
        print("Width:", msg.width)

        # Camera Intrinsic Matrix
        camera_matrix = np.array(msg.K).reshape((3,3))
        self.K = camera_matrix
        self.K_inv = np.linalg.inv(camera_matrix)
        fovx, fovy, focalLength, principalPoint, aspectRatio = cv2.calibrationMatrixValues(camera_matrix, image_size, 0, 0)
        print("FOV_X:", fovx)
        print("FOV_Y:", fovy)
        print("Focal Length:", focalLength)
        print("Principal Point:", principalPoint)
        print("Aspect Ratio:", aspectRatio)

        # Image distortion
        if np.count_nonzero(msg.D) > 0:
            if msg.distortion_model == 'plumb_bob':
                self.undistort = undistort_plumb_bob(camera_matrix, msg.D, image_size)
                print("Loaded 'plumb-bob' distortion model.")
            else:
                print(f"Unsupported distortion model '{msg.distortion_model}'.")

        # Camera parameter loading complete
        print("-- END --")
        self.camera_initialized = True
        self.caminfo_sub.unregister()
        

    def shutdown(self):
		# Gracefully shut down all of our windows/processes
        
        self.stop()
        print("Shutting down the drone_control node...")

        try:
            #plt.show()
            pass

        except:
            e = sys.exc_info()[1]
            print("There was a problem shutting down.")
            print(e)

        return

def parse_args():
    kwargs = {}

    #if rospy.has_param('~resolution'):
    #    kwargs['resolution'] = rospy.get_param('~resolution')
    #else:
    #    rospy.logwarn(f"PARAMS: Resolution not provided; using {GRID_RESOLUTION_DEFAULT:.1f}")

    return kwargs

def main():
    rospy.init_node('drone_control', anonymous=False)
    kwargs = parse_args()
    driver = Driver(**kwargs)
    driver.register()
    driver.main_loop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("drone_control node terminated")
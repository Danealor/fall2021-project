#!/usr/bin/env python3
import sys
import rospy
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
import clover.srv

import cv2, numpy as np
from cv_bridge import CvBridge
from matplotlib import pyplot as plt

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

def update_img_dims(shape):
    HEIGHT, WIDTH = shape[:2]

def idx_to_xy(idx):
    return idx[...,::-1] * (1,-1) + (0, HEIGHT)

def xy_to_idx(xy):
    return ((xy - (0, HEIGHT)) * (1,-1))[...,::-1]
    
def to_cv(idx):
    return np.round(idx[...,::-1]).astype(int).clip(-9999, 9999)

### Control Logic ###

# Parameters
CONTROL_RATE = 5 # Hz
SPEED_TAKEOFF = 0.5 # m/s
SPEED_FLY = 1.0 # m/s
FLIGHT_HEIGHT = 1.5 # m
TARGET_FLOW_DIST = 1.0 # m

class Driver(object):

    def __init__(self):
        self.running = False
        self.state = 'take_off'
        self.img_msg = None
        self.bridge = CvBridge()
        self.filter_red = Filter(**RED_FILTER)

    def register(self):
        self.image_sub = rospy.Subscriber('/main_camera/image_raw', 
                                          Image, self.image_callback, queue_size=1)

        self.get_telemetry = rospy.ServiceProxy('get_telemetry', clover.srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', clover.srv.Navigate)
        self.navigate_global = rospy.ServiceProxy('navigate_global', clover.srv.NavigateGlobal)
        self.set_position = rospy.ServiceProxy('set_position', clover.srv.SetPosition)
        self.set_velocity = rospy.ServiceProxy('set_velocity', clover.srv.SetVelocity)
        self.set_attitude = rospy.ServiceProxy('set_attitude', clover.srv.SetAttitude)
        self.set_rates = rospy.ServiceProxy('set_rates', clover.srv.SetRates)
        self.land = rospy.ServiceProxy('land', Trigger)
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
        success = self.navigate(z=FLIGHT_HEIGHT, speed=SPEED_TAKEOFF, frame_id='body', auto_arm=True)
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
        update_img_dims(img.shape)

        # Filter image for boundary
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        filter = self.filter_red.apply(hsv)
        filter_img = (filter * 255).astype(np.uint8)
        edges = cv2.Canny(filter_img, 20, 240)

        edges_idx = np.moveaxis(np.indices(edges.shape),0,-1)[edges>127].reshape(-1,2)
        edges_xy = idx_to_xy(edges_idx)

        if edges_xy.shape[0] < 10:
            # No boundary found, just go forward
            self.set_velocity(vx=SPEED_FLY, frame_id='body')
            cv2.imshow("Camera", img)
            cv2.waitKey(3)
            return

        # Draw edge points
        img_edges = np.zeros_like(img)
        img_edges[tuple(xy_to_idx(edges_xy).astype(int).clip((0,0),(HEIGHT-1,WIDTH-1)).T)] = (0, 0, 255)

        # Find closest point
        center_idx = np.array(img.shape[:2])/2
        center_xy = idx_to_xy(center_idx)

        closest_pt_xy = edges_xy[np.argmin(np.linalg.norm(edges_xy - center_xy, axis=-1))] 
        closest_pt_idx = xy_to_idx(closest_pt_xy)

        # Draw line to closest point
        cv2.line(img, to_cv(center_idx), to_cv(closest_pt_idx), (0,255,255), 2)
        cv2.line(img_edges, to_cv(center_idx), to_cv(closest_pt_idx), (0,255,255), 1)

        # Control drone
        self.direct(closest_pt_xy - center_xy)

        # Show imagery
        cv2.imshow("Camera", img)
        cv2.imshow("Parsed", img_edges)
        cv2.waitKey(3)
        

    def stop(self):
        self.running = False
        self.land()

    ### Control the drone's flight path ###

    def direct(self, direction):
        dir_unit = direction / np.linalg.norm(direction)
        dx, dy = TARGET_FLOW_DIST * dir_unit
        self.set_position(x=dx, y=dy, frame_id='body')

    ### Receive and parse callbacks from the drone ###

    def image_callback(self, msg):
        self.img_msg = msg


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
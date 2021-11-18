#!/usr/bin/env python3
import sys
import rospy
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
import clover.srv

### Control Logic ###

# Parameters
CONTROL_RATE = 5 # Hz

class Driver(object):

    def __init__(self):
        self.running = False

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
                pass

            rate.sleep()

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
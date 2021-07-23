from morse.builder import *
from lab_world.builder.sensors import Rosimu

class Husky(GroundRobot):
    """
    A template robot model for husky, with a motion controller and a pose sensor.
    """
    def __init__(self, name = None, debug = True):

        # husky.blend is located in the data/robots directory
        GroundRobot.__init__(self, 'lab_world/robots/husky.blend', name)
        self.properties(classpath = "lab_world.robots.husky.Husky")

        ###################################
        # Actuators
        ###################################


        # (v,w) motion controller
        # Check here the other available actuators:
        # http://www.openrobots.org/morse/doc/stable/components_library.html#actuators
        self.motion = MotionVW()
        self.append(self.motion)
        self.motion.add_interface('ros', topic='/cmd_vel')

        # Optionally allow to move the robot with the keyboard
        if debug:
            keyboard = Keyboard()
            keyboard.properties(ControlType = 'Position')
            self.append(keyboard)

        ###################################
        # Sensors
        ###################################

        self.pose = Pose()
        self.append(self.pose)
        self.pose.add_interface('ros', topic='/pose')

        self.imu = Rosimu()
        self.append(self.imu)
        self.imu.add_interface('ros', topic='/imu')

        self.lidar = Hokuyo()
        self.lidar.properties(scan_window = 360.0)
        self.lidar.frequency(5.0)
        self.lidar.translate(0, 0, 1.923)
        self.append(self.lidar)
        self.lidar.add_interface('ros', topic='/velodyne_points')


"""husky_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Gyro, Accelerometer, Supervisor
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, PointCloud
from rosgraph_msgs.msg import Clock

#DIFF DRIVE CONSTANTS
wheelRadius=0.165 
treadConstant=0.75

# create the Robot instance.
robot = Robot()
# supervisor = Supervisor()
# robot = supervisor.getFromDef("Husky")

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

flMotor = robot.getMotor('front_left_wheel')
frMotor = robot.getMotor('front_right_wheel')
rlMotor = robot.getMotor('rear_left_wheel')
rrMotor = robot.getMotor('rear_right_wheel')

acc = robot.getAccelerometer('accelerometer')
gyro = robot.getGyro('gyro')
lidar = robot.getLidar('Velodyne VLP-16')

# flMotor.setPosition(10)
# frMotor.setPosition(-10)
# rlMotor.setPosition(10)
# rrMotor.setPosition(-10)
flMotor.setPosition(float('inf'))
frMotor.setPosition(float('inf'))
rlMotor.setPosition(float('inf'))
rrMotor.setPosition(float('inf'))
flMotor.setVelocity(0.0)
frMotor.setVelocity(0.0)
rlMotor.setVelocity(0.0) 
rrMotor.setVelocity(0.0)

acc.enable(1)
gyro.enable(1)

lidar.enable(1)
lidar.enablePointCloud()

#ROS functions
def getLeftWheelSpeed(x, w):
    return (x - treadConstant*w)/(wheelRadius)
def getRightWheelSpeed(x, w):
    return (x + treadConstant*w)/(wheelRadius)
def handle_cmd_vel(data):
    leftSpeed = getLeftWheelSpeed(data.linear.x, data.angular.z)
    rightSpeed = getRightWheelSpeed(data.linear.x, data.angular.z)
    print(leftSpeed)
    print(rightSpeed)
    flMotor.setVelocity(leftSpeed)
    frMotor.setVelocity(rightSpeed)
    rlMotor.setVelocity(leftSpeed)
    rrMotor.setVelocity(rightSpeed)
    
def getImuData():
    a = acc.getValues()
    g = gyro.getValues()

    imu = Imu()
    imu.header.frame_id = 'world'
    imu.header.stamp = rospy.Time.from_sec(robot.getTime())
    imu.orientation.x = 0.0
    imu.orientation.y = 0.0
    imu.orientation.z = 0.0
    imu.orientation.w = 1.0
    imu.linear_acceleration.x = a[2]
    imu.linear_acceleration.y = a[1]    
    imu.linear_acceleration.z = a[0]
    imu.angular_velocity.x = g[2]
    imu.angular_velocity.y = g[1]    
    imu.angular_velocity.z = g[0]
    return imu
    
def getLidarData(points):
    pc = PointCloud()
    pc.header.frame_id = 'world'
    pc.header.stamp = rospy.Time.from_sec(robot.getTime())
    pc.points = points
    # for i in range(lidar.getNumberOfPoints()):
        # pc.points[i].x = points[i].x
        # pc.points[i].y = points[i].y
        # pc.points[i].z = points[i].z
    return pc

#ROS setup
rospy.init_node('husky_webots_controller')
rospy.Subscriber("/cmd_vel", Twist, handle_cmd_vel)
imuPub = rospy.Publisher("/imu/data", Imu, queue_size=10)
clkPub = rospy.Publisher("/clock", Clock, queue_size=10)
#lidarPub = rospy.Publisher("/velodyne_points", PointCloud, queue_size=10)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    imuData = getImuData()
    clkTime = rospy.Time.from_sec(robot.getTime())
    #lidarData = getLidarData(lidar.getPointCloud())
    
    # Process sensor data here.
    imuPub.publish(imuData)
    clkPub.publish(clkTime)
    #lidarPub.publish(lidarData)

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0))
    
    pass

# Enter here exit cleanup code.

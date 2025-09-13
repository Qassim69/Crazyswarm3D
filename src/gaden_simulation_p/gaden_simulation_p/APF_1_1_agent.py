#!/usr/bin/env python3

"""
Artificial Potential Field Agent

NOTE: the Mellinger controller is Crazyswarm's default controller, but it has not been tuned (or even tested) for velocity control mode.
      Switch to the PID controller by changing`firmwareParams.stabilizer.controller` to `1` in your launch file.
-> Done
"""
"""IMPORTS"""
import numpy as np
import rclpy
import csv
import time

from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from std_msgs.msg import ColorRGBA, UInt8
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from visualization_msgs.msg import Marker
from olfaction_msgs.msg import GasSensor

from gaden_simulation_interfaces.srv import GetForces

from crazyflie_py.crazyflie_py.crazyflie import Crazyflie

from rcl_interfaces.srv import ListParameters, DescribeParameters
from rcl_interfaces.msg import ParameterType

"""PARAMETERS"""
# Computational
EPSILON = np.finfo(np.float16).eps

# Flight & Navigation parameters
MAX_LIN_VELOCITY    =   0.4     	# m/s   only theoretically, if obstacle, traffic and bout forces would all align
MAX_MODE_TIME       =   10       	# s
HEIGHT_DESIRED      =   0.5     	# m
UPDATE_RATE         =   5      		# 1/s
CLAMPING_THRESHOLD  =   0.25
SWITCHING_THRESHOLD =   0.25
SWITCHING_TIME      =   10.0
WEIGHTS 	    = np.array([2,1]) 	#(obstacle + traffic),attraction(bout/random),cruise speed:attraction weight/sum(weights)=1/3*max_vel
BOUNDS 		    = [[0,0,0],[10,6,2.5]]   	# [[min],[max]]; [m]

# Bout detection parameters
BOUT_THRESHOLD      =   200     	# bout amplitude threshold for noise reduction
TAU                 =   0.5     	# [s] halflife for emwa smoothing of x and its derivatives

# Communication
PREFIX              = "GSL"
BOUT_TOPIC          = f"{PREFIX}/bouts"
GETFORCES_SERVICE   = f"{PREFIX}/getForces"
SENSOR_TOPIC        = "/mox{}/Sensor_reading"
CF                  = "/cf{}"

"""
def initParams():
    global BOUT_THRESHOLD, TAU, CLAMPING_THRESHOLD, SWITCHING_THRESHOLD, SWITCHING_TIME
    BOUT_THRESHOLD = rospy.get_param("/CrazyflieDistributed/bout_threshold")
    TAU = rospy.get_param("/CrazyflieDistributed/bout_halflife")
    CLAMPING_THRESHOLD = rospy.get_param("/CrazyflieDistributed/mc_clamping_threshold")    
    SWITCHING_THRESHOLD = rospy.get_param("/CrazyflieDistributed/mc_switching_threshold")    
    SWITCHING_TIME = rospy.get_param("/CrazyflieDistributed/mc_switching_time")   
"""

"""BOUT DETECTION"""
class EmwaData:
    """Helper class to manage sensor data and derivatives"""
    def __init__(self, alpha, value=0, old_value=0):
        """
        Constructor
        Args:
        alpha (float): Factor for emwa-filter: current_value = new_value*alpha + current_value*(1-alpha)
        """
        self.old_value = old_value
        self.value = value
        self.alpha = alpha
    
    def emwa(self, new_value):
        """
        Applies exponentially-weighted moving average (emwa) filter.
        Args:
        new_value (float): New entry for emwa filter
        """
        self.old_value = self.value
        self.value = new_value*self.alpha + self.value*(1-self.alpha)
   
    def diff(self):
        return self.value - self.old_value
    
    def positiveZeroCrossing(self):
        return (self.old_value <= 0 and self.value > 0)
    
    def negativeZeroCrossing(self):
        return (self.old_value >= 0 and self.value < 0)

class BoutDetector:
    """Detects bouts and publishes the location via the passed publisher"""
    def __init__(self, node, tau, freq, threshold, positive, publisher, cf):
        """
        Constructor
        Args:
        tau(float):       halflife for emwa-filter: x = new_x*alpha + x*(1-alpha); alpha = 1-exp(log(0.5)/tau*freq)
        freq(float):      frequency of sensor measurements for emwa filter: x = new_x*alpha + x*(1-alpha); alpha = 1-exp(log(0.5)/tau*freq)
        threshold(float): threshold of x_d1 for bout detection, to discard noise induced bouts
        positive(Bool):   True if raw ~ concentration (e.g. signal=concentration); False if -raw ~ concentration (e.g. signal=resistance)
        publisher(rospy.Publisher): Publisher to publish point to after bout has been detected
        tf(tf.TransformListener):   TransformListener from main for position queries
        cfid(int):                  id of active crazyflie
        """
        self.node = node
        self.alpha = 1-np.exp(np.log(0.5)/(tau*freq))   # alpha from halflife
        self.threshold = threshold
        self.raw_factor = 1 if positive else -1
        self.publisher = publisher
        self.cf = cf

        # init emwas
        self.x_d0 = EmwaData(self.alpha)
        self.x_d1 = EmwaData(self.alpha)
        self.x_d2 = EmwaData(self.alpha)

        self.candidatePoint = Point(0,0,0)
        self.candidateStartValue = 0
        self.candidateFlag = False
        self.startTime = node.get_clock().now() + Duration(seconds=5)

    def run(self, raw):
        """
        Run bout detection by inputing new data
        Args:
        raw (float): sensor data
        """
        self.x_d0.emwa(raw*self.raw_factor)
        self.x_d1.emwa(self.x_d0.diff())
        self.x_d2.emwa(self.x_d1.diff())

        #Points are flagged as candidates if a positive zero-crossing occurs
        if self.x_d2.positiveZeroCrossing():
            try:
                pos = self.cf.position()
                self.candidatePoint = Point(x=pos[0], y=pos[1], z=pos[2])
                self.candidateStartValue = self.x_d1.old_value
                self.candidateFlag = True
            except Exception as e:
                self.node.get_logger().warning(f"TF lookup failed: {e}")
        
        #Candidates are confirmed as bouts if threshold is exceeded
        if self.candidateFlag and ((self.x_d1.value - self.candidateStartValue) > self.threshold):
            self.candidateFlag = False
            if self.node.get_clock().now() > self.startTime:
                self.publisher.publish(self.candidatePoint)

        #Candidates are discarded if negative zero-crossing occurs
        if self.x_d2.negativeZeroCrossing():
            self.candidateFlag = False

def sensorCallback(msg, pkg):
    """
    Function called for every new datapoint received from sensor
    Args:
        msg: message received on the sensor topic
        pkg: list of [BoutDetector, Crazyflie, csv.writer]
    """
    raw = msg.raw
    boutDetector, cf, writer = pkg

    # Run bout detection logic
    boutDetector.run(raw)

    # Logging: use wall time (not ROS time)
    pos = cf.position()
    writer.writerow([time.time(), *pos, raw])

class MotionController:
    """Derives velocity commands from current state of the agent and environment"""
    def __init__(self, node, bounds, max_velocity, max_time, service_name, cf, cfid):
        self.node = node
        self.bounds = bounds
        self.max_velocity = max_velocity
        self.max_time = max_time
        self.cfid = cfid
        self.random_mode = False
        self.random_setpoint = np.array([0,0])
        
        self.client = node.create_client(GetForces, service_name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for getForces service...')

        self.cf = cf 

        self.velocity = np.array([0,0,0])
        self.setpoint_mode = False    		# True: Try to reach random point; False: Follow bout force
        self.setpoint = np.array([0,0])
        self.offset = 0.5
        
        self.generateSetpoint()
        
        self.force_threshold = SWITCHING_THRESHOLD
        self.modeStartTime = self.node.get_clock().now()
        self.modeDur = Duration(seconds=SWITCHING_TIME)
        
        # visualization, not utilized anymore?
        self.vis_modePub = node.create_publisher(Marker, f"cf{self.cfid}_mode", 10)
        self.modeMarker = self.initMarker(Marker.ADD, Marker.POINTS)
        self.modeMarker.points = [Point(0,0,0)]

        self.vis_setpointPub = node.create_publisher(Marker, f"cf{self.cfid}_setpoint", 10)
        self.setpointMarker = self.initMarker(Marker.ADD, Marker.POINTS, frame="world")

        self.vis_resultantPub = node.create_publisher(Marker, f"cf{self.cfid}_resultant", 10)
        self.resultantMarker = self.initMarker(Marker.ADD, Marker.ARROW, [0.1, 0.1, 0], [0, 0, 1, 1])

        self.vis_attractionPub = node.create_publisher(Marker, f"cf{self.cfid}_attraction", 10)
        self.attractionMarker = self.initMarker(Marker.ADD, Marker.ARROW, [0.1, 0.1, 0], [0, 1, 0, 1])
        
        self.boutColor = ColorRGBA(r=0.56, g=0.93, b=0.56, a=1)
        self.setpointColor = ColorRGBA(r=0.0, g=0.39, b=0.0, a=1)

        self.vis_repulsionPub = node.create_publisher(Marker, f"cf{self.cfid}_repulsion", 10)
        self.repulsionMarker = self.initMarker(Marker.ADD, Marker.ARROW, [0.1, 0.1, 0], [1, 0, 0, 1])    

    def getForces(self):
        #Get forces from environment via service
        request = GetForces.Request()
        request.id = self.cfid

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            resp = future.result()
            attraction = np.array([resp.attraction_x, resp.attraction_y, resp.attraction_z])
            repulsion = np.array([resp.repulsion_x, resp.repulsion_y, resp.repulsion_z])
            return attraction, repulsion
        else:
            self.node.get_logger().warning("Service call to GetForces failed.")
            return np.zeros(3), np.zeros(3)

    def update(self):
        f_bout, f_repulsion = self.GetForces()
        m_bout = np.linalg.norm(f_bout)
        m_repulsion  = np.linalg.norm(f_repulsion)

        # clamp bout force to 0 or 1
        if m_bout < CLAMPING_THRESHOLD: 
            f_bout *= 0
        else: 
            f_bout /= m_bout

        while True:
            f_attraction = self.getSetpointForce() if self.setpoint_mode else f_bout
            f_resultant = self.normalize(f_attraction, f_repulsion)
            m_attraction = np.linalg.norm(f_attraction)
            m_resultant = np.linalg.norm(f_resultant)
        
            now = self.node.get_clock().now()
            if m_resultant < self.force_threshold or m_attraction < self.force_threshold or self.modeStartTime + self.modeDur <= now:
                self.setpoint_mode = not self.setpoint_mode
                self.modeStartTime = now
                self.modeDur = Duration(self.max_time)

            if self.setpoint_mode:
                self.generateSetpoint()
                continue
            else:
                break

        self.velocity = f_resultant*self.max_velocity          		# Forces are normalized to the range 0 - 1
        self.cf.cmdVelocityWorld(self.velocity[:3], 0.0)	        # Instruct crazyflie
        self.node.get_logger().info(f"cmdVelocityWorld({np.append(self.velocity, 0.0)})")

        self.visualizeMode()
        self.visualizeSetpoint()
       
        self.attractionMarker.color = self.setpointColor if self.setpoint_mode else self.boutColor
        self.visualizeForce(self.attractionMarker, f_attraction, self.vis_attractionPub)
        self.visualizeForce(self.repulsionMarker, f_repulsion, self.vis_repulsionPub)
        self.visualizeForce(self.resultantMarker, f_resultant, self.vis_resultantPub)

    def generateSetpoint(self):
        min_bounds = np.array(self.bounds[0]) + self.offset
        max_bounds = np.array(self.bounds[1]) - self.offset
        self.setpoint = np.random.uniform(min_bounds, max_bounds)

    def getSetpointForce(self):
        vec = self.setpoint - self.cf.position() 
        mag = np.linalg.norm(vec)
        return vec / mag if mag > 0.25 else vec * 4		#Slow down close to setpoint

    def normalize(self, attraction, repulsion):
        resultant = repulsion + attraction
        norm = np.linalg.norm(resultant)
        return resultant / norm if norm > EPSILON else resultant

    def initMarker(self, action, type, scale=[0.1, 0.1, 0.1], color=[1, 0, 0, 1], frame=None):
        if frame is None:
            frame = f"cf{self.cfid}"

        marker = Marker()
        marker.header.frame_id = frame
        marker.type = type
        marker.action = action
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        
        return marker

    def visualizeMode(self):
        if self.setpoint_mode:
            self.modeMarker.color.r = 1
            self.modeMarker.color.g = 0
        else:
            self.modeMarker.color.r = 0
            self.modeMarker.color.g = 1
        
        self.modeMarker.header.stamp = self.node.get_clock().now().to_msg()
        self.vis_modePub.publish(self.modeMarker)

    def visualizeSetpoint(self):
        if self.setpoint_mode:
            self.setpointMarker.action = Marker.ADD
            self.setpointMarker.points = [Point(*self.setpoint)]
        else:
            self.setpointMarker.action = Marker.DELETE

        self.vis_setpointPub.publish(self.setpointMarker)

    def visualizeForce(self, marker, force, publisher):
        marker.points = [Point(0,0,0), Point(*force)]
        marker.header.stamp = self.node.get_clock().now().to_msg()    
        publisher.publish(marker)

def run(node, cf, cfid):
    #Logging 
    now = time.localtime()
    now_str = f"{now.tm_year}_{now.tm_mon}_{now.tm_mday}_{now.tm_hour}_{now.tm_min}"
    logfile = open(f"log/GSL/{now_str}_cf{cfid}.csv", "w")
    writer = csv.writer(logfile)
    writer.writerow(["t", "x", "y", "z", "raw"])
    
    #Bouts
    bout_pub = node.create_publisher(Point, BOUT_TOPIC, 10)
    bout_detector = BoutDetector(node, TAU, UPDATE_RATE, BOUT_THRESHOLD, False, bout_pub, cf)

    #Sensor 
    node.create_subscription(GasSensor,SENSOR_TOPIC.format(cfid),lambda msg: sensorCallback(msg, [bout_detector, cf, writer]),10)

    #MotionController
    motion_controller = MotionController(node, BOUNDS, MAX_LIN_VELOCITY, MAX_MODE_TIME, GETFORCES_SERVICE, cf, cfid)

    #Start crazyflie(lift off)
    cf.takeoff(targetHeight=HEIGHT_DESIRED, duration=2.0)
    time.sleep(3)

    """MAIN LOOP"""
    rate_hz = 5
    while rclpy.ok():
        motion_controller.update()
        rclpy.spin_once(node, timeout_sec=1.0 / rate_hz)
        
    logfile.close()

"""HELPER FUNCTIONS"""
# helper function to check if the supplied position lies in bounds 3D
def isInBounds(pos, bounds):
    return (bounds[0][0] < pos[0] < bounds[1][0]) and (bounds[0][1] < pos[1] < bounds[1][1]) and (bounds[0][2] < pos[2] < bounds[1][2])

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("crazyflie_distributed")
    
    cfid = node.declare_parameter("cfid", 0).value
    crazyflies = node.declare_parameter("crazyflies", []).value
    
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    cf = None
    for crazyflie in crazyflies:
        if int(crazyflie["id"]) == cfid:
            initial_position = crazyflie["initialPosition"]
            cfname = f"cf{cfid}"
            break
    
    if cf is None:
        node.get_logger().warning(f"No CF with required ID {cfid} found!")
        rclpy.shutdown()
        return
    
    paramTypeDict = fetch_param_types(node, cfname)

    cf = Crazyflie(node, cfname, paramTypeDict, tf_buffer)
    #initParams()
    cfname = f"cf{cfid}"
    run(node, cf, cfid)

    rclpy.shutdown()

def fetch_param_types(node, cfname):
    # Create clients
    list_client = node.create_client(ListParameters, '/crazyflie_server/list_parameters')
    describe_client = node.create_client(DescribeParameters, '/crazyflie_server/describe_parameters')
    
    # Wait for services
    while not list_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for ListParameters service...')
    while not describe_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for DescribeParameters service...')
    
    paramTypeDict = {}
    try:
        # Step 1: List all parameters (recursive, no prefixes to get everything)
        list_req = ListParameters.Request()
        list_req.depth = ListParameters.Request.DEPTH_RECURSIVE  # Get all nested params
        future = list_client.call_async(list_req)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            param_names = [name for name in future.result().result.names if name.startswith(f"{cfname}.params.")]
        else:
            node.get_logger().warning("Failed to list parameters")
            return paramTypeDict  # Empty fallback
        
        if not param_names:
            node.get_logger().warning(f"No parameters found for {cfname}")
            return paramTypeDict
        
        # Describe parameters to get types
        describe_req = DescribeParameters.Request()
        describe_req.names = param_names
        future = describe_client.call_async(describe_req)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            for param_name, descriptor in zip(param_names, future.result().descriptors):
                # Strip prefix (e.g., 'cf1.params.group.name' -> 'group.name')
                stripped_name = param_name.replace(f"{cfname}.params.", "")
                paramTypeDict[stripped_name] = descriptor.type  # e.g., ParameterType.PARAMETER_INTEGER
        else:
            node.get_logger().warning("Failed to describe parameters")
    except Exception as e:
        node.get_logger().error(f"Error fetching param types: {str(e)}")
    
    # Clean up clients
    node.destroy_client(list_client)
    node.destroy_client(describe_client)
    
    return paramTypeDict

if __name__ == "__main__":
    main()

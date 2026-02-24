
#!/usr/bin/env python3

"""IMPORTS"""
import numpy as np
import rclpy
import csv
import time
import ast

from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener

from std_msgs.msg import ColorRGBA, UInt8
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from visualization_msgs.msg import Marker
from olfaction_msgs.msg import GasSensor

from gaden_simulation_interfaces.msg import GenericLogData
from gaden_simulation_interfaces.srv import GetForces

from crazyflie_py.crazyflie import Crazyflie

from rcl_interfaces.srv import ListParameters, DescribeParameters
from rcl_interfaces.msg import ParameterType

"""PARAMETERS"""
# Computational
EPSILON = np.finfo(np.float16).eps

# Flight & Navigation parameters
MAX_LIN_VELOCITY    =   0.2     	# m/s   only theoretically, if obstacle, traffic and bout forces would all align
MAX_MODE_TIME       =   10       	# s
HEIGHT_DESIRED      =   0.3     	# m
UPDATE_RATE         =   5      		# 1/s
CLAMPING_THRESHOLD  =   0.675
SWITCHING_THRESHOLD =   0.375
SWITCHING_TIME      =   10.0
BOUNDS 		    = [[0.04,-0.10,0],[3.04,2.90,2.0]]   	# [[min],[max]]; [m]

# Bout detection parameters
SENSOR_RATE         =   2
BOUT_THRESHOLD      =   25     	# bout amplitude threshold for noise reduction
TAU                 =   0.25     	# [s] halflife for emwa smoothing of x and its derivatives

# Communication
PREFIX              = "GSL"
BOUT_TOPIC          = f"{PREFIX}/bouts"
GETFORCES_SERVICE   = f"{PREFIX}/GetForces"
CF                  = "/cf{}"
SENSOR_TOPIC        = "/cf{}/sgp30"	# 2 values (1L, 1R,) 2L, 2R

def batteryCheck(msg, cf, node=None):
    if msg.values[0] == 3:
        cf.land(targetHeight=0.02, duration=2)
        time.sleep(2)
        if node:
            node.get_logger().info("Battery critical. Shutting down.")
        return True
    return False
        
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
    def __init__(self, node, tau, freq, threshold, positive, publisher, cf, logger):
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
        self.logger = logger
        self.cf = cf

        # init emwas
        self.x_d0 = EmwaData(self.alpha)
        self.x_d1 = EmwaData(self.alpha)
        self.x_d2 = EmwaData(self.alpha)

        self.candidatePoint = Point(x=0.0, y=0.0, z=0.0)
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
                self.logger.writerow('a', [self.get_clock().now(), self.candidatePoint.x, self.candidatePoint.y, self.candidatePoint.z])

        #Candidates are discarded if negative zero-crossing occurs
        if self.x_d2.negativeZeroCrossing():
            self.candidateFlag = False

def sensorCallback(node, msg, pkg):
    """
    Function called for every new datapoint received from sensor
    Args:
        msg: message received on the sensor topic
        pkg: list of [BoutDetector, Crazyflie, csv.writer]
    """
    boutDetector, cf, writer = pkg

    # Run bout detection logic
    boutDetector[0].run(msg.values[0])
    boutDetector[1].run(msg.values[1])
    
    node.get_logger().info("{}".format(msg))

    # Logging: use ROS time
    ros_time = node.get_clock().now()
    pos = cf.position()
    writer.writerow([ros_time.nanoseconds * 1e-9, *pos, *msg.values])

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
            self.node.get_logger().info('Waiting for GetForces service...')

        self.cf = cf 

        self.velocity = np.array([0,0,0])
        self.setpoint_mode = False    		# True: Try to reach random point; False: Follow bout force
        self.setpoint = np.array([0,0])
        self.offset = 0.1
        
        self.generateSetpoint()
        
        self.force_threshold = SWITCHING_THRESHOLD
        self.modeStartTime = self.node.get_clock().now()
        self.modeDur = Duration(seconds=SWITCHING_TIME)
        
        # visualization, not utilized anymore?
        self.vis_modePub = node.create_publisher(Marker, f"cf{self.cfid}_mode", 10)
        self.modeMarker = self.initMarker(Marker.ADD, Marker.POINTS)
        self.modeMarker.points = [Point(x=0.0, y=0.0, z=0.0)]

        self.vis_setpointPub = node.create_publisher(Marker, f"cf{self.cfid}_setpoint", 10)
        self.setpointMarker = self.initMarker(Marker.ADD, Marker.POINTS, frame="world")

        self.vis_resultantPub = node.create_publisher(Marker, f"cf{self.cfid}_resultant", 10)
        self.resultantMarker = self.initMarker(Marker.ADD, Marker.ARROW, [0.1, 0.1, 0.0], [0, 0, 1, 1])

        self.vis_attractionPub = node.create_publisher(Marker, f"cf{self.cfid}_attraction", 10)
        self.attractionMarker = self.initMarker(Marker.ADD, Marker.ARROW, [0.1, 0.1, 0.0], [0, 1, 0, 1])
        
        self.boutColor = ColorRGBA(r=0.56, g=0.93, b=0.56, a=1.0)
        self.setpointColor = ColorRGBA(r=0.0, g=0.39, b=0.0, a=1.0)

        self.vis_repulsionPub = node.create_publisher(Marker, f"cf{self.cfid}_repulsion", 10)
        self.repulsionMarker = self.initMarker(Marker.ADD, Marker.ARROW, [0.1, 0.1, 0.0], [1, 0, 0, 1])    

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
            self.node.get_logger().info("getForces returned {}, {}.".format(attraction, repulsion))
            return attraction, repulsion
        else:
            self.node.get_logger().warning("Service call to GetForces failed.")
            return np.zeros(3), np.zeros(3)

    def update(self):
        f_bout, f_repulsion = self.getForces()
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
                self.modeDur = Duration(seconds=self.max_time)

            if self.setpoint_mode:
                self.generateSetpoint()
                continue
            else:
                break

        dz = HEIGHT_DESIRED - self.cf.position()[2]
        vz = dz/10
        if vz > 0.02: vz = 0.02

        self.velocity_setpoint = np.array([		# Forces are normalized to the range 0 - 1
            (f_resultant*self.max_velocity)[0],
            (f_resultant*self.max_velocity)[1], 
            vz
        ])
        self.velocity = self.velocity*0.25 + self.velocity_setpoint*0.75
        
        self.cf.cmdVelocityWorld(self.velocity, 0.0)	        # Instruct crazyflie
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

    def initMarker(self, action=Marker.ADD, type=Marker.POINTS, scale=[0.1,0.1,0.1], color=[1,0,0,1], frame=None):
        if frame is None:
            frame = f"cf{self.cfid}"

        marker = Marker()
        marker.header.frame_id = frame
        marker.type = type
        marker.action = action
        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])
        
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        
        return marker

    def visualizeMode(self):
        if self.setpoint_mode:
            self.modeMarker.color.r = 1.0
            self.modeMarker.color.g = 0.0
        else:
            self.modeMarker.color.r = 0.0
            self.modeMarker.color.g = 1.0
        
        self.modeMarker.header.stamp = self.node.get_clock().now().to_msg()
        self.vis_modePub.publish(self.modeMarker)

    def visualizeSetpoint(self):
        if self.setpoint_mode:
            self.setpointMarker.action = Marker.ADD
            self.setpointMarker.points = [Point(x=self.setpoint[0], y=self.setpoint[1], z=self.setpoint[2])]
        else:
            self.setpointMarker.action = Marker.DELETE

        self.vis_setpointPub.publish(self.setpointMarker)

    def visualizeForce(self, marker, force, publisher):
        marker.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=force[0], y=force[1], z=force[2])]
        marker.header.stamp = self.node.get_clock().now().to_msg()    
        publisher.publish(marker)

class Logger:
    def __init__(self, filename):
        self.filename = filename
    def writerow(self, mode, entries):
        with open(self.filename, mode) as f:
            writer = csv.writer(f)
            writer.writerow(entries)

def run(node, cf, cfid):
    node.get_logger().info(f"APF AGENT: run() function started for cf{cfid}")
    # Logging
    now = time.localtime()
    now_str = f"{now[0]}_{now[1]}_{now[2]}_{now[3]}_{now[4]}"
    
    sensorFile = 'log/GSL/Field/Sensor/'+now_str+'_cf'+str(cfid)+'_sensors.csv'
    boutFile = 'log/GSL/Field/Bout/'+now_str+'_cf'+str(cfid)+'_bouts{}.csv'

    logfile = open(sensorFile, 'w')
    sensorWriter = csv.writer(logfile) 

    boutLoggerL = Logger(boutFile.format("L2"))
    boutLoggerL.writerow("w+", ["t", "x", "y", "z"])
    
    boutLoggerR = Logger(boutFile.format("R2"))
    boutLoggerR.writerow("w+", ["t", "x", "y", "z"])

    node.get_logger().info(f'Logging sensor data to: {sensorFile}')
    node.get_logger().info(f'Logging bout data to: {boutFile}')

    # Battery Check
    node.create_subscription(GenericLogData, "/cf{}/battery".format(cfid), lambda msg: batteryCheck(msg, cf), 10)
    
    # Bouts
    bout_pub = node.create_publisher(Point, BOUT_TOPIC, 10)
    bout_detector = [BoutDetector(node, TAU, SENSOR_RATE, BOUT_THRESHOLD, False, bout_pub, cf, boutLoggerL),
                     BoutDetector(node, TAU, SENSOR_RATE, BOUT_THRESHOLD, False, bout_pub, cf, boutLoggerR)]

    # Sensor 
    node.create_subscription(GenericLogData,SENSOR_TOPIC.format(cfid),lambda msg: sensorCallback(node, msg, [bout_detector, cf, sensorWriter]),10)

    # MotionController
    motion_controller = MotionController(node, BOUNDS, MAX_LIN_VELOCITY, SWITCHING_TIME, GETFORCES_SERVICE, cf, cfid)
    
    node.get_logger().info(f"APF AGENT: About to attempt takeoff for cf{cfid}")
    # Start crazyflie(lift off)
    cf.takeoff(targetHeight=0.4, duration=2.0)

    """MAIN LOOP"""
    rate_hz = 5
    while rclpy.ok():
        motion_controller.update()
        #time.sleep(0.2)
        rclpy.spin_once(node, timeout_sec=1.0 / rate_hz)
        
    logfile.close()
    cf.land(targetHeight=0.02, duration=3.0)

"""HELPER FUNCTIONS"""
# helper function to check if the supplied position lies in bounds 3D
def isInBounds(pos, bounds):
    return (bounds[0][0] < pos[0] < bounds[1][0]) and (bounds[0][1] < pos[1] < bounds[1][1]) and (bounds[0][2] < pos[2] < bounds[1][2])

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("crazyflie_distributed")
    
    # DEBUG: Log node creation
    node.get_logger().info("=== APF Agent Main Function Started ===")
    
    cfid_param = node.declare_parameter("cfid", 0).value
    try:
        cfid = int(cfid_param)
        # DEBUG: Log cfid parameter
        node.get_logger().info(f"DEBUG: cfid parameter = {cfid_param}, converted to int = {cfid}")
    
    except (ValueError, TypeError):
        node.get_logger().error(f"Invalid cfid parameter:{cfid_param}. Using default value 0.")
        cfid = 0
    
    crazyflies_ids_param = node.declare_parameter("crazyflies_ids", "[]").value
    try:
        if isinstance(crazyflies_ids_param, str):
            # Convert string representation to actual list
            crazyflies_ids = ast.literal_eval(crazyflies_ids_param)
        else:
            crazyflies_ids = crazyflies_ids_param
        # Ensure all values are integers
        crazyflies_ids = [int(x) for x in crazyflies_ids]
        # DEBUG: Log crazyflies_ids
        node.get_logger().info(f"DEBUG: crazyflies_ids = {crazyflies_ids}")

    except (ValueError, SyntaxError, TypeError) as e:
        node.get_logger().error(f"Invalid crazyflies_ids parameter {crazyflies_ids_param}. Using default empty list. Error: {e}")
        crazyflies_ids = []
    
    crazyflies_positions_param = node.declare_parameter("crazyflies_positions", "[]").value
    try:
        if isinstance(crazyflies_positions_param, str):
            crazyflies_positions = ast.literal_eval(crazyflies_positions_param)
        else:
            crazyflies_positions = crazyflies_positions_param
        # DEBUG: Log crazyflies_positions
        node.get_logger().info(f"DEBUG: crazyflies_positions = {crazyflies_positions}")
    
    except (ValueError, SyntaxError, TypeError) as e:
        node.get_logger().error(f"Invalid crazyflies_positions parameter {crazyflies_positions_param}. Using default empty list. Error: {e}")
        crazyflies_positions = []
    
    crazyflies = []
    for i, id_val in enumerate(crazyflies_ids):
        crazyflies.append({
            "id": id_val,
            "initialPosition": crazyflies_positions[i]
        })
    
    # DEBUG: Log complete crazyflies list
    node.get_logger().info(f"DEBUG: Complete crazyflies list = {crazyflies}")
        
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    cfname = None
    initial_position = None
    for crazyflie in crazyflies:
        # DEBUG: Log each crazyflie being checked
        node.get_logger().info(f"DEBUG: Checking crazyflie {crazyflie['id']} against target cfid {cfid}")
        if int(crazyflie["id"]) == cfid:
            initial_position = crazyflie["initialPosition"]
            cfname = f"cf{cfid}"
            # DEBUG: Log successful match
            node.get_logger().info(f"DEBUG: Found matching CF! cfname = {cfname}, initial_position = {initial_position}")
            break
    
    if cfname is None:
        node.get_logger().error(f"DEBUG: No CF with required ID {cfid} found in crazyflies list!")
        node.get_logger().error(f"DEBUG: Available CF IDs: {[cf['id'] for cf in crazyflies]}")
        rclpy.shutdown()
        return
    
    # DEBUG: Log before parameter type fetching
    node.get_logger().info(f"DEBUG: About to fetch parameter types for {cfname}")
    paramTypeDict = fetch_param_types(node, cfname)
    node.get_logger().info(f"DEBUG: Fetched {len(paramTypeDict)} parameter types")
    
    # ADD THE TF DEBUG CODE HERE:
    node.get_logger().info(f"DEBUG: Checking TF availability for {cfname}")
    try:
       trans = tf_buffer.can_transform("world", cfname, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5))
       ts = trans.header.stamp
       z = trans.transform.translation.z
       node.get_logger().info(f"DEBUG TF init check: cf={cfname}, z={z:.3f}, tf_stamp={ts.sec}.{ts.nanosec:09d}," f"parent=world, child={cfname}")
       # node.get_logger().info(f"DEBUG: TF transform available for {cfname}")
    except Exception as e:
        node.get_logger().warning(f"DEBUG: TF transform not available for {cfname}: {e}")

    # DEBUG: Log before Crazyflie object creation
    node.get_logger().info(f"DEBUG: Creating Crazyflie object for {cfname}")
    cf = Crazyflie(node, cfname, paramTypeDict, tf_buffer)
    node.get_logger().info(f"DEBUG: Crazyflie object created successfully")
    
    try:
        test_pos = cf.position
        node.get_logger().info(f"DEBUG: TF lookup test successful: {test_pos}")
    except Exception as e:
        node.get_logger().error(f"DEBUG: TF lookup failed during initialization: {e}")

    cfname = f"cf{cfid}"  # This line is redundant - already set above
    
    # Add timeout before run()
    node.get_logger().info(f"DEBUG: Waiting 2 seconds before run...")
    time.sleep(2)
    
    # DEBUG: Log before entering run function    
    node.get_logger().info(f"DEBUG: About to call run function for cfid={cfid}, cfname={cfname}")
    node.get_logger().info(f"DEBUG: Initial position should be: {initial_position}")
    
    run(node, cf, cfid)

    rclpy.shutdown()

def fetch_param_types(node, cfname):
    # Create clients
    list_client = node.create_client(ListParameters, '/crazyflie_server/list_parameters')
    describe_client = node.create_client(DescribeParameters, '/crazyflie_server/describe_parameters')
    
    # Wait for services
    if not list_client.wait_for_service(timeout_sec=10.0):
        node.get_logger().warn('Services not available, proceeding without parameter types')
        return {}
    if not describe_client.wait_for_service(timeout_sec=10.0):
        node.get_logger().warn('Services not available, proceeding without parameter types') 
        return {}
    
    paramTypeDict = {}
    try:
        # Step 1: List all parameters (recursive, no prefixes to get everything)
        list_req = ListParameters.Request()
        list_req.depth = ListParameters.Request.DEPTH_RECURSIVE  # Get all nested params
        future = list_client.call_async(list_req)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            param_names = [name for name in future.result().result.names if name.startswith("robot_types.default.firmware_params.")]
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
                stripped_name = param_name.replace("robot_types.default.firmware_params.", "")
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

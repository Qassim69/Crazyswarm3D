
#!/usr/bin/env python3

"""IMPORTS"""
import numpy as np
import rclpy
import csv
import time
import ast

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.clock import Clock, ClockType
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
UPDATE_RATE         =   5      		# Hz
CLAMPING_THRESHOLD  =   0.675
SWITCHING_THRESHOLD =   0.375
SWITCHING_TIME      =   10.0
BOUNDS 		    = [[0.04,-0.10,0],[3.04,2.90,2.0]]   	# [[min],[max]]; [m]

# Bout detection parameters
SENSOR_RATE         =   20     # Hz
BOUT_THRESHOLD      =   25     # bout amplitude threshold for noise reduction
TAU                 =   0.25   # [s] halflife for emwa smoothing of x and its derivatives

# Communication
PREFIX              = "GSL"
BOUT_TOPIC          = f"{PREFIX}/bouts"
GETFORCES_SERVICE   = f"{PREFIX}/GetForces"
CF                  = "/cf{}"
SENSOR_TOPIC        = "/cf{}/sgp30"	# 2 values (1L, 1R,) (2L, 2R)

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
        self.system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.startTime = self.system_clock.now() + Duration(seconds=5)

    def run(self, raw):
        """
        Run bout detection by inputing new data
        Args:
        raw (float): sensor data
        """
        self.x_d0.emwa(raw*self.raw_factor)
        self.x_d1.emwa(self.x_d0.diff())
        self.x_d2.emwa(self.x_d1.diff())

        # Points are flagged as candidates if a positive zero-crossing occurs
        if self.x_d2.positiveZeroCrossing():
            try:
                pos = self.cf.position()
                self.candidatePoint = Point(x=pos[0], y=pos[1], z=pos[2])
                self.candidateStartValue = self.x_d1.old_value
                self.candidateFlag = True
            except Exception as e:
                self.node.get_logger().warning(f"TF lookup failed: {e}")
        
        # Candidates are confirmed as bouts if threshold is exceeded
        if self.candidateFlag and ((self.x_d1.value - self.candidateStartValue) > self.threshold):
            self.candidateFlag = False
            if self.system_clock.now() > self.startTime:
                self.publisher.publish(self.candidatePoint)
                wall_time = self.system_clock.now().nanoseconds * 1e-9
                self.logger.writerow('a', [wall_time, self.candidatePoint.x, self.candidatePoint.y, self.candidatePoint.z])

        # Candidates are discarded if negative zero-crossing occurs
        if self.x_d2.negativeZeroCrossing():
            self.candidateFlag = False

def sensorCallback(node, msg, pkg):
    """
    Function called for every new datapoint received from sensor
    Args:
        msg: message received on the sensor topic
        pkg: list of [BoutDetector, Crazyflie, csv.writer]
    """
    # List of Tools passed via lambda    
    boutDetector, cf, sensorWriter = pkg

    # Apply Bout Detection logic on Gas Conc. Message
    boutDetector[0].run(msg.values[0])
    boutDetector[1].run(msg.values[1])
    
    node.get_logger().info("{}".format(msg))

    # Logging: use Wall Time
    system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
    wall_time = system_clock.now().nanoseconds * 1e-9
    pos = cf.position()
    sensorWriter.writerow([wall_time, *pos, *msg.values])

class MotionController:
    """Derives velocity commands from current state of the agent and environment"""
    def __init__(self, node, bounds, max_velocity, max_time, service_name, cf, cfid):
        # The main ROS 2 node reference
        self.node = node
        # The 3D physical boundaries of the environment [[min], [max]]
        self.bounds = bounds
        # The Maximum Velocity the drone can travel (m/s)
        self.max_velocity = max_velocity
        # Maximum duration for each Mode[Setpoint or Bout Point] before switching (seconds)
        self.max_time = max_time
        # The unique ID for a Crazyflie Drone
        self.cfid = cfid
        # The Crazyflie control object [cf.cmdVelocityWorld(), cf.position(), cf.takeoff()]
        self.cf = cf
        
        # This is a Client Service that Requests the Attraction and Repulsion Forces from APF_Field_1_env.py [GetForcesServer class]
        self.client = node.create_client(GetForces, service_name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for GetForces service...')

        # Initial velocity vector[x,y,z] to be sent to the Drone until the first velocity is calculated by the update() function.
        self.velocity = np.array([0,0,0])
        # Initial Mode for the Drone.
        self.setpoint_mode = True       # True: Try to reach Random Point; False: Follow Bout Force
        # Initial Setpoint Coordinate[x,y,z] for the Drone until the first setpoint is generated by generateSetpoint() function.
        self.setpoint = np.array([0.0, 0.0, 0.0], dtype=float)
        
        # Minimum distance (m) to keep setpoints away from walls
        self.offset = 0.1

        # Generate the first Random Setpoint 
        self.generateSetpoint()
        
        # Stores the pending asynchronous request to the GetForces service
        self._forces_future = None
        # Initially sets the Attraction and Repulsion Forces to Zero Vectors until the first successful service response is received.
        self._last_forces = (np.zeros(3, dtype=float), np.zeros(3, dtype=float))        # Format: (Attraction_Vector[x,y,z], Repulsion_Vector[x,y,z])

        # Minimum required force magnitude to stay in current mode
        self.force_threshold = SWITCHING_THRESHOLD

        self.system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        # Wall Clock Timestamp of when the Initial Mode[Setpoint] started
        self.modeStartTime = self.system_clock.now()
        # Duration for how long the Drone should stay in each Mode[Setpoint or Bout Point] before switching
        self.modeDur = Duration(seconds=SWITCHING_TIME)
        
        # 1. Mode Marker: Creates a Publisher for the topic cf{id}_mode.
        self.vis_modePub = node.create_publisher(Marker, f"cf{self.cfid}_mode", 10)
        # A POINT Marker is used to Represent the Mode[Setpoint or Bout Point].
        self.modeMarker = self.initMarker(Marker.ADD, Marker.POINTS)
        self.modeMarker.points = [Point(x=0.0, y=0.0, z=0.0)]

        # 2. Setpoint Marker: Creates a Publisher for the topic cf{id}_setpoint.
        self.vis_setpointPub = node.create_publisher(Marker, f"cf{self.cfid}_setpoint", 10)
        # A POINT Marker is used to Represent the Setpoint[in the world frame].
        self.setpointMarker = self.initMarker(Marker.ADD, Marker.POINTS, frame="world")

        # 3. Force Markers: Creates Publishers for the topics cf{id}_attraction.
        self.vis_attractionPub = node.create_publisher(Marker, f"cf{self.cfid}_attraction", 10)
        # An ARROW Marker is used to Represent the Attraction Force.
        self.attractionMarker = self.initMarker(Marker.ADD, Marker.ARROW, [0.1, 0.1, 0.0], [0, 1, 0, 1])
        # 4. Force Markers: Creates Publishers for the topics cf{id}_repulsion.
        self.vis_repulsionPub = node.create_publisher(Marker, f"cf{self.cfid}_repulsion", 10)
        # An ARROW Marker is used to Represent the Repulsion Force.
        self.repulsionMarker = self.initMarker(Marker.ADD, Marker.ARROW, [0.1, 0.1, 0.0], [1, 0, 0, 1]) 
        # 5. Force Markers: Creates Publishers for the topics cf{id}_resultant.
        self.vis_resultantPub = node.create_publisher(Marker, f"cf{self.cfid}_resultant", 10)
        # An ARROW Marker is used to Represent the Resultant Force.
        self.resultantMarker = self.initMarker(Marker.ADD, Marker.ARROW, [0.1, 0.1, 0.0], [0, 0, 1, 1])
        
        # Colors for the Markers when in Bout Mode and Setpoint Mode.
        self.boutColor = ColorRGBA(r=0.56, g=0.93, b=0.56, a=1.0)       # Light Green for Bout Mode
        self.setpointColor = ColorRGBA(r=0.0, g=0.39, b=0.0, a=1.0)     # Dark Green for Setpoint Mode 

    def getForces(self):
        """Retrieves the Attraction and Repulsion Forces from the GetForces service."""
        # If there is a completed future, consume it and cache the result
        future = self._forces_future
        if future is not None:
            if future.done():
                try:
                    resp = future.result() # type: ignore
                    if resp is not None:
                        attraction = np.array([resp.attraction_x, resp.attraction_y, resp.attraction_z], dtype=float)
                        repulsion = np.array([resp.repulsion_x, resp.repulsion_y, resp.repulsion_z], dtype=float)
                        self._last_forces = (attraction, repulsion)
                
                except Exception as e:
                    self.node.get_logger().warning(f"GetForces future failed: {e}")
                
                finally:
                    self._forces_future = None

        # If no in-flight request, start a new one
        if self._forces_future is None:
            req = GetForces.Request()
            req.id = self.cfid
            self._forces_future = self.client.call_async(req)

        # Always return the last known forces (zeros initially)
        return self._last_forces

    def update(self):
        # Get the latest Attraction[f_bout] and Repulsion[f_repulsion] Forces from the GetForces service
        f_bout, f_repulsion = self.getForces()
        self.node.get_logger().debug(f"Forces types: bout={[type(x) for x in f_bout]}, repulsion={[type(x) for x in f_repulsion]}")
        
        # Calculate the Magnitude of the Attraction Force[f_bout], it is a Direction Unit Vector.
        m_bout = np.linalg.norm(f_bout)
        # Calculate the Magnitude of the Repulsion Force[f_repulsion], it is a Direction Unit Vector.
        # m_repulsion  = np.linalg.norm(f_repulsion)

        # If the Magnitude of the Attraction Force[m_bout] is below the Clamping Threshold, it is set to Zero Vector. Otherwise, it is Normalized to a Unit Vector.
        if m_bout < CLAMPING_THRESHOLD: 
            f_bout *= 0
        else:
            f_bout /= m_bout

            # If the Attraction Force is valid (not zero), ensure we are in Bout Mode. If we were in Setpoint Mode, switch to Bout Mode.
            if self.setpoint_mode:
                self.setpoint_mode = False

        while True:
            # If the Setpoint Mode is True[Random Point Mode], the getSetpointForce() Function gives Direction Unit Vector and it is stored in f_attraction. 
            # If the Setpoint Mode is False[Bout Mode], the Bout Attraction Force[f_bout] is stored in f_attraction.
            f_attraction = self.getSetpointForce() if self.setpoint_mode else f_bout

            # If getSetpointForce() returns Direction Unit Vector[vec/mag], normalising f_attraction gives 1.
            m_attraction = np.linalg.norm(f_attraction)
            
            if self.setpoint_mode:
                # In setpoint mode, attraction magnitude encodes distance via vec*4
                # When far (>0.25m): getSetpointForce returns Direction Unit Vector[vec/mag] and magnitude approx 1.0
                # When close (<0.25m): getSetpointForce returns Distance Vector[vec*4] and magnitude is less than 1.0 because Distance Vector[vec]*Scalar Vector[4]
                speed_scale = min(m_attraction, 1.0)
            else:
                # In bout mode, always full speed
                speed_scale = 1.0
            
            # The sum of Direction Unit Vector[f_attraction] and Repulsion Force[f_repulsion] is stored in f_resultant
            f_resultant = f_repulsion + f_attraction
            # Magnitude of Direction Unit Vector + Repulsion Forces
            m_resultant = np.linalg.norm(f_resultant)
            
            # Checks the If-condition for Switching Modes
            now = self.system_clock.now()
            if m_resultant < self.force_threshold or m_attraction < self.force_threshold or self.modeStartTime + self.modeDur <= now:
                self.setpoint_mode = not self.setpoint_mode
                self.modeStartTime = now
                self.modeDur = Duration(seconds=self.max_time)

                if self.setpoint_mode:
                    self.generateSetpoint()
                continue
            else: 
                break

        # Calculate Velocity Value by combining speed_scale
        if m_resultant > EPSILON:
            direction_unit = f_resultant / m_resultant
            self.velocity = direction_unit * self.max_velocity * speed_scale
        else:
            self.velocity = np.zeros(3)

        # Instruct crazyflie
        self.cf.cmdVelocityWorld(self.velocity[:3], 0.0)
        self.node.get_logger().info(f"cmdVelocityWorld({np.append(self.velocity, 0.0)}), speed_scale={speed_scale:.3f}")

        # # Safety Check: Ensure the drone is within bounds before sending velocity
        # pos = self.cf.position()
        # if isInBounds(pos, self.bounds):
        #     # Normal operation — send calculated velocity
        #     self.cf.cmdVelocityWorld(self.velocity[:3], 0.0)
        #     self.node.get_logger().info(f"cmdVelocityWorld({np.append(self.velocity, 0.0)}), speed_scale={speed_scale:.3f}")
        # else:
        #     # Out of bounds — push drone back toward center
        #     center = (np.array(self.bounds[0]) + np.array(self.bounds[1])) / 2.0
        #     retreat_dir = center - pos
        #     mag = np.linalg.norm(retreat_dir)
        #     if mag > EPSILON:
        #         retreat_dir /= mag
        #     self.cf.cmdVelocityWorld(retreat_dir * self.max_velocity * 0.5, 0.0)
        #     self.node.get_logger().warning(f"Out of bounds at {pos}, retreating to center")

    # Instruct RViz to Visualize the Mode, Setpoint and Forces with Markers
        # Starts the def visualizeMode() function to visualize the Mode[Setpoint or Bout Point].
        self.visualizeMode()
        # Starts the def visualizeSetpoint() function to visualize the Setpoint where the drone is trying to go.
        self.visualizeSetpoint()
        # Assigns Color to the Attraction Force Marker based on the Mode[Setpoint or Bout Point].
        self.attractionMarker.color = self.setpointColor if self.setpoint_mode else self.boutColor
        # Starts the def visualizeForce() function to visualize the Attraction.
        self.visualizeForce(self.attractionMarker, f_attraction, self.vis_attractionPub)
        # Starts the def visualizeForce() function to visualize the Repulsion.
        self.visualizeForce(self.repulsionMarker, f_repulsion, self.vis_repulsionPub)
        # Starts the def visualizeForce() function to visualize the Resultant.
        self.visualizeForce(self.resultantMarker, f_resultant, self.vis_resultantPub)

    def generateSetpoint(self):
        """Generates a Random Setpoint within bounds and offset from walls to avoid collisions."""
        min_bounds = np.array(self.bounds[0]) + self.offset	# [0.1, 0.1, 0.1]
        max_bounds = np.array(self.bounds[1]) - self.offset	# [9.9, 5.9, 2.5]
        self.setpoint = np.random.uniform(min_bounds, max_bounds)

        # Fixed Height:
        #x = np.random.uniform(min_bounds[0], max_bounds[0])
        #y = np.random.uniform(min_bounds[1], max_bounds[1])
        #z = 0.5  # Fixed height
        #self.setpoint = np.array([x, y, z])

    def getSetpointForce(self):
        """Calculates the Attraction Force towards the Setpoint based on the current position."""
        # Distance from Current Position to Setpoint
        vec = self.setpoint - self.cf.position()
        # Magnitude of the Vector [vec]
        mag = np.linalg.norm(vec)
        # When Magnitude>0.25, return Direction Unit Vector. 
        # When Magnitude<=0.25, return a Distance Vector that is Scaled. The Distance Vector is Multiplied by a Scalar Vector[4] which makes the magnitude less than 1.0
        return vec/mag if mag > 0.25 else vec * 4

    def initMarker(self, action=Marker.ADD, type=Marker.POINTS, scale=[0.1,0.1,0.1], color=[1,0,0,1], frame=None):
        # If no Frame, it defaults to the Drone's Frame.[cf1_mode, cf1_setpoint, cf1_resultant etc.]
        if frame is None:
            frame = f"cf{self.cfid}"

        marker = Marker()
        marker.header.frame_id = frame
        marker.type = type
        marker.action = action

        # Scale.x is used to scale POINTS Marker, but Scale.x,y,z is used to scale ARROW Marker
        marker.scale.x = float(scale[0])    # Scale.x = shaft diameter
        marker.scale.y = float(scale[1])    # Scale.y = head diameter
        marker.scale.z = float(scale[2])    # Scale.z = head length
        
        # Set color of the Marker 
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        
        return marker

    def visualizeMode(self):
        """Based on the Mode [Setpoint or Bout Point] a single point marker on the drone is Broadcasted."""
        # Red for Setpoint Mode, Green for Bout Mode
        if self.setpoint_mode:
            self.modeMarker.color.r = 1.0
            self.modeMarker.color.g = 0.0
        else:
            self.modeMarker.color.r = 0.0
            self.modeMarker.color.g = 1.0

        # Latest Marker with the exact Wall Time Timestamp
        self.modeMarker.header.stamp = self.system_clock.now().to_msg()
        # Publishes the Mode[Setpoint or Bout Point] Marker
        self.vis_modePub.publish(self.modeMarker)

    def visualizeSetpoint(self):
        """It manages the Marker that shows where the drone is trying to go."""
        if self.setpoint_mode:
            # RViz is instructed to Draw the Marker
            self.setpointMarker.action = Marker.ADD
            # The coordinate from generateSetpoint() function is assigned to the Marker position
            self.setpointMarker.points = [Point(x=float(self.setpoint[0]), y=float(self.setpoint[1]), z=float(self.setpoint[2]))]
        else:
            # RViz is instructed to Delete the Marker when in Bout Mode
            self.setpointMarker.action = Marker.DELETE
        
        # Publishes the Setpoint where the drone is trying to go.
        self.vis_setpointPub.publish(self.setpointMarker)

    def visualizeForce(self, marker, force, publisher):
        """This draws the APF physics (attraction, repulsion, resultant vector) as 3D arrows."""
        # ARROW Marker depends on 2 Points. First is Tail Point[Fixed to Drone's Position] and Second is Head Point[Position + Force Vector]
        marker.points = [Point(x=0.0, y=0.0, z=0.0), Point(x=float(force[0]), y=float(force[1]), z=float(force[2]))]
        
        # Latest Marker with the exact Wall Time Timestamp
        marker.header.stamp = self.system_clock.now().to_msg()    
        # Publishes the ARROW Marker representing the Force Vector
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
    # Get Wall Clock Time in seconds as a float
    system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
    time_sec = system_clock.now().nanoseconds / 1e9
    # File with Sensor Readings
    sensorFile = f"log/GSL/Field/Sensor/{time_sec:.4f}_cf{cfid}_sensors.csv"
    # File with Bout Readings
    boutFile = f"log/GSL/Field/Bout/{time_sec:.4f}_cf{cfid}_bouts{{}}.csv"

    # Opens the File with Sensor Readings
    logfile = open(sensorFile, 'w')
    sensorWriter = csv.writer(logfile) 
    # Heading of the File with Sensor Readings
    sensorWriter.writerow(["Wall Time", "x(metres)", "y(metres)", "z(metres)", "Gas Conc. L", "Gas Conc. R"])

    # Logs Bout Reading for Left IR Sensors of a Drone cf{cfid}
    boutLoggerL = Logger(boutFile.format("L2"))
    boutLoggerL.writerow("w+", ["Wall Time", "x(metres)", "y(metres)", "z(metres)"])
    # Logs Bout Reading for Right IR Sensors of a Drone cf{cfid}
    boutLoggerR = Logger(boutFile.format("R2"))
    boutLoggerR.writerow("w+", ["Wall Time", "x(metres)", "y(metres)", "z(metres)"])

    node.get_logger().info(f'Logging sensor data to: {sensorFile}')
    node.get_logger().info(f'Logging bout data to: {boutFile}')

# 1. Create the Subscriber FIRST
    # This Subscribes to the Battery Readings on the /cf{cfid}/battery Topic and uses the batteryCheck() Function to process the incoming data.
    node.create_subscription(GenericLogData, "/cf{}/battery".format(cfid), lambda msg: batteryCheck(msg, cf), 10)
    
# 2. Create the Publisher SECOND
    # This only Publishes the Detected Bout Point on the GSL/bouts Topic.
    bout_pub = node.create_publisher(Point, BOUT_TOPIC, 10)
# 3. Initialize the BoutDetector Object THIRD
    # This initializes the BoutDetector Object, which applies the Bout Detection Logic for Sensor Readings from Left and Right IR Sensors and uses bout_pub to Publish the Detected Bout Point.
    bout_detector = [BoutDetector(node, TAU, SENSOR_RATE, BOUT_THRESHOLD, False, bout_pub, cf, boutLoggerL),
                     BoutDetector(node, TAU, SENSOR_RATE, BOUT_THRESHOLD, False, bout_pub, cf, boutLoggerR)]

# 4. Create the Subscription FOURTH
    # This Subscribes to the Sensor Readings on the /cf{}/sgp30 Topic and uses the sensorCallback() Function to process the incoming data.
    node.create_subscription(GenericLogData,SENSOR_TOPIC.format(cfid),lambda msg: sensorCallback(node, msg, [bout_detector, cf, sensorWriter]),10)

# 5. Initialize the MotionController Object FIFTH
    # This initializes the MotionController Object, which calculates the Velocity Commands based on the current Mode[Setpoint or Bout Point].
    motion_controller = MotionController(node, BOUNDS, MAX_LIN_VELOCITY, SWITCHING_TIME, GETFORCES_SERVICE, cf, cfid)

# 6. Instruct the Crazyflie to Takeoff to the Desired Height.
    node.get_logger().info(f"Takeoff for cf{cfid} Started")
    cf.takeoff(targetHeight=0.4, duration=2.0)
    while True:
        rclpy.spin_once(node, timeout_sec=0.0)
        z = cf.position()[2]
        
        if z >= HEIGHT_DESIRED * 0.95:   # e.g. 0.475 m for target=0.5
            node.get_logger().info(f"Takeoff for cf{cfid} Reached z={z:.3f}")
            break
    node.get_logger().info(f"Takeoff for cf{cfid} Completed")

    """MAIN LOOP"""
    update_period = 1.0 / float(UPDATE_RATE)
    def update_cb():
        try:
            # This initiates the update() Function to send Velocity commands
            motion_controller.update()
        except Exception as e:
            node.get_logger().error(f"Error in update timer: {e}")

    timer = node.create_timer(update_period, update_cb)
    # Spin so /clock advances and the timer fires under use_sim_time
    try:
        rclpy.spin(node)
    finally:
        timer.cancel()
        logfile.close()
        cf.land(targetHeight=0.02, duration=3.0)

"""HELPER FUNCTIONS"""
# This checks if the supplied position lies within the defined bounds
def isInBounds(pos, bounds):
    return (bounds[0][0] < pos[0] < bounds[1][0]) and (bounds[0][1] < pos[1] < bounds[1][1]) and (bounds[0][2] < pos[2] < bounds[1][2])

def fetch_param_types(node, cfname):
    """
    Dynamically queries the Crazyflie Server to get Parameter Types.
    """
    # Define Service Clients
    list_client = node.create_client(ListParameters, '/crazyflie_server/list_parameters')
    describe_client = node.create_client(DescribeParameters, '/crazyflie_server/describe_parameters')
    
    # Wait for Services
    if not list_client.wait_for_service(timeout_sec=10.0):
        node.get_logger().warn("Services not available. Proceeding without Parameter Types")
        return {}
    
    # Create a Parameter Types List 
    FIRMWARE_PREFIX = "robot_types.default.firmware_params."
    paramTypeDict = {}
    try:
    # Get List of all Parameter Types
        list_req = ListParameters.Request()
        list_req.depth = ListParameters.Request.DEPTH_RECURSIVE

        future = list_client.call_async(list_req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

        if not future.done() or future.result() is None:
            node.get_logger().warn("Failed to retrieve Parameter List (timeout or empty).")
            return {}
        
        all_names = future.result().result.names
        param_names = [name for name in all_names if name.startswith(FIRMWARE_PREFIX)]
        
        if not param_names:
            node.get_logger().warn(f"No firmware parameters found under '{FIRMWARE_PREFIX}'.")
            return {}
        
    # Describe Parameters to get Types
        describe_req = DescribeParameters.Request()
        describe_req.names = param_names

        future = describe_client.call_async(describe_req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

        if not future.done() or future.result() is None:
            node.get_logger().warn("Failed to retrieve Parameter Descriptions.")
            return {}
        
        # Build the Dictionary without the long prefix [robot_types.default.firmware_params.]
        for param_name, descriptor in zip(param_names, future.result().descriptors):
            clean_name = param_name.replace(FIRMWARE_PREFIX, "")
            paramTypeDict[clean_name] = descriptor.type

        node.get_logger().info(f"Successfully cached {len(paramTypeDict)} Parameter Types.")

    except Exception as e:
        node.get_logger().error(f"Error fetching Param Types: {str(e)}")

# Destroy clients to free up resources
    finally:
        node.destroy_client(list_client)
        node.destroy_client(describe_client)
    
    return paramTypeDict

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("crazyflie_distributed")
    node.get_logger().info("=== APF Agent Node Started ===")
    
# Get CFID 
    cfid_param = node.declare_parameter("cfid", 0).value
    try:
        cfid = int(cfid_param)    
    except (ValueError, TypeError):
        node.get_logger().error(f"Invalid cfid parameter: {cfid_param}. Defaulting to 0.")
        cfid = 0
    
# Get List of IDs and make sure it is a List and not a String
    crazyflies_ids_param = node.declare_parameter("crazyflies_ids", "[]").value
    try:
        if isinstance(crazyflies_ids_param, str):
            # If it is represented as a String convert it to a List
            crazyflies_ids = ast.literal_eval(crazyflies_ids_param)
        else:
            crazyflies_ids = crazyflies_ids_param
        # Ensure all values are integers
        crazyflies_ids = [int(x) for x in crazyflies_ids]
        # DEBUG: Log crazyflies_ids
        node.get_logger().info(f"DEBUG: crazyflies_ids = {crazyflies_ids}")

    except (ValueError, SyntaxError, TypeError) as e:
        node.get_logger().error(f"Failed to parse crazyflies_ids: {e}. Defaulting to [].")
        crazyflies_ids = []
    
# Get List of Initial Positions and make sure it is a List and not a String
    crazyflies_positions_param = node.declare_parameter("crazyflies_positions", "[]").value
    try:
        if isinstance(crazyflies_positions_param, str):
            # If it is represented as a String convert it to a List
            crazyflies_positions = ast.literal_eval(crazyflies_positions_param)
        else:
            crazyflies_positions = crazyflies_positions_param
        # DEBUG: Log crazyflies_positions
        node.get_logger().info(f"DEBUG: crazyflies_positions = {crazyflies_positions}")
    
    except (ValueError, SyntaxError, TypeError) as e:
        node.get_logger().error(f"Failed to parse crazyflies_positions: {e}. Defaulting to [].")
        crazyflies_positions = []

# Create a crazyflies List with IDs and Initial Positions
    crazyflies = []
    for i, id_val in enumerate(crazyflies_ids):
        crazyflies.append({
            "id": id_val,
            "initialPosition": crazyflies_positions[i]
        })
    # DEBUG: Log complete crazyflies list
    node.get_logger().info(f"DEBUG: The complete crazyflies List with IDs and Initial Positions: {crazyflies}")

# Setting up TF and creating Crazyflie object
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node, spin_thread=False)
    
    cfname = None
    initial_position = None

# Iterate through crazyflies List to match cfid with id in the crazyflies List
    for crazyflie in crazyflies:
        if int(crazyflie["id"]) == cfid:
            initial_position = crazyflie["initialPosition"]
            cfname = f"cf{cfid}"
            # DEBUG: Log successful match
            node.get_logger().info(f"DEBUG: Match Found: Configured {cfname} with initial pos {initial_position}")
            break
    # DEBUG: Log if cfid has No Match
    if cfname is None:
        node.get_logger().error(f"CFID {cfid} not found in 'crazyflies_ids'. Available IDs: {[cf['id'] for cf in crazyflies]}")
        rclpy.shutdown()
        return
    
# Fetch Firmware Parameters from crazyflie_server.py
    paramTypeDict = fetch_param_types(node, cfname)
    node.get_logger().info(f"DEBUG: Fetched {len(paramTypeDict)} parameter types")
    
# Verify TF Connection
    try:
       trans = tf_buffer.lookup_transform("world", cfname, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5))
       ts = trans.header.stamp
       z = trans.transform.translation.z
       node.get_logger().info(f"TF Connection Verified: Found transform for {cfname}")

    except Exception as e:
        node.get_logger().warning(f"TF Warning: Transform for {cfname} not yet available: {e}")

# Initialize Crazyflie Object
    cf = Crazyflie(node, cfname, paramTypeDict, tf_buffer)
    node.get_logger().info(f"DEBUG: Crazyflie object created successfully")
    
# Testing if Position() from crazyflie.py works [Remove after Testing]
    try:
        test_pos = cf.position()
        node.get_logger().info(f"DEBUG: TF Lookup Test Successful: {test_pos}")
    except Exception as e:
        node.get_logger().error(f"DEBUG: TF Lookup Test Failed: {e}")
    
# Wait to ensure other Nodes are fully aware of APF_1_1_agent Node
    node.get_logger().info(f"Waiting 2 seconds for system stabilization...")
    wait_dur = Duration(seconds=2.0)
    start = node.get_clock().now()
    while node.get_clock().now() - start < wait_dur:
        rclpy.spin_once(node, timeout_sec=0.1)

# Begin Execution 
    node.get_logger().info(f"Starting Control Loop for {cfname} Node")
    run(node, cf, cfid)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
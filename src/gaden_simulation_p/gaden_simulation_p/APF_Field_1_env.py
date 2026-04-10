#!/usr/bin/env python3
"""
Artificial Potential Field Environment
"""

# Standard Library Imports
import ast
import os

# Third-Party Imports
import numpy as np
import csv
from scipy import ndimage			        # This is for 3D [gaussian_filter()]

# ROS2 Core Imports
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from tf2_ros import Buffer, TransformListener
from rclpy.clock import Clock, ClockType

# ROS2 Message & Service Imports
from gaden_simulation_interfaces.srv import GetForces
from geometry_msgs.msg import Point

"""PARAMETERS"""
D_MIN           = [0.05, 0.20]                          # Minimum Distance to start applying Repulsive Force [wall, traffic] [m]
ROI             = [1.25, 1.0]                           # Radius of Influence [traffic, bout] [m]
MEASUREMENT_EPS = 0.10                                  # [m]
B_PARTICLE_SIZE = 0.5                                   # [m]
X_SOURCE        = np.array([0.75, 0.80, 0.75])          # Position of Gas Source [m]
BOUNDS          = [[0.04,-0.10,0],[3.04,2.90,2.0]]      # Size of the Environment [[min],[max]]; [m]
RESOLUTION      = 0.10                                  # Size of each cell in the grid maps [m]
UPDATE_RATE     = 10                                    # Hz

TOPIC_PREFIX        = "GSL"
BOUT_TOPIC          = f"{TOPIC_PREFIX}/bouts/cf{{}}"
SERVICE_NAME        = f"{TOPIC_PREFIX}/GetForces"
CF_FRAME            = "cf{}"
WORLD_FRAME         = "world"

def run(node, tf_buffer, drone_ids):
# Logging
    # Get Wall Clock Time in seconds as a float
    system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
    time_sec = system_clock.now().nanoseconds / 1e9
    # File for Source Estimation
    logfile = open(f"log/GSL/Env/Sensor_Estimation_{time_sec:.4f}.csv", "w")
    writer = csv.writer(logfile)
    # Heading of the File for Source Estimation
    writer.writerow(["Wall Time", "Estimate_x", "Estimate_y", "Estimate_z", "Error"])
    
    # Create instances of TrafficServer to calculate Repulsive Forces from Walls and Drones.
    traffic = TrafficServer(node, BOUNDS, D_MIN[0], D_MIN[1], 1.25, ROI[0], drone_ids, tf_buffer)
    # Create instance of MapSnaps to generate 2D and 3D Maps
    # Create instance of BoutMap to calculate Attractive Forces towards Bouts.
    bout = BoutMap(node, BOUNDS, RESOLUTION, ROI[1], BOUT_TOPIC, B_PARTICLE_SIZE, 1, drone_ids)
    # Create instance of GetForcesServer to handle service Requests for combined(Repulsion + Attraction) Forces.
    getForcesServer = GetForcesServer(node, SERVICE_NAME, traffic, bout, tf_buffer)

    def timer_callback():
        bout.update()
    
    # Logging
        # Get Wall Clock Time in seconds as a float
        current_time = system_clock.now().nanoseconds / 1e9
        writer.writerow([current_time, *bout.sourceEstimate, bout.sourceEstimateError])
	
    # Create a Timer for periodic updates (every 1/UPDATE_RATE seconds)
    timer_period = 1.0 / UPDATE_RATE 
    timer = node.create_timer(timer_period, timer_callback)

    try:
        # Spin the node to process callbacks, subscriptions, and services
        rclpy.spin(node)
    finally:
        # Safely closes the CSV File when the Node Shutdowns
        logfile.close()

class GetForcesServer:
    """
    ROS2 Service Server that computes and returns the combined repulsive 
    and attractive forces for requesting Agent[Drone] based on TF data.
    """
    def __init__(self, node, name, traffic, bout, tf_buffer):
        self.node = node
        self.service = node.create_service(GetForces, name, self.handleGetForces)
        # The TrafficServer instance to calculate Repulsive Forces from Walls and Drones.
        self.traffic = traffic
        # The BoutMap instance to calculate Attractive Forces towards Bouts.
        self.bout = bout
        # TF Buffer for looking up Drone positions.
        self.tf = tf_buffer

    def handleGetForces(self, request, response):
        try:
            # Locates the Position of the requesting Agent[Drone] using TF wrt the Map Frame.
            tf = self.tf.lookup_transform(WORLD_FRAME,CF_FRAME.format(request.id),RclpyTime())
            pos = tf.transform.translation
            position = np.array([pos.x, pos.y, pos.z])

            # Calculate Repulsive Forces and sends it back in the Response message.
            response.repulsion_x, response.repulsion_y, response.repulsion_z = self.traffic.getForce(position, request.id)
            # Calculate Attractive Forces and sends it back in the Response message.
            response.attraction_x, response.attraction_y, response.attraction_z = self.bout.getForce(position)
        
        except Exception as e:
            self.node.get_logger().warn(f"TF lookup failed: {e}")
            response.repulsion_x, response.repulsion_y, response.repulsion_z = 0.0, 0.0, 0.0
            response.attraction_x, response.attraction_y, response.attraction_z = 0.0, 0.0, 0.0
        
        return response

class TrafficServer:
    """
    Calculates Repulsive Forces from Wall Boundaries and other Agents[Drones] in the environment.
    """
    def __init__(self, node, bounds, wall_dmin, traffic_dmin, wall_dmax, traffic_dmax, traffic_ids, tf_buffer):
        self.node = node
        # Size of the Room 
        self.bounds = bounds
        # Minimum Distance to start applying Repulsive Force from Walls.
        self.wall_dmin = wall_dmin
        # Maximum Distance to apply Repulsive Force from Walls. Beyond this, no force is applied.      
        self.wall_dmax = wall_dmax
        # Minimum Distance to start applying Repulsive Force from other Drones.
        self.traffic_dmin = traffic_dmin
        # Maximum Distance to apply Repulsive Force from other Drones. Beyond this, no force is applied.
        self.traffic_dmax = traffic_dmax
        # List of Drone IDs in the environment to consider for Repulsive Forces.
        self.traffic_ids = traffic_ids
        # TF Buffer for looking up Drone positions.
        self.tf = tf_buffer
        
        # Calculate Force Multipliers
        # Force > 1 for d < d_min
        self.traffic_k = 1 * traffic_dmin**2 / (1/traffic_dmin - 1/traffic_dmax)
        # Force > 1 for d < d_min
        self.wall_k = 1 * wall_dmin**2 / (1/wall_dmin - 1/wall_dmax) 

    # A. This calculates Repulsive Forces for the requesting Agent[Drone] from Drones.
    def getTrafficForce(self, ego_position, ego_id=-1):
        force = np.zeros(3)
        # Small threshold to avoid division by zero
        epsilon = 1e-6
        # If ego_id is -1, skip calculating traffic forces
        if ego_position is None or ego_id == -1:
            return force

        # Loop through all other Agent[Drones]
        for id in self.traffic_ids:
            
            # Skip comparing the requesting Agent[Drone] with itself
            if id == ego_id:
                continue
                
            try:
                # Look up the position of the other Agent[Drone] using TF.
                tf = self.tf.lookup_transform(WORLD_FRAME, CF_FRAME.format(id), RclpyTime())
                # Extract the translation components to get the position of the other Agent[Drone].
                other_position = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
            except Exception:
                continue   
        
            # Calculate the Direction vector from the requesting Agent[Drone] to the other Agent[Drone]
            vector = other_position - ego_position
            # Calculate the Magnitude between the requesting Agent[Drone] and the other Agent[Drone]
            distance = np.linalg.norm(vector)
            
            # Apply the virtual bumper (measurement inaccuracy)
            u_distance = distance - 2 * MEASUREMENT_EPS
            
            # Safety check: Prevent division by zero if drones physically overlap
            if u_distance < epsilon:
                continue
                
            # Push away if the other Agent[Drone] is less than the traffic_dmax(1.25m) from the requesting Agent[Drone].
            if u_distance < self.traffic_dmax:
                force_magnitude = (-0.5*(self.traffic_k*(1/u_distance - 1/self.traffic_dmax)/(u_distance**2))        # exponential term
                                    -0.5*(1-(u_distance-self.traffic_dmin)/(self.traffic_dmax-self.traffic_dmin)))   # linear term
                force += (vector / distance) * force_magnitude
    
        #buffer = force.copy()
        #force[0] += buffer[1] + buffer[2]   # Fx += Fy + Fz
        #force[1] -= buffer[0] - buffer[2]   # Fy -= Fx - Fz
        #force[2] += buffer[0] - buffer[1]   # Fz += Fx - Fy
    
        return force

    # B. This calculates Repulsive Forces for the requesting Agent[Drone] from Walls.
    def getWallForce(self, position):
        force = np.zeros(3)
        
        # X-min boundary (left wall)
        distance = abs(position[0] - self.bounds[0][0]) - MEASUREMENT_EPS
        if distance < self.wall_dmax:
            magnitude = (0.75*self.wall_k*(1/distance - 1/self.wall_dmax)/(distance**2)
                        +0.25*(1-distance-self.wall_dmin)/(self.wall_dmax-self.wall_dmin))
            force += np.array([1, 0, 0]) * magnitude

        # X-max boundary (right wall)
        distance = abs(position[0] - self.bounds[1][0]) - MEASUREMENT_EPS
        if distance < self.wall_dmax:
            magnitude = (0.75*self.wall_k*(1/distance - 1/self.wall_dmax)/(distance**2)
                        +0.25*(1-distance-self.wall_dmin)/(self.wall_dmax-self.wall_dmin))
            force += np.array([-1, 0, 0]) * magnitude

        # Y-min boundary (front wall)
        distance = abs(position[1] - self.bounds[0][1]) - MEASUREMENT_EPS
        if distance < self.wall_dmax:
            magnitude = (0.75*self.wall_k*(1/distance - 1/self.wall_dmax)/(distance**2)
                        +0.25*(1-distance-self.wall_dmin)/(self.wall_dmax-self.wall_dmin))
            force += np.array([0, 1, 0]) * magnitude

        # Y-max boundary (back wall)
        distance = abs(position[1] - self.bounds[1][1]) - MEASUREMENT_EPS
        if distance < self.wall_dmax:
            magnitude = (0.75*self.wall_k*(1/distance - 1/self.wall_dmax)/(distance**2)
                        +0.25*(1-distance-self.wall_dmin)/(self.wall_dmax-self.wall_dmin))
            force += np.array([0, -1, 0]) * magnitude
        
        # Z-min boundary (floor)
        distance = abs(position[2] - self.bounds[0][2]) - MEASUREMENT_EPS
        if distance < self.wall_dmax:
            magnitude = (0.75*self.wall_k*(1/distance - 1/self.wall_dmax)/(distance**2)
                        +0.25*(1-distance-self.wall_dmin)/(self.wall_dmax-self.wall_dmin))
            force += np.array([0, 0, +1]) * magnitude

        # Z-max boundary (ceiling)
        distance = abs(position[2] - self.bounds[1][2]) - MEASUREMENT_EPS
        if distance < self.wall_dmax:
            magnitude = (0.75*self.wall_k*(1/distance - 1/self.wall_dmax)/(distance**2)
                        +0.25*(1-distance-self.wall_dmin)/(self.wall_dmax-self.wall_dmin))
            force += np.array([0, 0, -1]) * magnitude
	
        #buffer = force.copy()
        #force[0] += buffer[1] + buffer[2]   # Fx += Fy + Fz
        #force[1] -= buffer[0] - buffer[2]   # Fy -= Fx - Fz
        #force[2] += buffer[0] - buffer[1]   # Fz += Fx - Fy
        
        return force

    def getForce(self, position, ego_id=-1):
        """Returns the combined Forces [Repulsion from Walls + Repulsion from Drones] for a given Position and Agent[Drone] ID."""
        return self.getWallForce(position) + self.getTrafficForce(position, ego_id)

class MapServer:
    """
    Base class for spatial maps representing environmental data using grids.
    Handles basic diffusion, differentiation, and vector fields (vortices).
    """
    def __init__(self, bounds, resolution, roi, base=None):
        # Size of the Room 
        self.bounds = bounds
        # Resolution of the grid (size of each cell in meters)
        self.resolution = resolution

        self.width = bounds[1][0] - bounds[0][0]
        self.height = bounds[1][1] - bounds[0][1]
        self.depth  = bounds[1][2] - bounds[0][2]

        self.n = int(self.width/self.resolution)
        self.m = int(self.height/self.resolution)
        self.l = int(self.depth/self.resolution)

        self.sigma = roi / self.resolution

        if base is None:
            # If no map was provided, create a giant 3D grid filled with 0.0 as the Base layer.
            self.base = np.full([self.n, self.m, self.l], 0.0)
        else:
            # If a map was provided, make a copy of it to use as the Base layer.
            self.base = base.copy()

        # Processing layers
        self.diffused = np.zeros_like(self.base)

        self.gradients = np.array([np.zeros_like(self.base), np.zeros_like(self.base)])
        self.differentiate()

        self.vorteces = np.zeros_like(self.gradients)
        self.vortexize()

        self.forces = np.zeros_like(self.gradients)
        self.normalize()

    def addParticle(self, position, size, value):
        """Adds a source particle to the environment matrix."""
        r = int(size/self.resolution/2)
        i, j, k = self.cell(position[0]), self.cell(position[1]), self.cell(position[2])
        
        # 3D bounds checking
        i_start = max(0, i - r + 1)
        i_end   = min(self.n, i + r)
        j_start = max(0, j - r + 1)  
        j_end   = min(self.m, j + r)
        k_start = max(0, k - r + 1)
        k_end   = min(self.l, k + r)
        
        # Only proceed if we have valid ranges
        if i_start < i_end and j_start < j_end and k_start < k_end:
            # Mask generation for spherical particle distribution
            x, y, z = np.mgrid[
                i_start - i + r - 1 : i_end - i + r - 1, 
                j_start - j + r - 1 : j_end - j + r - 1, 
                k_start - k + r - 1 : k_end - k + r - 1
            ]
            sphere = x**2 + y**2 + z**2
            mask = sphere < r**2 - 1
            self.base[i_start:i_end, j_start:j_end, k_start:k_end] += mask * value

    def cell(self, coord):
        """Helper to convert spatial coordinates to grid indices."""
        return int(coord/self.resolution)

    def diffuse(self):
        """Applies a 3D Gaussian filter to simulate diffusion."""
        self.diffused = ndimage.gaussian_filter(self.base, sigma=self.sigma, mode='constant', cval=0)

    def differentiate(self):
        """Calculates spatial gradients of the diffused map."""
        self.gradients = np.array(np.gradient(self.diffused, self.resolution))

    def normalize(self):
        """Normalizes the vortex forces to a 0-1 scale."""
        magnitude = np.linalg.norm(self.vorteces, axis=0)
        _min = magnitude.min()
        _max = magnitude.max()

        if _min != _max:
            self.forces = (self.vorteces-_min)/(_max-_min)
        else:
            self.forces = self.gradients.copy()

    def getForce(self, position):
        """Returns the specific normalized force vector at a given 3D position."""
        i, j, k = self.cell(position[0]), self.cell(position[1]), self.cell(position[2])
        
        # 3D bounds validation
        if 0 <= i < self.n and 0 <= j < self.m and 0 <= k < self.l:
            return np.array([self.forces[0, i, j, k], self.forces[1, i, j, k], self.forces[2, i, j, k]])
        else:
            return np.zeros(3)  # Return zero force for out-of-bounds positions

    def vortexize(self, factor=0.75):
        """Converts gradient matrices into a vortex vector field."""
        self.vorteces[0] = self.gradients[0] + self.gradients[1]*factor
        self.vorteces[1] = self.gradients[1] - self.gradients[0]*factor
        self.vorteces[2] = 0.0

        # gx, gy, gz = self.gradients
        # self.vorteces[0] = gx + factor * (gy + gz)
        # self.vorteces[1] = gy - factor * (gx + gz)
        # self.vorteces[2] = gz + factor * (gx - gy)

class BoutMap (MapServer):
    """
    Subclass of MapServer specifically tailored to handle attractive goals or 'bouts'.
    Subscribes to point messages to update internal maps dynamically.
    """
    def __init__(self, node, bounds, resolution, roi, topic, particle_size, particle_value, drone_ids):
        super().__init__(bounds, resolution, roi)
        self.node = node
        self.particle_size = particle_size
        self.particle_value = particle_value
        
        # This Subscribes to 'GSL/bouts/cf' when APF_Field_1_agent.py Publishes a Bout Point, handleInput drops a new Attractive Force Particle onto the 3D grid
        self.subscribers = []
        for i in drone_ids:
            topic = BOUT_TOPIC.format(i)
            sub = node.create_subscription(Point, topic, self.handleInput, 10)
            self.subscribers.append(sub)

        self.sourceEstimate = np.array([99.0, 99.0, 99.0])
        self.sourceEstimateError = 99.0

    def handleInput(self, msg):
        """Callback to add new sources (bouts) based on incoming messages."""
        pos = np.array([msg.x, msg.y, msg.z])
        self.addParticle(pos, self.particle_size, self.particle_value)
        

    
    def update(self):
        """Processes the physics layers frame-by-frame."""
        self.updateSourceEstimate()
        self.differentiate()
        self.vortexize(1)
        self.normalize()

    def updateSourceEstimate(self):
        self.node.get_logger().info("1")
        self.sourceEstimate = np.array(np.unravel_index(self.diffused.argmax(), self.diffused.shape))*self.resolution + self.resolution*0.5
        self.node.get_logger().info("2")
        self.sourceEstimateError = np.linalg.norm(X_SOURCE-self.sourceEstimate)
        self.node.get_logger().info("3")
        
def main(args=None):
    rclpy.init(args=args)
    node = Node("GSL_MapServer")

    # Extract No. of Drones from Launch file
    node.declare_parameter('drone_ids', [1, 2, 3, 4])
    drone_ids_param = node.get_parameter('drone_ids').value
    if isinstance(drone_ids_param, str):
        try:
            drone_ids = ast.literal_eval(drone_ids_param)
        except Exception as e:
            node.get_logger().error(f"Failed to parse drone_ids string: {e}")
            drone_ids = [1, 2, 3, 4]
    else:
        drone_ids = drone_ids_param

    # Debug environment info
    node.get_logger().info(f"PYTHONPATH: {os.environ.get('PYTHONPATH')}")
    node.get_logger().info(f"PATH: {os.environ.get('PATH')}")
    
    # Initialize TF Listeners
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    try:
        run(node, tf_buffer, drone_ids)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
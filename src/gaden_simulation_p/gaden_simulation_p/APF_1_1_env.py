#!/usr/bin/env python3
"""
Artificial Potential Field Environment
"""
# Standard Library Imports
import ast
import os

# Third-Party Imports
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from scipy import ndimage			        # This is for 3D [gaussian_filter()]

# ROS2 Core Imports
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from tf2_ros import Buffer, TransformListener

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup    # ← NEW
from rclpy.executors import MultiThreadedExecutor                                           # ← NEW

# ROS2 Message & Service Imports
from geometry_msgs.msg import Point
from gaden_simulation_interfaces.srv import GetForces

"""PARAMETERS"""
D_MIN               = [0.30, 0.5]    				# Minimum Distance to start applying Repulsive Force [wall, traffic] [m]
ROI                 = [1, 3]                        # Radius of Influence [traffic, bout] [m]
B_PARTICLE_SIZE     = 0.5                           # [m]
X_SOURCE            = np.array([1.5, 3.0, 0.75])    # Position of Gas Source [m]
BOUNDS              = [[0,0,0],[10,6,2.6]]          # Size of the Environment [[min],[max]]; [m]
RESOLUTION          = 0.10                          # Size of each cell in the grid maps [m]
UPDATE_RATE         = 20                            # Hz

TOPIC_PREFIX        = "GSL"
BOUT_TOPIC          = f"{TOPIC_PREFIX}/bouts/cf{{}}"
SERVICE_NAME        = f"{TOPIC_PREFIX}/GetForces"
CF_FRAME            = "cf{}"
WORLD_FRAME         = "map"

def run(node, tf_buffer, executor, drone_ids):
    service_cb_group = ReentrantCallbackGroup()               # ← NEW
    timer_cb_group = MutuallyExclusiveCallbackGroup()         # ← NEW

    # Create instance of TrafficServer to calculate Repulsive Forces from Walls and Drones.
    traffic = TrafficServer(node, BOUNDS, D_MIN[0], D_MIN[1], 1.0, ROI[0], drone_ids, tf_buffer)
    # Create instance of MapSnaps to generate 2D and 3D Maps
    snaps = MapSnaps(node, BOUNDS, RESOLUTION, B_PARTICLE_SIZE, tf_buffer, drone_ids)
    # Create instance of BoutMap to calculate Attractive Forces towards Bouts.
    bout = BoutMap(node, BOUNDS, RESOLUTION, ROI[1], BOUT_TOPIC, B_PARTICLE_SIZE, 1, snaps, drone_ids)  
    # Create instance of GetForcesServer to handle service Requests for combined(Repulsion + Attraction) Forces.
    getForcesServer = GetForcesServer(node, SERVICE_NAME, traffic, bout, tf_buffer, service_cb_group)       # ← NEW

    def timer_callback():
        node.get_logger().debug("Bout update")
        bout.update()

    # Create a Timer for periodic updates (every 1/UPDATE_RATE seconds[0.05s])
    timer_period = 1.0 / UPDATE_RATE
    timer = node.create_timer(timer_period, timer_callback, callback_group=timer_cb_group)      # ← NEW

    # Spin the node to process callbacks, subscriptions, and services
    executor.spin()
    # rclpy.spin(node)

class MapSnaps:
    """
    Generates 2D and 3D Plots for Visualising the Bout Point's Position for Testing how close it is to the Actual Gas Source.
    """
    def __init__(self, node, bounds, resolution, particle_size, tf_buffer, drone_ids):
        self.node = node
        # The 3D physical boundaries of the environment [[min], [max]]
        self.bounds = bounds
        # Size of each cell in the grid maps
        self.resolution = resolution
        self.tf = tf_buffer
        # List of drone IDs in the swarm
        self.drone_ids = drone_ids
        self.SPATIAL_THRESHOLD = particle_size

        # Throttling state
        self.last_bout_pos = None
        self.last_bout_time = 0.0
        self.TEMPORAL_THRESHOLD = 2.0

        self.map_dir = "log/GSL/CFSim/Maps"

    def should_throttle(self, bout_pos):
        """
        This Function is to check the Time Difference and Bout Position between 2 Bout Points in order to avoid 
        generating Multiple 2D and 3D Plots and saving to the Map Folder.
        """
        # Simulation Time in seconds
        bout_time = self.node.get_clock().now().nanoseconds / 1e9
        
        if self.last_bout_pos is not None:
            dist = np.linalg.norm(bout_pos - self.last_bout_pos)
            time_diff = bout_time - self.last_bout_time
            if dist < self.SPATIAL_THRESHOLD and time_diff < self.TEMPORAL_THRESHOLD:
                return True
        return False

    def trigger_snap(self, pos, drone_name, diffused_grid):
        """
        Based on the should_throttle() Function the 2D and 3D Maps will be generated for the Drone that detected the Bout.
        """
        if self.should_throttle(pos):
            self.node.get_logger().debug(f"SnapShot from {drone_name} throttled.")
            return

        self.last_bout_pos = pos.copy()
        self.last_bout_time = self.node.get_clock().now().nanoseconds / 1e9

        self.node.get_logger().info(f"Generating Maps for {drone_name} at Z={pos[2]:.2f}m")
        self.save_maps(pos, drone_name, diffused_grid)

    def save_maps(self, pos, drone_name, diffused_grid):
        """
        This Function generates the 2D and 3D Maps with well defined Visualisation Tools.
        """
        try:
            drone_positions = {}
            for i in self.drone_ids:
                try:
                    tf = self.tf.lookup_transform(WORLD_FRAME, CF_FRAME.format(i), RclpyTime())
                    drone_positions[f"cf{i}"] = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
                except Exception as e:
                    self.node.get_logger().debug(f"TF lookup failed for cf{i}: {e}")

            # Create a Blank Canvas of size 14x14 inches
            fig = plt.figure(figsize=(14, 14))

        # 2D MAP (Heatmap of Bout)
            # Create a 2D Map on the Left side of the 14x6 inches Blank Canvas[1: row, 2: columns, position 1]
            map_2d = fig.add_subplot(1, 2, 1)
            
            # Find closest Z index to the bout height
            z_idx = int(pos[2] / self.resolution)
            grid_depth = diffused_grid.shape[2]
            z_idx = max(0, min(grid_depth - 1, z_idx))
            
            # Extract the 2D cross section from the actual physics grid
            slice_2d = diffused_grid[:, :, z_idx].T
            
            # This generates the 2D Heatmap. extent: Defines the 2D Plot Size as the Room Size 
            cax = map_2d.imshow(
                slice_2d, 
                origin='lower', 
                extent=[self.bounds[0][0], self.bounds[1][0], self.bounds[0][1], self.bounds[1][1]], 
                cmap='viridis'
            )
            # Draw the Latest Bout Position as red 'x' on the 2D Heatmap
            map_2d.scatter(pos[0], pos[1], color='red', marker='x', s=100, label='Latest Bout')
            map_2d.set_title(f"2D Gas Spread at Height: {pos[2]:.2f}m\n(Detected by {drone_name})")
            map_2d.set_xlabel("X [m]")
            map_2d.set_ylabel("Y [m]")
            map_2d.legend()
            fig.colorbar(cax, ax=map_2d, fraction=0.046, pad=0.04, label="Gas Concentration")

        # 3D MAP (Volumetric Scatter)
            # Create a 3D Map on the Right side of the 14x14 inches Blank Canvas[1: row, 2: columns, position 2]
            map_3d = fig.add_subplot(1, 2, 2, projection='3d')
            
            z_floor = self.bounds[0][2]
            x_wall = self.bounds[0][0]
            rad = self.SPATIAL_THRESHOLD / 2.0  
            
            # Creates Sphere in the 3D Plot around the Bout Point
            u_longitude = np.linspace(0, 2 * np.pi, 20)
            v_latitude = np.linspace(0, np.pi, 10)
            x_sphere = pos[0] + rad * np.outer(np.cos(u_longitude), np.sin(v_latitude))
            y_sphere = pos[1] + rad * np.outer(np.sin(u_longitude), np.sin(v_latitude))
            z_sphere = pos[2] + rad * np.outer(np.ones_like(u_longitude), np.cos(v_latitude))
            
            # This generates a Wireframe of a Sphere
            map_3d.plot_wireframe(x_sphere, y_sphere, z_sphere, color='orange', alpha=0.3)

            bout_label = f"Bout [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]"
            # Marks a Point to represent a Bout in the 3D Map
            map_3d.scatter(pos[0], pos[1], pos[2], color='black', marker='o', s=70, depthshade=False, label=bout_label)

            # Creates a Line from the Bout Point to the Z-Plane and X-Plane
            map_3d.plot([pos[0], pos[0]], [pos[1], pos[1]], [z_floor, pos[2]], color='black', linestyle='--', alpha=0.5)
            map_3d.plot([x_wall, pos[0]], [pos[1], pos[1]], [pos[2], pos[2]], color='black', linestyle='--', alpha=0.5)

            # Draw Locked Drone Positions
            colors = ['red', 'blue', 'orange', 'purple']
            for d_name, d_pos in drone_positions.items():
                idx = int(d_name.replace("cf", "")) - 1
                c = colors[idx % len(colors)]
                
                drone_label = f"{d_name} [{d_pos[0]:.2f}, {d_pos[1]:.2f}, {d_pos[2]:.2f}]"
                # Marks a Triangle to represent Drone's Position in the 3D Map
                map_3d.scatter(d_pos[0], d_pos[1], d_pos[2], color=c, marker='^', s=100, label=drone_label)
                
                # Creates a Line from each Triangle(Drone) to the Z-Plane
                map_3d.plot([d_pos[0], d_pos[0]], [d_pos[1], d_pos[1]], [z_floor, d_pos[2]], color=c, linestyle='--', alpha=0.5)

            # Format each Grid Line in X,Y,Z Axis
            map_3d.xaxis.set_major_locator(ticker.MultipleLocator(0.5))
            map_3d.yaxis.set_major_locator(ticker.MultipleLocator(0.5))
            map_3d.zaxis.set_major_locator(ticker.MultipleLocator(0.5))
            map_3d.grid(which='major', color='gray', linestyle='-', linewidth=0.5, alpha=0.5)
            # Rotates the Tick in X,Y Axis
            map_3d.tick_params(axis='x', labelrotation=45)
            map_3d.tick_params(axis='y', labelrotation=-35)
            
            # 3D Limits and Labels
            map_3d.set_xlim(self.bounds[0][0], self.bounds[1][0])
            map_3d.set_ylim(self.bounds[0][1], self.bounds[1][1])
            map_3d.set_zlim(self.bounds[0][2], self.bounds[1][2])
            map_3d.set_title("3D Bout Position")
            map_3d.set_xlabel("X [m]")
            map_3d.set_ylabel("Y [m]")
            map_3d.set_zlabel("Z [m]")
            
            # Clean Legend & Push Outside
            handles, labels = map_3d.get_legend_handles_labels()
            by_label = dict(zip(labels, handles))
            map_3d.legend(by_label.values(), by_label.keys(), loc='center left', bbox_to_anchor=(1.05, 0.5))

        # Save Images to the Map Folder
            plt.tight_layout()
            time_sec = self.node.get_clock().now().nanoseconds / 1e9
            filename = os.path.join(self.map_dir, f"Map_{time_sec:.4f}_{drone_name}_Z{pos[2]:.2f}m.png")
            
            plt.savefig(filename, dpi=150, bbox_inches='tight')
            plt.close(fig)

        except Exception as e:
            self.node.get_logger().error(f"Failed to generate 2D and 3D Map: {e}")

class GetForcesServer:
    """
    ROS2 Service Server that computes and returns the combined repulsive 
    and attractive forces for requesting Agent[Drone] based on TF data.
    """
    def __init__(self, node, name, traffic, bout, tf_buffer, callback_group=None):      # ← NEW
        self.node = node
        self.service = node.create_service(GetForces, name, self.handleGetForces, callback_group=callback_group)        # ← NEW
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
            if distance < epsilon:
                continue
            
            # Push away if the other Agent[Drone] is less than the traffic_dmax(1m) from the requesting Agent[Drone].
            if distance < self.traffic_dmax:
                force_magnitude = -self.traffic_k * (1/distance - 1/self.traffic_dmax) / (distance**2)
                force += (vector / distance) * force_magnitude
    
        # buffer = force.copy()
        # force[0] += buffer[1] + buffer[2]   # Fx += Fy + Fz
        # force[1] -= buffer[0] - buffer[2]   # Fy -= Fx - Fz
        # force[2] += buffer[0] - buffer[1]   # Fz += Fx - Fy
    
        return force
    
    # B. This calculates Repulsive Forces for the requesting Agent[Drone] from Walls.
    def getWallForce(self, position):
        force = np.zeros(3)
        # Small threshold to avoid division by zero
        epsilon = 1e-6
        
        # X-min boundary (left wall)
        distance = abs(position[0] - self.bounds[0][0])
        if distance > epsilon and distance < self.wall_dmax:
            magnitude = self.wall_k * (1/distance - 1/self.wall_dmax) / (distance**2)
            force += np.array([1, 0, 0]) * magnitude

        # X-max boundary (right wall)
        distance = abs(position[0] - self.bounds[1][0])
        if distance > epsilon and distance < self.wall_dmax:
            magnitude = self.wall_k * (1/distance - 1/self.wall_dmax) / (distance**2)
            force += np.array([-1, 0, 0]) * magnitude

        # Y-min boundary (front wall)
        distance = abs(position[1] - self.bounds[0][1])
        if distance > epsilon and distance < self.wall_dmax:
            magnitude = self.wall_k * (1/distance - 1/self.wall_dmax) / (distance**2)
            force += np.array([0, 1, 0]) * magnitude

        # Y-max boundary (back wall)
        distance = abs(position[1] - self.bounds[1][1])
        if distance > epsilon and distance < self.wall_dmax:
            magnitude = self.wall_k * (1/distance - 1/self.wall_dmax) / (distance**2)
            force += np.array([0, -1, 0]) * magnitude
        
        # Z-min boundary (floor)
        distance = abs(position[2] - self.bounds[0][2])
        if distance > epsilon and distance < self.wall_dmax:
            magnitude = self.wall_k * (1/distance - 1/self.wall_dmax) / (distance**2)
            force += np.array([0, 0, +1]) * magnitude

        # Z-max boundary (ceiling)
        distance = abs(position[2] - self.bounds[1][2])
        if distance > epsilon and distance < self.wall_dmax:
            magnitude = self.wall_k * (1/distance - 1/self.wall_dmax) / (distance**2)
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
        self.diffuse()

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
    def __init__(self, node, bounds, resolution, roi, topic, particle_size, particle_value, snaps, drone_ids):
        super().__init__(bounds, resolution, roi)
        self.node = node
        self.particle_size = particle_size
        self.particle_value = particle_value
        self.snaps = snaps
        
        # This Subscribes to 'GSL/bouts/cf' when APF_1_1_agent.py Publishes a Bout Point, handleInput drops a new Attractive Force Particle onto the 3D grid
        self.subscribers = []
        for i in drone_ids:
            drone_name = f"cf{i}"
            topic = BOUT_TOPIC.format(i)
            sub = node.create_subscription(Point, topic, lambda msg, dn=drone_name: self.handleInput(msg, dn), 10)
            self.subscribers.append(sub)

    def handleInput(self, msg, drone_name):
        """Callback to add new sources (bouts) based on incoming messages."""
        pos = np.array([msg.x, msg.y, msg.z])
        self.addParticle(pos, self.particle_size, self.particle_value)
        self.base = ndimage.gaussian_filter(self.base, sigma=self.sigma, mode="constant", cval=0)
        
        # NEW: Update the 3D blur grid immediately before taking the Snap
        self.diffuse()

        self.snaps.trigger_snap(pos, drone_name, self.diffused)

    def update(self):
        """Processes the physics layers frame-by-frame."""
        self.diffuse()
        self.differentiate()
        self.vortexize(1)
        self.normalize()
        
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
    
    executor = MultiThreadedExecutor(num_threads=4)           # ← NEW
    executor.add_node(node)                                   # ← NEW

    try:
        run(node, tf_buffer, executor, drone_ids)             # ← NEW
    finally:
        executor.shutdown()                                   # ← NEW
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
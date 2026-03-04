#!/usr/bin/env python3
"""
Artificial Potential Field Environment
"""
import os
import numpy as np
import rclpy
# ROS2 Imports
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from tf2_ros import Buffer, TransformListener
# Gausian Filtering Imports
from scipy import ndimage			        # This is for 3D [gaussian_filter()]
# from cv2 import getGaussianKernel		    # This is for 2D [getGaussianKernel()]
# Python Visualization Imports
import matplotlib.pyplot as plt
# ROS2 Custom Services
from gaden_simulation_interfaces.srv import GetForces
# ROS2 Messages
from geometry_msgs.msg import Point

"""PARAMETERS"""
D_MIN               = [0.30, 0.5]    				# Minimum Distance to start applying Repulsive Force [wall, traffic] [m]
ROI                 = [1, 3]                        # Radius of Influence [traffic, bout] [m]
B_PARTICLE_SIZE     = 0.5                           # [m]
X_SOURCE            = np.array([1.5, 3.0, 0.75])    # Posiotion of Gas Source [m]
BOUNDS              = [[0,0,0],[10,6,2.6]]          # Size of the Environment [[min],[max]]; [m]
RESOLUTION          = 0.10                          # Size of each cell in the grid maps [m]
UPDATE_RATE         = 20                            # [1/s]
# DYNAMIC_KERNEL    = [0.1,0.8,0.1]                 # This is for 2D [getGaussianKernel()]

TOPIC_PREFIX        = "GSL"
BOUT_TOPIC          = f"{TOPIC_PREFIX}/bouts"
SERVICE_NAME        = f"{TOPIC_PREFIX}/GetForces"
CF_FRAME            = "cf{}"
WORLD_FRAME         = "map"

def draw(fig, ax, attraction):
    ax.clear()
    
    # Define grid dimensions for visualization (adjust as needed for performance)
    n, m, l = 10, 6, 4  # Grid resolution in x, y, z
    res = RESOLUTION
    
    # Get the 3D force matrix (shape: (3, n, m, l))
    forces = attraction.getMatrix(n, m, l, res)
    
    # Create 3D meshgrid for quiver positions
    X, Y, Z = np.meshgrid(np.linspace(0, n * res, n),
                          np.linspace(0, m * res, m),
                          np.linspace(0, l * res, l),
                          indexing='ij')
    
    # Plot 3D quiver (vectors normalized for clarity)
    ax.quiver(X, Y, Z, forces[0], forces[1], forces[2], length=0.1, normalize=True)
    # Scatter the source point
    ax.scatter(*X_SOURCE, color='red', s=30)
    
    # Set 3D axis labels and limits
    ax.set_xlim(0, n * res)
    ax.set_ylim(0, m * res)
    ax.set_zlim(0, l * res)
    ax.set_xlabel("x [0.1 m]")
    ax.set_ylabel("y [0.1 m]")
    ax.set_zlabel("z [0.1 m]")
    ax.set_title("3D Force Field")
    
    # Update the canvas non-blockingly
    fig.canvas.draw()
    fig.canvas.flush_events()

def run(node, tf_buffer):
    # Create instances of TrafficServer to calculate Repulsive Forces from Walls and Drones.
    traffic = TrafficServer(node, BOUNDS, D_MIN[0], D_MIN[1], 1.0, ROI[0], 4, tf_buffer)
    # Create instance of BoutMap to calculate Attractive Forces towards Bouts.
    bout = BoutMap(node, BOUNDS, RESOLUTION, ROI[1], BOUT_TOPIC, B_PARTICLE_SIZE, 1)  
    # Create instance of GetForcesServer to handle service Requests for combined(Repulsion + Attraction) Forces.
    getForcesServer = GetForcesServer(node, SERVICE_NAME, traffic, bout, tf_buffer)

    # Visualization
    plt.ion()
    fig_bout, axs_bout = plt.subplots(1, 3, figsize=(15, 5))
    axs_bout[2] = fig_bout.add_subplot(133, projection='3d')  # Make third subplot 3D
    
    # Separate visualization setup for the main 3D force field (as in original draw)
    fig_main = plt.figure()
    ax_main = fig_main.add_subplot(111, projection='3d')
    ax_main.set_title("3D Force Field")
    
    counter = [0]	 # Initialize counter for visualization updates

    def timer_callback():
        node.get_logger().debug("Bout update")
        bout.update()

        # Visualization logic
        counter[0] += 1
        if counter[0] >= 10:
            # Call BoutMap's plot for detailed subplots
            bout.plot(axs_bout)
            fig_bout.canvas.draw()
            fig_bout.canvas.flush_events()

            # Call original draw for main 3D force field (using bout as attraction)
            draw(fig_main, ax_main, bout)

            counter[0] = 0

    # Create a Timer for periodic updates (every 1/UPDATE_RATE seconds[0.05s])
    timer_period = 1.0 / UPDATE_RATE
    timer = node.create_timer(timer_period, timer_callback)

    # Spin the node to process callbacks, subscriptions, and services
    rclpy.spin(node)

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
    def __init__(self, node, bounds, wall_dmin, traffic_dmin, wall_dmax, traffic_dmax, traffic_n, tf_buffer):
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
        # Number of Drones in the environment to consider for Repulsive Forces.
        self.traffic_n = traffic_n
        # TF Buffer for looking up Drone positions.
        self.tf = tf_buffer

        # Calculate Force Multipliers
        # Force > 1 for d < d_min
        self.traffic_k = 1 * traffic_dmin**2 / (1/traffic_dmin - 1/traffic_dmax)
        # Force > 1 for d < d_min
        self.wall_k = 1 * wall_dmin**2 / (1/wall_dmin - 1/wall_dmax)  
	
    # A. This calculates Repulsive Forces for the requesting Agent[Drone] from Drones.
    def getTrafficForce(self, position, ego_id=-1):
        force = np.zeros(3)
        # Small threshold to avoid division by zero
        epsilon = 1e-6
        # If ego_id is -1, skip calculating traffic forces
        if position is None or ego_id == -1:
            return force

        # Loop through all other Agent[Drones], 1 to 4
        for i in range(self.traffic_n):
            id = i + 1

            # Skip comparing the requesting Agent[Drone] with itself
            if id == ego_id:
                continue

            try:
                # Look up the position of the other Agent[Drone] using TF.
                tf = self.tf.lookup_transform(WORLD_FRAME, CF_FRAME.format(id), RclpyTime())
                # Extract the translation components to get the position of the other Agent[Drone].
                trans = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
            
            except Exception:
                continue
            
            # Calculate the Direction vector from the requesting Agent[Drone] to the other Agent[Drone]
            vector = trans - position
            # Calculate the Magnitude between the requesting Agent[Drone] and the other Agent[Drone]
            distance = np.linalg.norm(vector)
            if distance < epsilon:
                continue
            
            # Push away if the other Agent[Drone] is less than the traffic_dmax(1m) from the requesting Agent[Drone].
            if distance < self.traffic_dmax:
                force_magnitude = -self.traffic_k * (1/distance - 1/self.traffic_dmax) / (distance**2)
                force += (trans / distance) * force_magnitude
    
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
            magnitude = self.traffic_k*(1/distance - 1/self.traffic_dmax)/(distance**2)
            force += np.array([1, 0, 0]) * magnitude

        # X-max boundary (right wall)
        distance = abs(position[0] - self.bounds[1][0])
        if distance > epsilon and distance < self.wall_dmax:
            magnitude = self.traffic_k*(1/distance - 1/self.traffic_dmax)/(distance**2)
            force += np.array([-1, 0, 0]) * magnitude

        # Y-min boundary (front wall)
        distance = abs(position[1] - self.bounds[0][1])
        if distance > epsilon and distance < self.wall_dmax:
            magnitude = self.traffic_k*(1/distance - 1/self.traffic_dmax)/(distance**2)
            force += np.array([0, 1, 0]) * magnitude

        # Y-max boundary (back wall)
        distance = abs(position[1] - self.bounds[1][1])
        if distance > epsilon and distance < self.wall_dmax:
            magnitude = self.traffic_k*(1/distance - 1/self.traffic_dmax)/(distance**2)
            force += np.array([0, -1, 0]) * magnitude
        
        # Z-min boundary (floor)
        distance = abs(position[2] - self.bounds[0][2])
        if distance > epsilon and distance < self.wall_dmax:
            magnitude = self.traffic_k*(1/distance - 1/self.traffic_dmax)/(distance**2)
            force += np.array([0, 0, +1]) * magnitude

        # Z-max boundary (ceiling)
        distance = abs(position[2] - self.bounds[1][2])
        if distance > epsilon and distance < self.wall_dmax:
            magnitude = self.traffic_k*(1/distance - 1/self.traffic_dmax)/(distance**2)
            force += np.array([0, 0, -1]) * magnitude
	
        #buffer = force.copy()
        #force[0] += buffer[1] + buffer[2]   # Fx += Fy + Fz
        #force[1] -= buffer[0] - buffer[2]   # Fy -= Fx - Fz
        #force[2] += buffer[0] - buffer[1]   # Fz += Fx - Fy
        
        return force

    def getForce(self, position, ego_id=-1):
        """Returns the combined Forces [Repulsion from Walls + Repulsion from Drones] for a given Position and Agent[Drone] ID."""
        return self.getWallForce(position) + self.getTrafficForce(position, ego_id)

    def getMatrix(self, n, m, l, resolution):
        """
        Generate a 3D matrix of force vectors sampled across a voxel grid.
        Args:
            n (int): number of grid cells in X
            m (int): number of grid cells in Y
            l (int): number of grid cells in Z
            resolution (float): size of each voxel
        
        Returns:
            np.ndarray: a 4D array of shape (3, n, m, l) where each [i,j,k] stores a 3D force vector.
        """
        grid = np.mgrid[0:n, 0:m, 0:l] * resolution + resolution / 2
        matrix = np.zeros((3, n, m, l))
        
        for i in range(n):
            for j in range(m):
                for k in range(l):
                    pos = np.array([grid[0, i, j, k], grid[1, i, j, k], grid[2, i, j, k]])
                    force_vec = self.getForce(pos)
                    matrix[:, i, j, k] = force_vec
        
        return matrix

    def plot(self, axs, n, m, l, resolution):
        """
        Renders a 3D slice visualization of the traffic forces.
        """
    # 1. Prepare Data
        matrix = self.getMatrix(n, m, l, resolution)
        mid_z = l // 2
        
        X, Y = np.meshgrid(
            np.linspace(0, n * resolution, n), 
            np.linspace(0, m * resolution, m)
        )
        Z = np.full_like(X, mid_z * resolution)

    # 2. Prepare Axis
        axs.cla() 
        
    # 3. Plot Elements
        axs.quiver(
            X, Y, Z,
            matrix[0, :, :, mid_z],
            matrix[1, :, :, mid_z],
            matrix[2, :, :, mid_z],
            length=0.1, normalize=True
        )

    # 4. Format Axes
        axs.set_xlim(0, n * resolution)
        axs.set_ylim(0, m * resolution)
        axs.set_zlim((mid_z - 1) * resolution, (mid_z + 1) * resolution)
        axs.set_xlabel('X (m)')
        axs.set_ylabel('Y (m)')
        axs.set_zlabel('Z (m)')
        axs.set_title('Traffic Force Field (Middle Z Slice, 3D)')

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
    def __init__(self, node, bounds, resolution, roi, topic, particle_size, particle_value=1):
        super().__init__(bounds, resolution, roi)
        self.node = node
        # This Subscribes to 'GSL/bouts' when APF_1_1_agent.py Publishes a Bout Point, handleInput drops a new Attractive Force Particle onto the 3D grid
        self.subscriber = node.create_subscription(Point, topic, self.handleInput, 10)
        
        self.particle_size = particle_size
        self.particle_value = particle_value 

    def handleInput(self, msg):
        """Callback to add new sources (bouts) based on incoming messages."""
        self.addParticle([msg.x, msg.y, msg.z], self.particle_size, self.particle_value)
        self.base = ndimage.gaussian_filter(self.base, sigma=self.sigma, mode="constant", cval=0)
    
    def update(self):
        """Processes the physics layers frame-by-frame."""
        self.diffuse()
        self.differentiate()
        self.vortexize(1)
        self.normalize()

    def getMatrix(self, n, m, l, resolution):
        """Fetches the 3D grid representation of the force field."""
        grid = np.mgrid[0:n, 0:m, 0:l] * resolution + resolution / 2.0
        matrix = np.zeros((3, n, m, l))
        
        for i in range(n):
            for j in range(m):
                for k in range(l):
                    pos = np.array([grid[0, i, j, k], grid[1, i, j, k], grid[2, i, j, k]])
                    force_vec = self.getForce(pos)
                    matrix[:, i, j, k] = force_vec
        return matrix
        
    def plot(self, axs):
        """
        Renders detailed diagnostic views of the BoutMap across 3 subplots.
        axs[0]: Base map
        axs[1]: Diffused map
        axs[2]: 3D Force Field
        """
        # Middle Z plane for 2D visualizations
        mid_z = self.l // 2
        
    # 1. Raw Concentration (Base Map)
        axs[0].cla()
        
        # Plot and label
        axs[0].matshow(self.base[:, :, mid_z].T, origin='lower')
        peak_base = np.unravel_index(np.argmax(self.base[:, :, mid_z]), self.base[:, :, mid_z].shape)
        axs[0].scatter(*peak_base, c="lime")
        axs[0].set_title("Base Map (mid-Z)")
        
    # 2. Diffused Concentration
        axs[1].cla()
        
        # Plot and label
        axs[1].matshow(self.diffused[:, :, mid_z].T, origin='lower')
        peak_diff = np.unravel_index(np.argmax(self.diffused[:, :, mid_z]), self.diffused[:, :, mid_z].shape)
        axs[1].scatter(*peak_diff, c="lime")
        axs[1].set_title("Diffused Map (mid-Z)")
        
    # 3. Full 3D Force Field
        axs[2].cla()
        
        # Setup Grid
        X, Y = np.meshgrid(np.arange(self.n), np.arange(self.m))
        Z = np.full_like(X, mid_z)
        
        # Plot Quiver Elements
        axs[2].quiver(
            X, Y, Z,
            self.forces[0, :, :, mid_z].T,
            self.forces[1, :, :, mid_z].T,
            self.forces[2, :, :, mid_z].T,
            length=0.1, normalize=True
        )
        
        # Set limits and labels
        axs[2].set_xlim(0, self.n)
        axs[2].set_ylim(0, self.m)
        axs[2].set_zlim(mid_z - 1, mid_z + 1)
        axs[2].set_xlabel('X')
        axs[2].set_ylabel('Y')
        axs[2].set_zlabel('Z')
        axs[2].set_title("Bout Force Field (mid-Z, 3D)")
        
def main(args=None):
    rclpy.init(args=args)
    node = Node("GSL_MapServer")

    # Debug environment info
    node.get_logger().info(f"PYTHONPATH: {os.environ.get('PYTHONPATH')}")
    node.get_logger().info(f"PATH: {os.environ.get('PATH')}")
    
    # Initialize TF Listeners
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    try:
        run(node, tf_buffer)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
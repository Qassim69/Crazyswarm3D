#!/usr/bin/env python3
"""
Artificial Potential Field Environment
"""
import os
import time
import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from rclpy.time import Time as RclpyTime
from builtin_interfaces.msg import Time as TimeMsg

from scipy import ndimage			#This is for 3D [gaussian_filter()]
'''from cv2 import getGaussianKernel'''		#This is for 2D [getGaussianKernel()]

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped, Vector3, PointStamped
from gaden_simulation_interfaces.srv import GetForces
from gaden_simulation_interfaces.msg import Bout

# Communication
TOPIC_PREFIX        = "GSL"
BOUT_TOPIC          = f"{TOPIC_PREFIX}/bouts"
SERVICE_NAME        = f"{TOPIC_PREFIX}/getForces"
CF_FRAME            = "cf{}"
WORLD_FRAME         = "world"

D_MIN = [0.25, 0.5]    				# [m] [obstacle, traffic]
ROI = [0.5, 1, 3]                   # [m] radius of influence [obstacle, traffic, bout]
DYNAMIC_KERNEL = [0.1,0.8,0.1]
B_PARTICLE_SIZE = 0.5 # [m]
X_SOURCE = np.array([1.5, 3.0, 0.75])   # [m]
BOUNDS = [[0,0,0],[10,6,2.5]]   		# [[min],[max]]; [m]
RESOLUTION = 0.10                       # [m]
UPDATE_RATE = 5                         # [1/s]

nx, ny, nz = np.diff(BOUNDS, axis=0)[0] / RESOLUTION
OBSTACLE_MATRIX = np.zeros((int(nx), int(ny), int(nz)))
OBSTACLE_MATRIX[0,:,:] = -1.0 
OBSTACLE_MATRIX[-1,:,:] = -1.0 
OBSTACLE_MATRIX[:,0,:] = -1.0 
OBSTACLE_MATRIX[:,-1,:] = -1.0  
OBSTACLE_MATRIX[:,:,0] = -1.0  
OBSTACLE_MATRIX[:,:,-1] = -1.0  
							
MAP_NAMES = ["Obstacle (repulsion)", "Traffic (repulsion)", "Bout (attraction)"]


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
    
    # Set axis limits
    ax.set_xlim(0, n * res)
    ax.set_ylim(0, m * res)
    ax.set_zlim(0, l * res)
    ax.set_title("3D Force Field")
    
    # Update the canvas non-blockingly
    fig.canvas.draw()
    fig.canvas.flush_events()

def staticTransform(node, header, child):
    # Broadcast a transform from world to map, for GADEN - crazyswarm compatability
    bc = StaticTransformBroadcaster(node)
    static_transformStamped = TransformStamped()
    static_transformStamped.header.stamp = node.get_clock().now().to_msg()
    static_transformStamped.header.frame_id = header
    static_transformStamped.child_frame_id = child
    static_transformStamped.transform.translation = Vector3(x=0.0, y=0.0, z=0.0)
    static_transformStamped.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    bc.sendTransform(static_transformStamped)

def run(node, tf_buffer):
    """Initialization"""
    # Logging
    now = time.localtime()
    now_str = str(now[0])+'_'+str(now[1])+'_'+str(now[2])+'_'+str(now[3])+'_'+str(now[4])

    # Broadcast a transform from world to map, for GADEN - crazyswarm compatability
    staticTransform(node, "world", "map")

    # Map instances
    #obstacle = MapServer(BOUNDS, RESOLUTION, ROI[0], OBSTACLE_MATRIX)
    #traffic = TrafficMap(BOUNDS, RESOLUTION, ROI[1], T_PARTICLE_SIZE, -1)
    traffic = TrafficServer(node, BOUNDS, 0.3, 1.0, 0.5, ROI[1], 4, tf_buffer)
    bout = BoutMap(node, BOUNDS, RESOLUTION, ROI[2], UPDATE_RATE, BOUT_TOPIC, B_PARTICLE_SIZE, 1)
    
    #rospy.loginfo("o_kernel: {}, t_kernel: {}, b_kernel: {}".format(obstacle.kernel, traffic.kernel, bout.kernel))
    
    # GetForcesServer instance
    getForcesServer = GetForcesServer(node, SERVICE_NAME, None, traffic, bout, tf_buffer)

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
        # traffic.update()  # Commented out, as in original
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

    # Create a timer for periodic updates (fires every 1/UPDATE_RATE seconds)
    timer_period = 1.0 / UPDATE_RATE  # e.g., 0.2 seconds for UPDATE_RATE=5
    timer = node.create_timer(timer_period, timer_callback)

    # Spin the node to process callbacks, subscriptions, and services
    rclpy.spin(node)

    # Saving map matrices as csv
    # np.savetxt('log/GSL/'+now_str+'_bouts.csv', bout.matrix, delimiter=',')
    # np.savetxt('log/GSL/'+now_str+'_history.csv', history_map.matrix, delimiter=',')

class GetForcesServer:
    def __init__(self, node, name, obstacle, traffic, bout, tf_buffer):
        #self.obstacle = obstacle
        self.node = node
        self.traffic = traffic
        self.bout = bout
        self.tf = tf_buffer
        self.service = node.create_service(GetForces, name, self.handleGetForces)

    def handleGetForces(self, request, response):
        try:
            tf = self.tf.lookup_transform(WORLD_FRAME,CF_FRAME.format(request.id),RclpyTime())  # zero time means latest transform
            pos = tf.transform.translation
            position = np.array([pos.x, pos.y, pos.z])
            
            response.repulsion_x, response.repulsion_y, response.repulsion_z = self.traffic.getForce(position, request.id)
            response.attraction_x, response.attraction_y, response.attraction_z = self.bout.getForce(position)
        
        except Exception as e:
            self.node.get_logger().warn(f"TF lookup failed: {e}")
        return response

class TrafficServer:
    def __init__(self, node, bounds, wall_dmin, wall_dmax, traffic_dmin, traffic_dmax, traffic_n, tf_buffer):
        self.node = node
        self.bounds = bounds
        self.traffic_n = traffic_n
        self.traffic_dmax = traffic_dmax
        
        self.traffic_k = 1 * traffic_dmin**2/(1/traffic_dmin - 1/traffic_dmax)   # Force > 1 for d < d_min
        self.wall_k = 1 * wall_dmin**2/(1/wall_dmin - 1/wall_dmax)   		 # Force > 1 for d < d_min
	
        self.tf = tf_buffer

    '''
    Not used in the code: Use if required
    def getPositions(self):
        pass
    def getForceFromPositions(self, pos1, pos2):
        pass
    ''' 
    def getTrafficForce(self, ego_id=None, position=None):
        force = np.zeros(3)
        epsilon = 1e-6		# Small threshold to avoid division by zero
        if position is not None:
            for i in range(self.traffic_n):
                id = i+1
                try:
                    tf = self.tf.lookup_transform(WORLD_FRAME, CF_FRAME.format(id), RclpyTime())
                    trans = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
                except:
                    continue   
            
                vector = trans - position
                distance = np.linalg.norm(vector)
                if distance < epsilon:  # Skip if agents are too close (avoids div by zero)
                    continue

                force_magnitude = -self.traffic_k*(1/distance - 1/self.traffic_dmax)/(distance**2)
                force += (vector / distance) * force_magnitude
        else:
            positions = []
            for i in range(self.traffic_n):
                id = i+1
                try:
                    tf = self.tf.lookup_transform(CF_FRAME.format(ego_id), CF_FRAME.format(id), RclpyTime())
                    trans = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
                    positions.append(trans)
                except:
                    continue  

                if id != ego_id:
                    distance = np.linalg.norm(trans)
                    if distance < epsilon:  # Skip if agents are too close (avoids div by zero)
                        continue

                    force_magnitude = -self.traffic_k*(1/distance - 1/self.traffic_dmax)/(distance**2)
                    force += (trans / distance) * force_magnitude
    
        buffer = force.copy()
        force[0] += buffer[1] + buffer[2]   # Fx += Fy + Fz
        force[1] -= buffer[0] - buffer[2]   # Fy -= Fx - Fz
        force[2] += buffer[0] - buffer[1]   # Fz += Fx - Fy
    
        return force
    
    def getWallForce(self, position):
        force = np.zeros(3)
        distance = abs(position[0]-self.bounds[0][0])
        magnitude = self.traffic_k*(1/distance - 1/self.traffic_dmax)/(distance**2)
        force += np.array([1, 0, 0]) * magnitude

        distance = abs(position[0]-self.bounds[1][0])
        magnitude = self.traffic_k*(1/distance - 1/self.traffic_dmax)/(distance**2)
        force += np.array([-1, 0, 0]) * magnitude

        distance = abs(position[1]-self.bounds[0][1])
        magnitude = self.traffic_k*(1/distance - 1/self.traffic_dmax)/(distance**2)
        force += np.array([0, 1, 0]) * magnitude

        distance = abs(position[1]-self.bounds[1][1])
        magnitude = self.traffic_k*(1/distance - 1/self.traffic_dmax)/(distance**2)
        force += np.array([0, -1, 0]) * magnitude
	
        distance = abs(position[2] - self.bounds[0][2])					# Z−min (floor)
        magnitude = self.traffic_k*(1/distance - 1/self.traffic_dmax)/(distance**2)
        force += np.array([0, 0, +1]) * magnitude

        distance = abs(position[2] - self.bounds[1][2])					# Z−max (ceiling)
        magnitude = self.traffic_k*(1/distance - 1/self.traffic_dmax)/(distance**2)
        force += np.array([0, 0, -1]) * magnitude
	
        buffer = force.copy()
        force[0] += buffer[1] + buffer[2]   # Fx += Fy + Fz
        force[1] -= buffer[0] - buffer[2]   # Fy -= Fx - Fz
        force[2] += buffer[0] - buffer[1]   # Fz += Fx - Fy
        
        return force

    def getForce(self, position, id=-1):
        return self.getWallForce(position) + self.getTrafficForce(id)

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
        matrix = self.getMatrix(n, m, l, resolution)
        # Plot a 3D quiver slice (middle z for simplicity; adjust slice as needed)
        mid_z = l // 2
        axs.cla()  # Clear previous plot
        # Create 3D meshgrid for the slice (Z constant at mid_z, scaled by resolution)
        X, Y = np.meshgrid(np.linspace(0, n * resolution, n), np.linspace(0, m * resolution, m))
        Z = np.full_like(X, mid_z * resolution)  # Constant Z plane in world coordinates

        # Quiver with all three components (U=X, V=Y, W=Z forces)
        axs.quiver(X, Y, Z,
                   matrix[0, :, :, mid_z],
                   matrix[1, :, :, mid_z],
                   matrix[2, :, :, mid_z],
                   length=0.1, normalize=True)  # Normalize for clarity

        # Set limits and labels
        axs.set_xlim(0, n * resolution)
        axs.set_ylim(0, m * resolution)
        axs.set_zlim((mid_z - 1) * resolution, (mid_z + 1) * resolution)  # Tight Z-range
        axs.set_xlabel('X (m)')
        axs.set_ylabel('Y (m)')
        axs.set_zlabel('Z (m)')
        axs.set_title('Traffic Force Field (Middle Z Slice, 3D)')

class MapServer:
    def __init__(self, bounds, resolution, roi, base=None):
        self.bounds = bounds
        self.resolution = resolution
        self.width = bounds[1][0] - bounds[0][0]
        self.height = bounds[1][1] - bounds[0][1]
        self.depth  = bounds[1][2] - bounds[0][2]

        self.n = int(self.width/self.resolution)
        self.m = int(self.height/self.resolution)
        self.l = int(self.depth/self.resolution)

        self.sigma = roi / self.resolution

        if base is None:
            self.base = np.full([self.n, self.m, self.l], 0.0)
        else:
            self.base = base.copy()

        self.diffused = np.zeros_like(self.base)
        self.diffuse()

        self.gradients = np.array([np.zeros_like(self.base), np.zeros_like(self.base)])
        self.differentiate()

        self.vorteces = np.zeros_like(self.gradients)
        self.vortexize()

        self.forces = np.zeros_like(self.gradients)
        self.normalize()

    def addParticle(self, position, size, value):
        r = int(size/self.resolution/2)
        i, j, k = self.cell(position[0]), self.cell(position[1]), self.cell(position[2])
        
        # 3D bounds checking
        i_start = max(0, i-r+1)
        i_end = min(self.n, i+r)
        j_start = max(0, j-r+1)  
        j_end = min(self.m, j+r)
        k_start = max(0, k-r+1)
        k_end = min(self.l, k+r)
        
        # Only proceed if we have valid ranges
        if i_start < i_end and j_start < j_end and k_start < k_end:
            # Create mask for the valid region only
            x, y, z = np.mgrid[i_start-i+r-1:i_end-i+r-1, 
                            j_start-j+r-1:j_end-j+r-1, 
                            k_start-k+r-1:k_end-k+r-1]
            sphere = x**2 + y**2 + z**2
            mask = sphere < r**2-1
            self.base[i_start:i_end, j_start:j_end, k_start:k_end] += mask * value

    def cell(self, coord):
        return int(coord/self.resolution)  # Helper to transfer coordinates to indices

    def diffuse(self):
        # self.applyKernel(self.kernel, self.base, self.diffused)
        self.diffused = ndimage.gaussian_filter(self.base, sigma=self.sigma, mode='constant', cval=0)

    def differentiate(self):
        self.gradients = np.array(np.gradient(self.diffused, self.resolution))

    def normalize(self):
        magnitude = np.linalg.norm(self.vorteces, axis=0)
        _min = magnitude.min()
        _max = magnitude.max()

        if _min != _max:
            self.forces = (self.vorteces-_min)/(_max-_min)
        else:
            self.forces = self.gradients.copy()

    def getForce(self, position):
        i, j, k = self.cell(position[0]), self.cell(position[1]), self.cell(position[2])
        
        # 3D bounds validation
        if 0 <= i < self.n and 0 <= j < self.m and 0 <= k < self.l:
            return np.array([self.forces[0, i, j, k], self.forces[1, i, j, k], self.forces[2, i, j, k]])
        else:
            return np.zeros(3)  # Return zero force for out-of-bounds positions

    def vortexize(self, factor=0.75):
        gx, gy, gz = self.gradients
        self.vorteces[0] = gx + factor * (gy + gz)
        self.vorteces[1] = gy - factor * (gx + gz)
        self.vorteces[2] = gz + factor * (gx - gy)

class BoutMap (MapServer):
    def __init__(self, node, bounds, resolution, roi, frequency, topic, particle_size, particle_value=1):
        super().__init__(bounds, resolution, roi)
        
        self.node = node
        self.subscriber = node.create_subscription(Point, topic, self.handleInput, 10)
        self.particle_size = particle_size
        self.particle_value = particle_value
        self.dynamic_sigma = roi / resolution 

    def handleInput(self, msg):
        self.addParticle([msg.x, msg.y, msg.z], self.particle_size, self.particle_value)
        self.base = ndimage.gaussian_filter(self.base, sigma=self.dynamic_sigma, mode="constant", cval=0)
    
    def update(self):
        self.diffuse()
        self.differentiate()
        self.vortexize(1)
        self.normalize()

    def getMatrix(self, n, m, l, resolution):
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
        mid_z = self.l // 2  # Middle Z plane
        
        # --- 1) Raw concentration ---
        axs[0].cla()  # Clear previous plot
        axs[0].matshow(self.base[:, :, mid_z].T, origin='lower')
        peak = np.unravel_index(np.argmax(self.base[:, :, mid_z]), self.base[:, :, mid_z].shape)
        axs[0].scatter(*peak, c="lime")
        axs[0].set_title("Base Map (mid-Z)")
        
        # --- 2) Diffused concentration ---
        axs[1].cla()  # Clear previous plot
        axs[1].matshow(self.diffused[:, :, mid_z].T, origin='lower')
        peak = np.unravel_index(np.argmax(self.diffused[:, :, mid_z]), self.diffused[:, :, mid_z].shape)
        axs[1].scatter(*peak, c="lime")
        axs[1].set_title("Diffused Map (mid-Z)")
        
        # --- 3) Full 3D force field (with Z) ---
        axs[2].cla()  # Clear previous plot
        # Create 3D meshgrid for the slice (Z constant at mid_z)
        X, Y = np.meshgrid(np.arange(self.n), np.arange(self.m))
        Z = np.full_like(X, mid_z)  # Constant Z plane
        
        # Quiver with all three components (U=X, V=Y, W=Z forces)
        axs[2].quiver(X, Y, Z,
                      self.forces[0, :, :, mid_z].T,
                      self.forces[1, :, :, mid_z].T,
                      self.forces[2, :, :, mid_z].T,
                      length=0.1, normalize=True)  # Normalize for clarity
        
        # Set limits and labels
        axs[2].set_xlim(0, self.n)
        axs[2].set_ylim(0, self.m)
        axs[2].set_zlim(mid_z - 1, mid_z + 1)  # Tight Z-range
        axs[2].set_xlabel('X')
        axs[2].set_ylabel('Y')
        axs[2].set_zlabel('Z')
        axs[2].set_title("Bout Force Field (mid-Z, 3D)")
        
def main(args=None):
    rclpy.init(args=args)
    node = Node("GSL_MapServer")
    node.get_logger().info(f"PYTHONPATH: {os.environ.get('PYTHONPATH')}")
    node.get_logger().info(f"PATH: {os.environ.get('PATH')}")
    
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    try:
        run(node, tf_buffer)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

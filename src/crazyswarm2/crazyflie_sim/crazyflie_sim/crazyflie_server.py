#!/usr/bin/env python3

"""
A crazyflie server for simulation.

    2022 - Wolfgang Hönig (TU Berlin)
"""

from functools import partial
import importlib

from crazyflie_interfaces.msg import FullState, Hover, Position
from crazyflie_interfaces.srv import GoTo, Land, Takeoff, SetGroupMask
from crazyflie_interfaces.srv import NotifySetpointsStop, StartTrajectory, UploadTrajectory
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import rowan

from std_msgs.msg import String
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
from visualization_msgs.msg import Marker

# import BackendRviz from .backend_rviz
# from .backend import *
# from .backend.none import BackendNone
from .crazyflie_sil import CrazyflieSIL, TrajectoryPolynomialPiece
from .sim_data_types import State


class CrazyflieServer(Node):

    def __init__(self):
        super().__init__('crazyflie_server', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True,)

        self.led_pub = self.create_publisher(Marker, "/visualization_marker", 1)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Turn ROS parameters into a dictionary
        self._ros_parameters = self._param_to_dict(self._parameters)
        self.cfs = {}

        self.world_tf_name = 'world'
        try:
            self.world_tf_name = self._ros_parameters['world_tf_name']
        except KeyError:
            pass
        
        robot_data = self._ros_parameters['robots']
        # Parse robots
        names = []
        initial_states = []
        for cfname in robot_data:
            if robot_data[cfname]['enabled']:
                type_cf = robot_data[cfname]['type']
                # do not include virtual objects
                connection = self._ros_parameters['robot_types'][type_cf].get('connection', 'crazyflie')
                if connection == 'crazyflie':
                    names.append(cfname)
                    pos = robot_data[cfname]['initial_position']
                    initial_states.append(State(pos))

        # initialize backend by dynamically loading the module
        backend_name = self._ros_parameters['sim']['backend']
        module = importlib.import_module('.backend.' + backend_name, package='crazyflie_sim')
        class_ = getattr(module, 'Backend')
        self.backend = class_(self, names, initial_states)

        # initialize visualizations by dynamically loading the modules
        self.visualizations = []
        for vis_key in self._ros_parameters['sim']['visualizations']:
            if self._ros_parameters['sim']['visualizations'][vis_key]['enabled']:
                module = importlib.import_module('.visualization.' +
                                                 str(vis_key),
                                                 package='crazyflie_sim')
                class_ = getattr(module, 'Visualization')
                vis = class_(self,
                             self._ros_parameters['sim']['visualizations'][vis_key],
                             names,
                             initial_states)
                self.visualizations.append(vis)

        controller_name = backend_name = self._ros_parameters['sim']['controller']

        # create robot SIL objects
        for name, initial_state in zip(names, initial_states):
            self.cfs[name] = CrazyflieSIL(
                name,
                initial_state.pos,
                controller_name,
                self.backend.time)

        for name, _ in self.cfs.items():
            pub = self.create_publisher(
                    String,
                    name + '/robot_description',
                    rclpy.qos.QoSProfile(
                        depth=1,
                        durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))

            msg = String()
            msg.data = self._ros_parameters['robot_description'].replace('$NAME', name)
            pub.publish(msg)
            
            self.create_service(SetGroupMask,name + '/set_group_mask', partial(self._setGroupMask_callback, name=name))
            self.create_service(EmptySrv,name + '/emergency', partial(self._emergency_callback, name=name))
            self.create_service(Takeoff,name + '/takeoff', partial(self._takeoff_callback, name=name))
            self.create_service(Land,name + '/land', partial(self._land_callback, name=name))
            self.create_service(GoTo,name + '/go_to', partial(self._go_to_callback, name=name))
            self.create_service(StartTrajectory,name + '/start_trajectory', partial(self._start_trajectory_callback, name=name))
            self.create_service(UploadTrajectory,name + '/upload_trajectory', partial(self._upload_trajectory_callback, name=name))
            self.create_service(NotifySetpointsStop,name + '/notify_setpoints_stop', partial(self._notify_setpoints_stop_callback, name=name))
            
            self.create_subscription(Twist,name + '/cmd_vel_legacy', partial(self._cmd_vel_legacy_changed, name=name),10)
            self.create_subscription(Hover,name + '/cmd_hover', partial(self._cmd_hover_changed, name=name),10)
            self.create_subscription(FullState,name + '/cmd_full_state', partial(self._cmd_full_state_changed, name=name),10)
            self.create_subscription(Position,name + '/cmd_position', partial(self._cmd_position_changed, name=name),10)
            self.create_subscription(EmptyMsg,name + '/cmd_stop', partial(self._cmd_stop_changed, name=name),10)

        # Create services for the entire swarm and each individual crazyflie
        self.create_service(Takeoff, 'all/takeoff', self._takeoff_callback)
        self.create_service(Land, 'all/land', self._land_callback)
        self.create_service(GoTo, 'all/go_to', self._go_to_callback)
        self.create_service(StartTrajectory,'all/start_trajectory',self._start_trajectory_callback)

        # This is the last service to announce.
        # Can be used to check if the server is fully available.
        self.create_service(EmptySrv, 'all/emergency', self._emergency_callback)

        # step as fast as possible
        max_dt = 0.0 if 'max_dt' not in self._ros_parameters['sim'] \
            else self._ros_parameters['sim']['max_dt']
        self.timer = self.create_timer(max_dt, self._timer_callback)
        self.is_shutdown = False

    def on_shutdown_callback(self):
        if not self.is_shutdown:
            self.backend.shutdown()
            for visualization in self.visualizations:
                visualization.shutdown()

            self.is_shutdown = True

    def _timer_callback(self):
        # update setpoint
        states_desired = [cf.getSetpoint() for _, cf in self.cfs.items()]

        # execute the control loop
        actions = [cf.executeController() for _, cf in self.cfs.items()]

        # execute the physics simulator
        states_next = self.backend.step(states_desired, actions)

        # update the resulting state
        for state, (_, cf) in zip(states_next, self.cfs.items()):
            cf.setState(state)

        for vis in self.visualizations:
            vis.step(self.backend.time(), states_next, states_desired, actions)

    def _param_to_dict(self, param_ros):
        """Turn ROS 2 parameters from the node into a dict."""
        tree = {}
        for item in param_ros:
            t = tree
            for part in item.split('.'):
                if part == item.split('.')[-1]:
                    t = t.setdefault(part, param_ros[item].value)
                else:
                    t = t.setdefault(part, {})
        return tree

    def _emergency_callback(self, request, response, name='all'):
        self.get_logger().info(f'[{name}] emergency not yet implemented')

        return response

    def _setGroupMask_callback(self, request, response, name='all'):
        """Service callback to set the group mask of the crazyflie."""
        self.get_logger().info(
            f'[{name}] setGroupMask(group_mask={request.group_mask})'
        )
        cfs = self.cfs if name == 'all' else {name: self.cfs[name]}
        for _, cf in cfs.items():
            cf.setGroupMask(request.group_mask)

        return response

    def _takeoff_callback(self, request, response, name='all'):
        """Service callback to takeoff the crazyflie."""
        duration = float(request.duration.sec) + \
            float(request.duration.nanosec / 1e9)
        self.get_logger().info(
            f'[{name}] takeoff(height={request.height} m,'
            + f'duration={duration} s,'
            + f'group_mask={request.group_mask})'
        )
        cfs = self.cfs if name == 'all' else {name: self.cfs[name]}
        for _, cf in cfs.items():
            cf.takeoff(request.height, duration, request.group_mask)

        return response

    def _land_callback(self, request, response, name='all'):
        """Service callback to land the crazyflie."""
        duration = float(request.duration.sec) + \
            float(request.duration.nanosec / 1e9)
        self.get_logger().info(
            f'[{name}] land(height={request.height} m,'
            + f'duration={duration} s,'
            + f'group_mask={request.group_mask})'
        )
        cfs = self.cfs if name == 'all' else {name: self.cfs[name]}
        for _, cf in cfs.items():
            cf.land(request.height, duration, request.group_mask)

        return response

    def _go_to_callback(self, request, response, name='all'):
        """Service callback to have the crazyflie go to a position."""
        duration = float(request.duration.sec) + \
            float(request.duration.nanosec / 1e9)

        self.get_logger().info(
            """[%s] go_to(position=%f,%f,%f m,
             yaw=%f rad,
             duration=%f s,
             relative=%d,
             group_mask=%d)"""
            % (
                name,
                request.goal.x,
                request.goal.y,
                request.goal.z,
                request.yaw,
                duration,
                request.relative,
                request.group_mask,
            )
        )
        cfs = self.cfs if name == 'all' else {name: self.cfs[name]}
        for _, cf in cfs.items():
            cf.goTo([request.goal.x, request.goal.y, request.goal.z],
                    request.yaw, duration, request.relative, request.group_mask)

        return response

    def _notify_setpoints_stop_callback(self, request, response, name='all'):
        self.get_logger().info(f'[{name}] Notify setpoint stop not yet implemented')
        return response

    def _update_params_callback(self, request, response, name='all'):
        self.get_logger().info(f'[{name}] Updating params: {request.params}')
        
        cfs = self.cfs if name == 'all' else {name: self.cfs[name]}
        
        for _, cf in cfs.items():
            for param in request.params:
                if "ring/solid" in param:
                    self.update_led(name)
                
                elif param == "ring/effect":
                    # Check if parameter exists and get its value
                    effect_param = f"{name}.ring.effect"
                    if self.has_parameter(effect_param):
                        effect_value = self.get_parameter(effect_param).value
                        if effect_value == 0:
                            # Delete LED marker if effect is 0
                            marker = Marker()
                            marker.header.frame_id = name
                            marker.ns = "LED"
                            marker.id = int(name.replace("cf", ""))
                            marker.action = Marker.DELETE
                            self.led_pub.publish(marker)
                
                else:
                    self.get_logger().warn(f"[{name}] Updating param {param} not fully implemented in simulation!")
    
        return response

    def _upload_trajectory_callback(self, request, response, name='all'):
        self.get_logger().info('[%s] Upload trajectory(id=%d)' % (name, request.trajectory_id))

        cfs = self.cfs if name == 'all' else {name: self.cfs[name]}
        for _, cf in cfs.items():
            pieces = []
            for piece in request.pieces:
                poly_x = piece.poly_x
                poly_y = piece.poly_y
                poly_z = piece.poly_z
                poly_yaw = piece.poly_yaw
                duration = float(piece.duration.sec) + \
                    float(piece.duration.nanosec / 1e9)
                pieces.append(TrajectoryPolynomialPiece(
                    poly_x,
                    poly_y,
                    poly_z,
                    poly_yaw,
                    duration))
            cf.uploadTrajectory(request.trajectory_id, request.piece_offset, pieces)

        return response

    def _start_trajectory_callback(self, request, response, name='all'):
        self.get_logger().info(
            '[%s] start_trajectory(id=%d, timescale=%f, reverse=%d, relative=%d, group_mask=%d)'
            % (
                name,
                request.trajectory_id,
                request.timescale,
                request.reversed,
                request.relative,
                request.group_mask,
            )
        )
        cfs = self.cfs if name == 'all' else {name: self.cfs[name]}
        for _, cf in cfs.items():
            cf.startTrajectory(
                request.trajectory_id,
                request.timescale,
                request.reversed,
                request.relative,
                request.group_mask)

        return response

    def _cmd_vel_legacy_changed(self, msg, name=''):
        '''
        Topic update callback.

        Controls the attitude and thrust of the crazyflie with teleop.
        '''
        self.get_logger().info('cmd_vel_legacy not yet implemented')

    def _cmd_position_changed(self, msg, name=''):
        '''
        msg.x, msg.y, msg.z are floats for position
        msg.yaw is float for orientation
        '''
        self.cfs[name].cmdPosition([msg.x, msg.y, msg.z], msg.yaw)

    def _cmd_stop_changed(self, msg, name=''):
        '''
        Handle stop command from external nodes (e.g., crazyflie.py).
        Args:
            msg (std_msgs.msg.Empty): Empty trigger message (no data fields).
            name (str): Crazyflie name (e.g., 'cf1').
        '''
        self.get_logger().info(f"[{name}] Received stop command")
    
        try:
            if name in self.cfs:
                # Forward to simulator backend
                self.cfs[name].cmdStop()
               
                # Optional: Update visualization (e.g., remove LED)
                marker = Marker()
                marker.header.frame_id = name
                marker.ns = "LED"
                marker.id = int(name.replace("cf", ""))  # Extract ID from name
                marker.action = Marker.DELETE
                self.led_pub.publish(marker)
        
            else:
                self.get_logger().warn(f"[{name}] Unknown drone, ignoring stop command")
        
        except Exception as e:
            self.get_logger().error(f"[{name}] Error processing stop command: {str(e)}")

    def _cmd_hover_changed(self, msg, name=''):
        """
        Topic update callback for hover command.

        Used from the velocity multiplexer (vel_mux).
        """
        self.get_logger().info('cmd_hover not yet implemented')

    def _cmd_full_state_changed(self, msg, name):
        q = [msg.pose.orientation.w,
             msg.pose.orientation.x,
             msg.pose.orientation.y,
             msg.pose.orientation.z]
        rpy = rowan.to_euler(q, convention='xyz')

        self.cfs[name].cmdFullState(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
            [msg.acc.x, msg.acc.y, msg.acc.z],
            rpy[2],
            [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
    
    def update_led(self, name):
        marker = Marker()
        marker.header.frame_id = name
        marker.ns = "LED"
        marker.id = int(name.replace("cf", ""))  # Extract ID from name (e.g., cf1 → 1)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 0.2

        # Check if LED parameters exist
        red_param = f"{name}.ring.solidRed"
        green_param = f"{name}.ring.solidGreen"
        blue_param = f"{name}.ring.solidBlue"

        if (self.has_parameter(red_param) and self.has_parameter(green_param) and self.has_parameter(blue_param)):
            r = self.get_parameter(red_param).value
            g = self.get_parameter(green_param).value
            b = self.get_parameter(blue_param).value

            marker.color.r = float(r)
            marker.color.g = float(g)
            marker.color.b = float(b)

            self.led_pub.publish(marker)

def main(args=None):

    rclpy.init(args=args)
    crazyflie_server = CrazyflieServer()
    rclpy.get_default_context().on_shutdown(crazyflie_server.on_shutdown_callback)

    try:
        rclpy.spin(crazyflie_server)
    except KeyboardInterrupt:
        crazyflie_server.on_shutdown_callback()
    finally:
        rclpy.try_shutdown()
        crazyflie_server.destroy_node()

if __name__ == '__main__':
    main()

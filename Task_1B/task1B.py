#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 1B of StrataCobot (SC) Theme (eYRC 2026-27).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          4376
# Author List:		Saksham Kumar Aggarwal
# Filename:		    task1B.py
# Functions:
#			        arm_waypoints.__init__
#			        arm_waypoints.tcp_pose_callback
#			        arm_waypoints.joint_state_callback
#			        arm_waypoints.arm_status_callback
#			        arm_waypoints.switch_controller
#			        arm_waypoints.request_controller_switch
#			        arm_waypoints.publish_zero_joint_velocity
#			        arm_waypoints.publish_unfold_command
#			        arm_waypoints.publish_zero_velocity
#			        arm_waypoints.process_waypoints
#			        main
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /delta_twist_cmds, /delta_joint_cmds ]
#                   Subscribing Topics - [ /tcp_pose_raw, /joint_states, /etc... ]


################### IMPORT MODULES #######################

import rclpy
import sys
from rclpy.node import Node
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
import numpy as np


##################### TASK CONSTANTS #######################

# Tool positions in base_link, in metres, in the order they must be reached. The tool
# stops at each one and holds it for at least two seconds. Copy the signs as they are:
# base_link is the UR7e's own frame, not the Gazebo world's.
waypoints = [
    (-0.4085, -0.5379, 0.1967),   # 1
    (-0.8000, -0.0005, 0.3967),   # 2
    (-0.7430,  0.5280, 0.1967),   # 3
    (-0.4097,  0.5280, 0.1967),   # 4
    (-0.0763,  0.5280, 0.1967),   # 5
]

# The two command interfaces. Only ONE is active at a time; messages to the other are
# accepted and ignored.
servo_ns = '/ur_arm_controller'
twist_controller = 'delta_twist_controller'
joint_controller = 'delta_joint_controller'

# The only frame a twist may be stamped with; any other is refused, not converted. Note
# 'base' is base_link turned through 180 degrees, not another name for it.
base_frame = 'base_link'

# JointJog velocities are matched to these names, in this order.
joint_names = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# What the servo accepts. A command above one of these is dropped WHOLE, not clamped.
cap_linear_mps = 0.025     # magnitude of a twist's linear part
cap_joint_rps = 0.35      # per joint

# Dead-man switch: the arm stops this long after the last message it received.
# The supplied start pose is folded and close to an elbow/wrist singularity.
# Unfold in joint space before using Cartesian twist commands.
unfold_target = np.array([
    0.00,     # shoulder_pan_joint
   -1.20,     # shoulder_lift_joint
   -1.60,     # elbow_joint
   -1.90,     # wrist_1_joint
    1.57,     # wrist_2_joint
    1.57      # wrist_3_joint
])

unfold_kp = 1.0
unfold_max_rps = 0.18
unfold_tolerance = 0.03


##################### CLASS DEFINITION #######################

class ArmWaypoints(Node):

    def __init__(self):
        super().__init__(
            'arm_waypoints_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.Parameter.Type.BOOL,
                True
            )]
        )

        # Publishers
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/delta_twist_cmds',
            10
        )

        self.joint_pub = self.create_publisher(
            JointJog,
            '/delta_joint_cmds',
            10
        )

        # Subscribers
        self.tcp_sub = self.create_subscription(
            PoseStamped,
            '/tcp_pose_raw',
            self.tcp_pose_callback,
            20
        )

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            50
        )

        self.status_sub = self.create_subscription(
            Int32,
            '/arm_status',
            self.arm_status_callback,
            10
        )

        # Controller switching service
        self.switch_cli = self.create_client(
            SwitchController,
            f'{servo_ns}/switch_controller'
        )

        # Control loop
        self.timer = self.create_timer(
            0.05,
            self.process_waypoints
        )

        # Feedback
        self.tcp_pose = None
        self.joint_angles = None
        self.arm_status = None

        # Waypoint state
        self.current_waypoint = 0
        self.holding = False
        self.hold_start_time = None
        self.finished = False

        # The robot must first be unfolded in joint space. Only after that
        # do we switch to Cartesian twist control.
        self.motion_phase = 'unfolding'
        self.switch_future = None

        # Controller state
        self.controller_ready = False

        # Control gains
        self.position_kp = 1.0

        # Arrival threshold
        self.position_tolerance = 0.030

        self.get_logger().info(
            'Arm waypoint controller initialized.'
        )

        # Start in joint servoing so the arm can safely unfold from the
        # folded start pose before any Cartesian command is issued.
        self.switch_controller('joint_controller')


    def tcp_pose_callback(self, data):
        """
        Description:
            Store the current TCP pose.

        Args:
            data (PoseStamped):
                Current TCP pose in base_link.

        Returns:
            None
        """

        self.tcp_pose = data


    def joint_state_callback(self, data):
        """
        Description:
            Store joint positions matched by joint name.

        Args:
            data (JointState):
                Current joint state.

        Returns:
            None
        """

        try:
            self.joint_angles = [
                data.position[data.name.index(joint)]
                for joint in joint_names
            ]

        except ValueError:
            self.joint_angles = None


    def arm_status_callback(self, data):
        """
        Description:
            Store the current arm status.

        Args:
            data (Int32):
                Arm status code.

        Returns:
            None
        """

        if self.arm_status != data.data:
            self.get_logger().info(
                f'Arm status: {data.data}'
            )

        self.arm_status = data.data


    def switch_controller(self, controller):
        """
        Description:
            Activate the requested arm command controller.

        Args:
            controller (str):
                'twist_controller' or 'joint_controller'.

        Returns:
            bool:
                True if switching succeeds.
        """

        if not self.switch_cli.wait_for_service(timeout_sec=20.0):
            self.get_logger().error(
                'Controller switch service unavailable.'
            )
            return False

        request = SwitchController.Request()

        if controller == 'twist_controller':
            request.activate_controllers = [twist_controller]
            request.deactivate_controllers = [joint_controller]

        elif controller == 'joint_controller':
            request.activate_controllers = [joint_controller]
            request.deactivate_controllers = [twist_controller]

        else:
            self.get_logger().error(
                f'Unknown controller: {controller}'
            )
            return False

        request.strictness = SwitchController.Request.STRICT

        future = self.switch_cli.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=10.0
        )

        if not future.done():
            self.get_logger().error(
                'Controller switch timed out.'
            )
            return False

        result = future.result()

        if result is None:
            self.get_logger().error(
                'Controller switch returned no result.'
            )
            return False

        if result.ok:
            self.get_logger().info(
                f'Activated {controller}.'
            )
            self.controller_ready = True
            return True

        self.get_logger().error(
            f'Failed to activate {controller}.'
        )

        return False


    def request_controller_switch(self, controller):
        """
        Description:
            Request a controller switch without blocking the control timer.

        Args:
            controller (str):
                'twist_controller' or 'joint_controller'.

        Returns:
            None
        """

        request = SwitchController.Request()

        if controller == 'twist_controller':
            request.activate_controllers = [twist_controller]
            request.deactivate_controllers = [joint_controller]

        elif controller == 'joint_controller':
            request.activate_controllers = [joint_controller]
            request.deactivate_controllers = [twist_controller]

        else:
            self.get_logger().error(
                f'Unknown controller: {controller}'
            )
            return

        request.strictness = SwitchController.Request.STRICT

        self.switch_future = self.switch_cli.call_async(request)
        self.controller_ready = False

        self.get_logger().info(
            f'Requesting switch to {controller}.'
        )


    def publish_zero_joint_velocity(self):
        """
        Description:
            Publish zero joint velocity to the delta joint controller.

        Args:
            None

        Returns:
            None
        """

        msg = JointJog()
        msg.joint_names = joint_names
        msg.velocities = [0.0] * len(joint_names)

        self.joint_pub.publish(msg)


    def publish_unfold_command(self):
        """
        Description:
            Move the arm toward the documented open joint configuration.

        Args:
            None

        Returns:
            None
        """

        error = unfold_target - np.array(self.joint_angles)

        velocity = unfold_kp * error

        velocity = np.clip(
            velocity,
            -unfold_max_rps,
            unfold_max_rps
        )

        # Do not command joints that are already at their target.
        velocity[np.abs(error) <= unfold_tolerance] = 0.0

        msg = JointJog()
        msg.joint_names = joint_names
        msg.velocities = [float(value) for value in velocity]

        self.joint_pub.publish(msg)


    def publish_zero_velocity(self):
        """
        Description:
            Publish zero Cartesian velocity.

        Args:
            None

        Returns:
            None
        """

        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = base_frame

        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.twist_pub.publish(msg)


    def process_waypoints(self):
        """
        Description:
            Drive the TCP through all required Cartesian waypoints.

        Args:
            None

        Returns:
            None
        """

        # Do nothing until all required feedback is available.
        if self.tcp_pose is None:
            return

        if self.joint_angles is None:
            return

        if self.arm_status is None:
            return

        # Complete an asynchronous controller switch without blocking the
        # 20 Hz control loop.
        if self.switch_future is not None:
            if not self.switch_future.done():
                if self.motion_phase == 'switching':
                    self.publish_zero_joint_velocity()
                return

            result = self.switch_future.result()
            self.switch_future = None

            if result is None or not result.ok:
                self.get_logger().error(
                    'Controller switch failed; stopping motion.'
                )
                self.controller_ready = False
                return

            self.controller_ready = True

            if self.motion_phase == 'switching':
                self.motion_phase = 'waypoints'
                self.get_logger().info(
                    'Unfold complete. Cartesian waypoint motion starting.'
                )

            return

        if not self.controller_ready:
            if self.motion_phase == 'unfolding':
                return
            self.publish_zero_velocity()
            return

        # ---------------------------------------------------------
        # UNFOLDING STATE
        # ---------------------------------------------------------

        if self.motion_phase == 'unfolding':
            error = unfold_target - np.array(self.joint_angles)
            max_error = np.max(np.abs(error))

            if max_error <= unfold_tolerance:
                self.publish_zero_joint_velocity()

                self.get_logger().info(
                    'Arm unfolded successfully. Switching to Cartesian control.'
                )

                self.motion_phase = 'switching'
                self.request_controller_switch('twist_controller')
                return

            self.publish_unfold_command()

            self.get_logger().info(
                f'Unfolding | max joint error = {max_error:.3f} rad',
                throttle_duration_sec=0.5
            )

            return

        # Stop after completing all waypoints.
        if self.finished:
            self.publish_zero_velocity()
            return

        # A latched protective stop is the documented 20-23 range.
        # Ordinary warning states must not prevent motion.
        if 20 <= self.arm_status <= 23:
            self.publish_zero_velocity()

            self.get_logger().error(
                f'Protective stop is latched (arm status {self.arm_status}).'
            )

            return

        current_position = np.array([
            self.tcp_pose.pose.position.x,
            self.tcp_pose.pose.position.y,
            self.tcp_pose.pose.position.z
        ])

        # Move directly toward the official Cartesian waypoint.
        # The arm has already been unfolded in joint space, so no artificial
        # high-Z transit is introduced here.
        official_target = np.array(
            waypoints[self.current_waypoint]
        )

        error = official_target - current_position
        distance = np.linalg.norm(error)

        # ---------------------------------------------------------
        # HOLD STATE
        # ---------------------------------------------------------

        if self.holding:

            # Zero velocity during the required 2 second hold.
            self.publish_zero_velocity()

            elapsed = (
                self.get_clock().now().nanoseconds * 1e-9
                - self.hold_start_time
            )

            if elapsed >= 2.0:

                self.get_logger().info(
                    f'Waypoint {self.current_waypoint + 1} hold complete.'
                )

                self.current_waypoint += 1
                self.holding = False
                self.hold_start_time = None

                if self.current_waypoint >= len(waypoints):

                    self.finished = True

                    self.get_logger().info(
                        'ALL WAYPOINTS COMPLETE.'
                    )

            return

        # ---------------------------------------------------------
        # ARRIVAL CHECK
        # ---------------------------------------------------------

        if distance <= self.position_tolerance:

            self.publish_zero_velocity()

            self.holding = True
            self.hold_start_time = (
                self.get_clock().now().nanoseconds * 1e-9
            )

            self.get_logger().info(
                f'Waypoint {self.current_waypoint + 1} reached. '
                f'Distance = {distance:.4f} m. Holding...'
            )

            return

        # ---------------------------------------------------------
        # POSITION CONTROL
        # ---------------------------------------------------------

        velocity = self.position_kp * error

        velocity_magnitude = np.linalg.norm(velocity)

        # Cap the WHOLE vector rather than each component independently.
        if velocity_magnitude > cap_linear_mps:

            velocity *= (
                cap_linear_mps /
                velocity_magnitude
            )

        # ---------------------------------------------------------
        # POSITION-ONLY TWIST
        # ---------------------------------------------------------
        # Task 1B specifies Cartesian positions, not a target TCP
        # orientation. Do not add angular corrections: they can
        # unnecessarily constrain the Cartesian servo solution and
        # drive the elbow toward a singular configuration.
        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = base_frame

        msg.twist.linear.x = float(velocity[0])
        msg.twist.linear.y = float(velocity[1])
        msg.twist.linear.z = float(velocity[2])

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.twist_pub.publish(msg)

        # Development logging.
        self.get_logger().info(
            f'WP {self.current_waypoint + 1} | '
            f'distance = {distance:.3f} m',
            throttle_duration_sec=0.5
        )

##################### FUNCTION DEFINITION #######################

def main():
    """
    Description:
        Main function for the Task 1B waypoint controller.

    Args:
        None

    Returns:
        None
    """

    rclpy.init(args=sys.argv)

    node = ArmWaypoints()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()


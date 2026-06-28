"""
ROS 2 Multi-Agent MAVROS Interceptor Node.

Uses true Earth-Frame Proportional Navigation (PN) guidance.
Calculates LOS rates to "cut the corner" on circling targets, 
while relying on ArduPilot's native yaw behavior for smooth flight.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from rclpy.qos import qos_profile_sensor_data  

import os
import csv
import time
import math

# Import our strictly tested core logic
from interceptor_core.kinematics import calculate_los_kinematics

class SwarmInterceptorNode(Node):
    """ROS 2 Node for Swarm-Based Guidance without Gazebo."""

    def __init__(self) -> None:
        super().__init__("swarm_interceptor_node")

        # --- STATE VARIABLES ---
        self.intercepted = False
        self.int_pos: tuple[float, float, float] | None = None
        self.int_vel: tuple[float, float, float] | None = None
        self.tgt_pos: tuple[float, float, float] | None = None
        self.tgt_vel: tuple[float, float, float] | None = None

        # --- DRONE 0: INTERCEPTOR SUBSCRIBERS ---
        self.create_subscription(PoseStamped, "/drone0/local_position/pose", self.int_pose_cb, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, "/drone0/local_position/velocity_local", self.int_vel_cb, qos_profile_sensor_data)

        # --- DRONE 1: TARGET SUBSCRIBERS ---
        self.create_subscription(PoseStamped, "/drone1/local_position/pose", self.tgt_pose_cb, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, "/drone1/local_position/velocity_local", self.tgt_vel_cb, qos_profile_sensor_data)

        # --- COMMAND PUBLISHER (Controlling Drone 0) ---
        self.cmd_vel_pub = self.create_publisher(
            Twist, "/drone0/setpoint_velocity/cmd_vel_unstamped", 10
        )

        # --- DATA LOGGING SETUP ---
        self.log_file = "interception_log.csv"
        with open(self.log_file, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp", 
                "int_x", "int_y", "int_z", 
                "tgt_x", "tgt_y", "tgt_z", 
                "dist_to_tgt", "closing_vel",
                "cmd_vx", "cmd_vy", "cmd_vz"
            ])

        # Run the guidance loop at 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.guidance_loop)
        self.get_logger().info(f"PN Guidance Node Initialized. Logging to: {os.path.abspath(self.log_file)}")

    # --- CALLBACKS TO CACHE TELEMETRY DATA ---
    def int_pose_cb(self, msg: PoseStamped) -> None:
        self.int_pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def int_vel_cb(self, msg: TwistStamped) -> None:
        self.int_vel = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)

    def tgt_pose_cb(self, msg: PoseStamped) -> None:
        self.tgt_pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def tgt_vel_cb(self, msg: TwistStamped) -> None:
        self.tgt_vel = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)

    # --- MAIN GUIDANCE LOOP ---
    def guidance_loop(self) -> None:
        """Calculates Proportional Navigation vector and publishes commands."""
        
        if getattr(self, 'intercepted', False):
            return

        if not all([self.int_pos, self.int_vel, self.tgt_pos, self.tgt_vel]):
            return

        try:
            # --- 1. CURRENT RELATIVE POSITION ---
            rx = self.tgt_pos[0] - self.int_pos[0]
            ry = self.tgt_pos[1] - self.int_pos[1]
            rz = self.tgt_pos[2] - self.int_pos[2]
            dist = math.sqrt(rx**2 + ry**2 + rz**2)
            
            if dist <=3.0:
                self.get_logger().info(f"TARGET INTERCEPTED! Distance: {dist:.1f}m. Halting interceptor.")
                self.intercepted = True
                self.cmd_vel_pub.publish(Twist()) # Stop
                return 

            # --- 2. KINEMATICS ---
            closing_vel, los_rate = calculate_los_kinematics(
                self.int_pos, self.tgt_pos, self.int_vel, self.tgt_vel
            )
            
            # If hovering, give it a baseline speed to start the engagement
            effective_vc = closing_vel if closing_vel > 0.1 else 25.0

            # --- 3. PROPORTIONAL NAVIGATION ---
            # Normalized LOS vector
            ux, uy, uz = rx/dist, ry/dist, rz/dist

            # Pure Pursuit velocity component
            pursuit_speed = 12.0
            v_pursuit_x = pursuit_speed * ux
            v_pursuit_y = pursuit_speed * uy
            v_pursuit_z = pursuit_speed * uz

            # Calculate lateral acceleration: a = N * Vc * (Omega x U)
            nav_constant = 3.0
            omega_x, omega_y, omega_z = los_rate
            
            cx = (omega_y * uz) - (omega_z * uy)
            cy = (omega_z * ux) - (omega_x * uz)
            cz = (omega_x * uy) - (omega_y * ux)

            ax = nav_constant * effective_vc * cx
            ay = nav_constant * effective_vc * cy
            az = nav_constant * effective_vc * cz

            # Blend Pure Pursuit + Lateral Acceleration to get the Lead Vector
            # We scale acceleration by a 1.0s lookahead to map it to velocity
            cmd_vx = v_pursuit_x + ax
            cmd_vy = v_pursuit_y + ay
            cmd_vz = v_pursuit_z + az

            # Normalize the final vector to ensure we fly exactly at our top speed (25 m/s)
            # This prevents the drone from trying to break the laws of physics in tight turns
            cmd_mag = math.sqrt(cmd_vx**2 + cmd_vy**2 + cmd_vz**2)
            if cmd_mag > 0.1:
                cmd_vx = (cmd_vx / cmd_mag) * pursuit_speed
                cmd_vy = (cmd_vy / cmd_mag) * pursuit_speed
                cmd_vz = (cmd_vz / cmd_mag) * pursuit_speed

            # --- 4. PUBLISH COMMANDS ---
            cmd_msg = Twist()
            cmd_msg.linear.x = cmd_vx  
            cmd_msg.linear.y = cmd_vy               
            cmd_msg.linear.z = cmd_vz            
            
            # Keep manual yaw zeroed out. ArduPilot will handle it beautifully.
            cmd_msg.angular.x = 0.0
            cmd_msg.angular.y = 0.0
            cmd_msg.angular.z = 0.0

            self.cmd_vel_pub.publish(cmd_msg)
            
            # Log data to CSV 
            with open(self.log_file, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    time.time(),
                    self.int_pos[0], self.int_pos[1], self.int_pos[2],
                    self.tgt_pos[0], self.tgt_pos[1], self.tgt_pos[2],
                    dist, closing_vel, cmd_vx, cmd_vy, cmd_vz
                ])
            
            self.get_logger().info(
                f"Dist: {dist:.1f}m | Vc: {closing_vel:.1f}m/s | Cmd V: ({cmd_vx:.1f}, {cmd_vy:.1f})", 
                throttle_duration_sec=1.0
            )

        except ValueError as e:
            self.get_logger().error(f"Guidance Error: {e}")

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SwarmInterceptorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

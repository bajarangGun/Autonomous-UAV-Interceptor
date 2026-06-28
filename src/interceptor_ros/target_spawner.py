"""
ROS 2 Target VTOL Spawner and Pilot.

Injects a realistic VTOL 3D model into Gazebo and commands it 
to fly a straight trajectory using kinematic state overrides.
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.msg import ModelState
import math

class TargetPilotNode(Node):
    def __init__(self) -> None:
        super().__init__("target_pilot_node")
        
        # Publisher to move the target by constantly setting its state
        self.state_pub = self.create_publisher(ModelState, '/gazebo/set_model_state', 10)
        
        # --- CONFIGURATION ---
        # Change this to any model folder name you have in your Gazebo models path
        # Common ArduPilot models: 'quadplane', 'plane', 'standard_vtol', 'zephyr_delta_wing'
        self.real_model_name = "quadplane" 
        
        # SDF string to include a realistic 3D model instead of a basic shape
        self.target_sdf = f"""
        <?xml version="1.0" ?>
        <sdf version="1.6">
          <model name="target_vtol">
            <pose>50 50 20 0 0 0</pose>
            <include>
              <uri>model://{self.real_model_name}</uri>
            </include>
          </model>
        </sdf>
        """
        
        # Kinematic State of the Target
        self.x = 50.0
        self.y = 50.0
        self.z = 20.0  # Cruising altitude of 20 meters
        self.vx = 0.0
        self.vy = -10.0 # Flying sideways at 10 m/s
        
        # Loop rate (50 Hz) - Fast enough to override gravity smoothly
        self.dt = 1.0 / 50.0 

        self.spawn_target()
        self.timer = self.create_timer(self.dt, self.fly_target)

    def spawn_target(self) -> None:
        client = self.create_client(SpawnEntity, '/spawn_entity')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Gazebo spawn service...')
            
        req = SpawnEntity.Request()
        req.name = 'target_vtol'
        req.xml = self.target_sdf
        
        future = client.call_async(req)
        self.get_logger().info(f"Spawned real {self.real_model_name} VTOL at (50, 50, 20).")

    def fly_target(self) -> None:
        # Update position kinematically
        self.x += self.vx * self.dt
        self.y += self.vy * self.dt
        
        # Publish the new position directly to Gazebo's physics engine
        msg = ModelState()
        msg.model_name = 'target_vtol'
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z
        
        # Keep the nose pointed in the direction of travel
        # (Assuming the model's nose is aligned with the X-axis)
        yaw = math.atan2(self.vy, self.vx)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        
        msg.twist.linear.x = self.vx
        msg.twist.linear.y = self.vy
        
        self.state_pub.publish(msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetPilotNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

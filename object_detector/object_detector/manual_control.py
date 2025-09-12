#!/usr/bin/env python3

import asyncio
import threading
import uvloop
import queue
import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from mavsdk import System
from mavsdk.offboard import OffboardError

class DroneNode(Node):
    def __init__(self):
        super().__init__('drone_node')
        self.drone = System()
        self.command_queue = queue.Queue()  # Thread-safe queue for commands
        self.loop = uvloop.new_event_loop()  # New uvloop asyncio event loop for MAVSDK
        self.mavsdk_thread = threading.Thread(target=self.run_mavsdk_loop, daemon=True)

        # Control inputs (with thread locks)
        self.control_lock = threading.Lock()
        self.roll = 0.0
        self.pitch = 0.0
        self.throttle = 0.5  # Initial throttle at mid position
        self.yaw = 0.0

        # ROS2 Publishers
        self.telemetry_pub = self.create_publisher(
            Float32MultiArray, 'drone/telemetry/velocity_ned', 10)
        self.position_pub = self.create_publisher(
            PoseStamped, 'drone/position_ned', 10)
        self.flight_mode_pub = self.create_publisher(
            Float32MultiArray, 'drone/telemetry/flight_mode', 10)

        # Start MAVSDK async loop in a separate thread
        self.mavsdk_thread.start()
        self.get_logger().info("DroneNode initialized")

        # Initialize Pygame in main thread
        pygame.init()
        pygame.display.set_mode((200, 200))  # Small window for keyboard focus
        self.keyboard_timer = self.create_timer(0.05, self.check_keyboard_input)  # 20Hz keyboard check

    def run_mavsdk_loop(self):
        """Run the MAVSDK asyncio event loop in a separate thread."""
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.run_mavsdk())

    def check_keyboard_input(self):
        """Handle keyboard inputs for manual control in main thread."""
        with self.control_lock:
            self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_q):
                self.get_logger().info("Quit requested, shutting down...")
                raise KeyboardInterrupt 
            
        keys = pygame.key.get_pressed()
        value = 1.5  # Control sensitivity
        with self.control_lock:
            if keys[pygame.K_LEFT]:
                self.pitch = -value
            elif keys[pygame.K_RIGHT]:
                self.pitch = value
            if keys[pygame.K_UP]:
                self.roll = value
            elif keys[pygame.K_DOWN]:
                self.roll = -value
            if keys[pygame.K_w]:
                self.throttle = min(1.0, self.throttle + 0.05)
            elif keys[pygame.K_s]:
                self.throttle = max(0.0, self.throttle - 0.05)
            if keys[pygame.K_a]:
                self.yaw = -value
            elif keys[pygame.K_d]:
                self.yaw = value

    async def print_flight_mode(self):
        """Publish flight mode to ROS2 topic."""
        try:
            async for flight_mode in self.drone.telemetry.flight_mode():
                flight_mode_msg = Float32MultiArray()
                flight_mode_msg.data = [float(flight_mode.value)]
                self.flight_mode_pub.publish(flight_mode_msg)
                self.get_logger().info(f"FlightMode: {flight_mode}")
                break
        except Exception as e:
            self.get_logger().error(f"Flight mode error: {str(e)}")

    async def manual_control_drone(self):
        """Send manual control inputs to the drone."""
        while True:
            with self.control_lock:
                roll, pitch, throttle, yaw = self.roll, self.pitch, self.throttle, self.yaw
            try:
                await self.drone.manual_control.set_manual_control_input(
                    float(roll), float(pitch), float(throttle), float(yaw))
                self.get_logger().debug(f"Control Inputs: roll={roll}, pitch={pitch}, throttle={throttle}, yaw={yaw}")
            except Exception as e:
                self.get_logger().error(f"Manual control error: {str(e)}")
            await asyncio.sleep(0.1)  # 10Hz control rate

    async def publish_telemetry(self):
        """Publish telemetry data to ROS2 topics."""
        try:
            async for odom in self.drone.telemetry.position_velocity_ned():
                # Publish velocity
                vel_msg = Float32MultiArray()
                vel_msg.data = [odom.velocity.north_m_s, odom.velocity.east_m_s, odom.velocity.down_m_s]
                self.telemetry_pub.publish(vel_msg)

                # Publish position
                pos_msg = PoseStamped()
                pos_msg.header.stamp = self.get_clock().now().to_msg()
                pos_msg.header.frame_id = "ned"
                pos_msg.pose.position.x = float(odom.position.north_m)
                pos_msg.pose.position.y = float(odom.position.east_m)
                pos_msg.pose.position.z = float(odom.position.down_m)
                self.position_pub.publish(pos_msg)
        except Exception as e:
            self.get_logger().error(f"Telemetry error: {str(e)}")

    async def run_mavsdk(self):
        """Main MAVSDK async logic with manual control and reconnection."""
        while True:
            try:
                await self.drone.connect(system_address="udpin://0.0.0.0:14540")
                self.get_logger().info("Waiting for drone to connect...")
                async for state in self.drone.core.connection_state():
                    if state.is_connected:
                        self.get_logger().info("-- Connected to drone!")
                        break

                self.get_logger().info("Waiting for global position estimate...")
                async for health in self.drone.telemetry.health():
                    if health.is_global_position_ok and health.is_home_position_ok:
                        self.get_logger().info("-- Global position estimate OK")
                        break

                # Start telemetry and manual control tasks
                asyncio.ensure_future(self.publish_telemetry())
                asyncio.ensure_future(self.manual_control_drone())

                # Special command handling
                while True:
                    keys = pygame.key.get_pressed()
                    if keys[pygame.K_r]:
                        async for landed in self.drone.telemetry.landed_state():
                            if landed:
                                await self.drone.action.arm()
                                self.get_logger().info("-- Armed")
                            break
                    elif keys[pygame.K_l]:
                        async for in_air in self.drone.telemetry.in_air():
                            if in_air:
                                self.get_logger().info("-- Landing")
                                await self.drone.action.land()
                            break
                    elif keys[pygame.K_i]:
                        await self.print_flight_mode()
                    await asyncio.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"MAVSDK error: {str(e)}")
                self.get_logger().info("Attempting to reconnect in 5 seconds...")
                await asyncio.sleep(5)  # Wait before retrying

def main():
    rclpy.init()
    node = DroneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down...")
    finally:
        # Stop MAVSDK event loop
        if node.loop.is_running():
            node.loop.call_soon_threadsafe(node.loop.stop)
            node.mavsdk_thread.join(timeout=2.0)
        # Clean up Pygame and ROS2
        pygame.quit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
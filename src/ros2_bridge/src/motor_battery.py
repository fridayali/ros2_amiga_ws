import argparse
import asyncio
from pathlib import Path
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import BatteryState
import rclpy
from rclpy.node import Node
from farm_ng.canbus.packet import MotorState
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig
from farm_ng.core.events_file_reader import proto_from_json_file

class MotorStatePublisher(Node):
    def __init__(self, service_config: EventServiceConfig):
        super().__init__("motor_state_publisher")
        
        # Publishers
        self.motor_state_pub = self.create_publisher(Float32MultiArray, "/motor_state", 10)
        self.battery_state_pub = self.create_publisher(BatteryState, "/battery_state", 10)

        # Service config
        self.service_config = service_config

    async def run(self):
        client = EventClient(self.service_config)
        subscription = self.service_config.subscriptions[0]
        
        async for event, msg in client.subscribe(subscription, decode=True):
            # Unpack the motor states
            print(type(msg))
            motors = []
            for motor in msg.motors:
                motors.append(MotorState.from_proto(motor))
            
            # Extract the motor temperatures and voltages
            motor_temperatures = [motor.temperature for motor in motors]
            motor_voltages = [motor.voltage for motor in motors]

            # Publish motor state
            self.publish_motor_state(motor_temperatures)

            # Publish battery state (with average temperature and voltage)
            self.publish_battery_state(motor_temperatures, motor_voltages)

    def publish_motor_state(self, motor_temperatures):
        """Publish motor temperatures as a Float32MultiArray."""
        motor_state_msg = Float32MultiArray()
        motor_state_msg.data = motor_temperatures
        self.motor_state_pub.publish(motor_state_msg)
        self.get_logger().info(f"Published motor temperatures: {motor_temperatures}")

    def publish_battery_state(self, motor_temperatures, motor_voltages):
        """Publish battery state using the average motor temperature and average voltage."""
        avg_temp = sum(motor_temperatures) / len(motor_temperatures)  # Average motor temperature
        avg_voltage = sum(motor_voltages) / len(motor_voltages)  # Average motor voltage
        
        battery_state_msg = BatteryState()
        
        battery_state_msg.header.stamp = self.get_clock().now().to_msg()
        battery_state_msg.voltage = avg_voltage  # Set the average motor voltage
        battery_state_msg.current = 10.0  # Example, update as needed
        battery_state_msg.charge = 50.0   # Example, update as needed
        battery_state_msg.capacity = 100.0  # Example, update as needed
        battery_state_msg.percentage = avg_voltage  # Example, update as needed
        battery_state_msg.power_supply_status = 2  # Example, update as needed
        battery_state_msg.power_supply_health = 1  # Example, update as needed
        battery_state_msg.power_supply_technology = 2  # Example, update as needed
        battery_state_msg.temperature = avg_temp  # Set the average motor temperature

        self.battery_state_pub.publish(battery_state_msg)
        self.get_logger().info(f"Published battery state with average temperature: {avg_temp}°C and average voltage: {avg_voltage}V")

async def main(service_config_path: Path) -> None:
    rclpy.init()
    config: EventServiceConfig = proto_from_json_file(service_config_path, EventServiceConfig())
    node = MotorStatePublisher(config)

    try:
        await node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Publish motor state and battery state.")
    parser.add_argument("--service-config", type=Path, required=True, help="Path to the service config.")
    args = parser.parse_args()

    asyncio.run(main(args.service_config))

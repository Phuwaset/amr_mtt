#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity

class EntitySpawner(Node):
    def __init__(self):
        super().__init__('entity_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()

    def send_request(self, file_path, name, x, y, z):
        self.req.name = name
        self.req.xml = open(file_path, 'r').read()
        self.req.pose.position.x = float(x)
        self.req.pose.position.y = float(y)
        self.req.pose.position.z = float(z)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    spawner = EntitySpawner()
    response = spawner.send_request(
        '/home/sphuwaset_ros/amr_mtt/src/amr_mtt/amr_mtt_bot/models/training_box/model.sdf',
        'training_box', 1.0, 0.0, 0.5)
    spawner.get_logger().info(f'Result: {response}')
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

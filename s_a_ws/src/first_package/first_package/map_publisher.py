import rclpy
import json
from rclpy.node import Node

from std_msgs.msg import String

class MapPublisher(Node):
    def __init__(self):
        super().__init__("map_publisher")
        self.publisher = self.create_publisher(String, 'json_read', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        with open('/home/marek/School/Ing - Kybernetika/2.ZS/NMVR/nmvr/s_a_ws/src/first_package/first_package/maps/map.json', 'r') as json_file:
            # self.get_logger().info('JSON file opened')
            self.walls = []
            data = json.load(json_file)
            if not data == None and len(data) > 0:
                msg = String()
                msg.data = json.dumps(data)
                self.publisher.publish(msg)
                # self.get_logger().info("Publishing data from MapPublisher.")
            json_file.close()
            # self.get_logger().info('JSON file closed')
      
def main(args=None):
    rclpy.init(args=args)
    map_publisher = MapPublisher()
    rclpy.spin_once(map_publisher)    
    rclpy.shutdown()
    map_publisher.destroy_node()

if __name__ == '__main__':
    main()
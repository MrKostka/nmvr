import rclpy 
import json
from rclpy.node import Node
from std_msgs.msg import String

class MapSubscriber(Node):
    def __init__(self):
        super().__init__('map_subscriber')
        self.subscription = self.create_subscription(
            String,
            'json_write',
            self.listener_callback,
            10)
        self.subscription
        
    def listener_callback(self, msg):
        with open('/home/marek/School/Ing - Kybernetika/2.ZS/NMVR/nmvr/f_a_ws/src/first_package/first_package/maps/map.json', 'w') as json_file:
            self.get_logger().info('JSON file opened')
            # self.get_logger().info(msg.data)            
            data = msg.data
            data = json.loads(data)
            # data = data[1:-1]
            # json.dump(data, json_file)
            json_file.write(msg.data)
            json_file.close()
            self.get_logger().info('JSON file closed')            

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MapSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
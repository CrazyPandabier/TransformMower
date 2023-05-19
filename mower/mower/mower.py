import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Polygon
from PIL import Image

class Mower(Node):
    def __init__(self):
        super().__init__('mower')
        self.get_logger().info('Hi from mower.')
        self.odom_callback = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', 1)
        self.max_x = 0
        self.max_y = 0
        self.min_x = 0
        self.min_y = 0
        self.polygon = Polygon()

    def odom_callback(self, msg:Odometry):
        self.polygon.points.append(msg.pose.pose.position)

        if msg.pose.pose.position.x > self.max_x:
            self.max_x = msg.pose.pose.position.x
        if msg.pose.pose.position.y > self.max_y:
            self.max_y = msg.pose.pose.position.y
        if msg.pose.pose.position.x < self.min_x:
            self.min_x = msg.pose.pose.position.x
        if msg.pose.pose.position.y < self.min_y:
            self.min_y = msg.pose.pose.position.y


        map = OccupancyGrid()
        map.header.frame_id = 'map'
        map.header.stamp = self.get_clock().now().to_msg()
        map.info.resolution = 0.05
        map.info.width = int((self.max_x - self.min_x) / map.info.resolution) + 10
        map.info.height = int((self.max_y - self.min_y) / map.info.resolution) +10
        map.info.origin.position.x = 0.0
        map.info.origin.position.y = 0.0
        map.info.origin.position.z = 0.0
        map.info.origin.orientation.x = 0.0
        map.info.origin.orientation.y = 0.0
        map.info.origin.orientation.z = 0.0
        map.info.origin.orientation.w = 1.0

        img = Image.new('L', (map.info.width, map.info.height))
        for i in range(map.info.width * map.info.height):
            map.data.append(-1)
            img.putpixel((i % map.info.width, i // map.info.width), 255)
        


        for point in self.polygon.points:
            grid_x = int((point.x - self.min_x)/ map.info.resolution)
            grid_y = int((point.y - self.min_y)/ map.info.resolution)
            index = grid_y * map.info.width + grid_x
            map.data[index] = 100  # Assuming 100 represents occupied
            img.putpixel((grid_x, grid_y), 0)

        self.map_publisher.publish(map)
        img.save('map.png')

        #create and save map.yaml
        with open('map.yaml', 'w') as f:
            f.write('image: map.png\n')
            f.write('resolution: {}\n'.format(map.info.resolution))
            f.write('origin: [{}, {}, 0.0]\n'.format(map.info.origin.position.x, map.info.origin.position.y))
            f.write('negate: 0\n')
            f.write('occupied_thresh: 0.65\n')
            f.write('free_thresh: 0.196\n')


            
    

def main():
    rclpy.init()
    node = Mower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

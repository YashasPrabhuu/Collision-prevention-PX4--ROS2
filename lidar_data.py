import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
from px4_msgs.msg import ObstacleDistance
import math
import numpy as np

class lidarPublisher(Node):
    '''Intialization of node,creating publisher and subscriber for data'''
    def __init__(self):
        super().__init__('Lidar_publisher')
        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(ObstacleDistance, '/fmu/in/obstacle_distance', 10)
        self.timer = self.create_timer(0.10, self.lidar_callback) #Publishing msgs at 10Hz its the max as we are limited by the frequency at which the LiDAR produces the scan data
        self.subscription_ = self.create_subscription(LaserScan, '/scan', self.scan_data, qos_profile)#Also the LiDAR data publishes at 10Hz.
        self.Laser_data_filter = []
        self.window_size = 5 #Window size for MVG ,BY increase the size of the window you will smoothen the data further but missout the real values"
        
    '''Function to obtain scandata of LiDAR that you get by subscribing to the /scan topic ''' 

    def scan_data(self, msg):
        if msg.ranges:
            self.scan_data = msg.ranges
            for i in range(len(self.scan_data)):
                if math.isnan(self.scan_data[i]): 
                    self.scan_data[i] = 12  # Converting nan values to max range of the lidar in meters

            # self.get_logger().info(f'scandataafternan:{self.scan_data}')
            # self.Laser_data_filter = self.Laser_data_filter[:455]
            # self.get_logger().info(f'scandatacappedto455:{len(self.Laser_data_filter)}')
            self.Laser_data_filter = [int(value * 100) for value in  self.scan_data]  #Convert to cm and to int as required by the PX4msg type.
            self.remove_data = self.process_data(self.Laser_data_filter) #Function call for the processed
            self.final_data=np.array(self.remove_data, dtype=np.uint16) #Changing the data tyoe uint16 as required by the PX4msg type.

            self.get_logger().info(f'lidardata:{(self.final_data)}')
        else:
            self.get_logger().warning('Data not received on /scan topic.')

    '''Processing the LIDAR data
       The obtained data from scan topic is of length :(455-457).To reduce this data to 72 we will 1st cap the length to 455 and then remove 23 values.
       We then find the min value of the distance values in increment of 6. That is 455-23 = 432, when found min of 6 values we get 432/6 = 72.
       The length of the array is reduced to 72.As per the PX4 requirement.And then we are calling the MVG-Moving average filter to smoothen the data out.'''

    def process_data(self,lst):
        num_to_remove = 23
        lst = lst[:455]
        step = len(lst) // num_to_remove  # Compute step size for removal
        data_after_removal=[val for i, val in enumerate(lst) if (i + 1) % step != 0]
        # self.get_logger().info(f'length after rmoval 23 points=432:{len(data_after_removal)}')
        reseized_value = []
        #For loop to find the min of 6 value
        for i in range(0,len(data_after_removal),6):
            value = data_after_removal[i:i+6]
            min_value = min(value)
            reseized_value.append(min_value)

        #Calling median filter to smoothen the data
        smooth_data = self.MVG_filter(reseized_value)
            
        return smooth_data
    #MVG filter to smoothen the data
    def MVG_filter(self, data):
        if len(data) < self.window_size:
            return None # Return original data if not enough points for averaging
        
        filtered_data = []
        for i in range(len(data)):
            start = max(0, i - self.window_size // 2)
            end = min(len(data), i + self.window_size // 2 + 1)
            filtered_data.append(int(sum(data[start:end]) / (end - start)))
           
        if len(filtered_data) == 72:
            # self.get_logger().info(f'Distance measured: {filtered_data}')
            return filtered_data
            

        else: 
            self.get_logger().warn('Length of the LiDAR is less than 72')
            return None         

    '''Format of the PX4 msg should be published on the /fmu/in/obstacle_distance topic according to the px4 document'''
    
    def lidar_callback(self):
        if len(self.Laser_data_filter) > 0: 
            msg = ObstacleDistance()
            msg.timestamp = (self.get_clock().now().to_msg().sec * 1000000) + (self.get_clock().now().to_msg().nanosec // 1000)
            msg.frame = 12
            msg.sensor_type = 0
            msg.increment = 5.0
            msg.min_distance = 0
            msg.max_distance = 1200
            msg.angle_offset = 0.0
            msg.distances =  self.final_data

            # self.get_logger().info(f'Distance measured: {msg.distances}')
            self.publisher_.publish(msg)
        else:
            self.get_logger().warning('No laser data received yet; skipping publish.')

#Calling the main class to prevent the node from dying 
def main(args=None):
    rclpy.init(args=args)
    node = lidarPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

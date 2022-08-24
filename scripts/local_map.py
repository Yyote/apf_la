#!/usr/bin/env python3
from numpy import mat
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
import math
import random


class Point:
    def __init__(self, x=0, y=0, z=0):
        """
        :param x:
        :param y:
        :param z:
        """
        self.x = x
        self.y = y
        self.z = z
        self.scale = 1.0


def frange(x, y, jump):
    while x < y:
        yield x
        x += jump


class OccupancyGridCreator():
    def __init__(self, width: int, height: int, origin: Pose, resolution):
        #super().__init__("local_map", allow_undeclared_parameters=True,
        #                 automatically_declare_parameters_from_overrides=True)

        self.grid_map = OccupancyGrid()
        self.grid_map.header.frame_id = "base_link"
        self.grid_map.header.stamp = rospy.Time.now()
        self.grid_map.info.width = width
        self.grid_map.info.height = height

        self.grid_map.info.origin = origin
        self.grid_map.info.resolution = resolution

        self.obstacles_added = False

        self.laser_scan = LaserScan()
        self.empty_grid = list()

        self.grid_map_center_in_coords = [0.0, 0.0]
        
        self.empty_grid = list()

        for i in range(width * height):
            self.empty_grid.append(0)
            self.grid_map.data.append(0)
        

        self.map_publisher = rospy.Publisher("/local_map", OccupancyGrid, queue_size=10) # HOOK

        # self.create_publisher(OccupancyGrid, "local_map", qos_profile_sensor_data)

        # self.create_subscription(LaserScan,"/scan",self.laser_scan_clb, qos_profile=qos_profile_sensor_data)
        rospy.Subscriber("/scan", LaserScan, self.laser_scan_clb, queue_size=10)
        

        # self.create_timer(0.1, self.map_loop)
        # self.timer = rospy.Timer(rospy.Duration(0.1), self.map_loop) # HOOK
        while not rospy.is_shutdown():
            self.map_loop()

    def world_to_map(self, x: int, y: int): # HOOK
        """
        Get map coordinates
        """
        mx = int((x - self.grid_map.info.origin.position.x) /
                 self.grid_map.info.resolution)
        my = int((y - self.grid_map.info.origin.position.y) /
                 self.grid_map.info.resolution)
        return [mx, my]

    def get_index(self, x: int, y: int):
        """
        Get index in map
        """
        return x + y * self.grid_map.info.width

    def laser_scan_clb(self, msg:LaserScan):

        self.laser_scan = msg

    def add_obstacle(self, cost):
        angle = self.laser_scan.angle_min
        laser_number = 0
        while angle <= self.laser_scan.angle_max:
            laser_len = self.laser_scan.ranges[laser_number]
            if (laser_len != math.inf and laser_len != math.nan and laser_len != 0.0):
                point = Point(laser_len * math.cos(angle), laser_len * math.sin(angle), 0)

                if ((point.x < (self.grid_map.info.width / 2) * self.grid_map.info.resolution and point.x > -(self.grid_map.info.width / 2) * self.grid_map.info.resolution) and
                        (point.y < (self.grid_map.info.height / 2) * self.grid_map.info.resolution and point.y > -(self.grid_map.info.height / 2) * self.grid_map.info.resolution)):
                    xm, ym = self.world_to_map(point.x, point.y)
                    idx = self.get_index(xm, ym)
                    if idx < self.grid_map.info.width * self.grid_map.info.height:
                        self.grid_map.data[idx] = cost

            laser_number = laser_number + 1
            angle = angle + self.laser_scan.angle_increment

    def clear_center_with_circle(self, radius):
        self.add_obstacle(
            self.obstacle_creator.obstacle_shape_circle([0, 0], radius), 0)

    def map_loop(self):
        if (len(self.laser_scan.ranges)!=0):
            self.add_obstacle(100)
            self.map_publisher.publish(self.grid_map)
            print("published local map")
        self.empty_grid = list()
        for i in range(self.grid_map.info.width *self.grid_map.info.height):
            self.empty_grid.append(0)
        print(f"data lenght {len(self.empty_grid)}")

        self.grid_map.data = self.empty_grid
        # print(f"empty map {self.empty_grid}")

def main():
    rospy.init_node("local_map")
    width = 50
    height = 50
    resolution = 0.1
    origin = Pose()
    origin.position.x = (- width * resolution) / 2
    origin.position.y = (- height * resolution) / 2

    node = OccupancyGridCreator(width, height, origin, resolution)
    rospy.spin()

    node.destroy_node()
    rospy.shutdown()

if __name__ == "__main__":
    main()    
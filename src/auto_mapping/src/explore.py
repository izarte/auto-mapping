#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped

from random import randrange
from move import ClientMoveBase
from math import sqrt, pow 

class Explore:
    def __init__(self):
        self.resolution = 0
        self.origin_x = 0
        self.origin_y = 0
        self.width = 0
        self.height = 0

        self.rand_point = {
            'x': 0,
            'y': 0,
            'value': 0,
            'uknowns': 0,
            'obstacles': 0
        }

        map_data = rospy.wait_for_message('/map', OccupancyGrid)
        self.get_info(map_data)
        print(self.resolution)
        print(self.origin_x)
        print(self.origin_y)
        print(self.width)
        print(self.height)
        self.client = ClientMoveBase()
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.read_odom)
        rospy.Subscriber("/map", OccupancyGrid, self.read_map)

    def get_info(self, data):
        self.resolution = data.info.resolution
        self.origin_x = data.info.origin.position.x
        self.origin_y = data.info.origin.position.y
        self.width = data.info.width
        self.height = data.info.height

    def read_odom(self, data):
        self.actual_x = data.pose.pose.position.x
        self.actual_y = data.pose.pose.position.y

    def read_map(self, data):
        rand_points = {
            'n': 0, 
            'points': []
            }

        # while valid is False:
        while rand_points['n'] < 10:
            map_rand = randrange(len(data.data))
            self.map = data.data[map_rand]

            if self.map != -1 and self.map <= 0.2 and self.is_edge(data, map_rand):
                # valid = True
                rand_points['n'] += 1
                rand_points['points'].append(self.rand_point)
        print(rand_points)
        (x, y) = self.get_best_point(rand_points['points'])

        print(x, y)
        result = self.client.moveTo(float(x), float(y))

    def is_edge(self, data, pos):
        
        unknowns = 0
        obstacles = 0
        
        for i in range(-3, 4):
            for j in range(-3, 4):
                neighbor = i * self.width + j
                try:
                    if data.data[pos + neighbor] == -1:
                        unknowns += 1
                    elif data.data[pos + neighbor] > 0.65:
                        obstacles += 1
                except IndexError:
                    pass

        if unknowns > 0:
            row = pos / self.width
            col = pos % self.width
            x = col * self.resolution + self.origin_x  # column * resolution + origin_x
            y = row * self.resolution + self.origin_y  
            self.rand_point['x'] = x
            self.rand_point['y'] = y
            self.rand_point['value'] = data.data[pos]
            self.rand_point['uknowns'] = unknowns
            self.rand_point['obstacles'] = obstacles
            return True
        else:
            return False

    def get_best_point(self, rand_points):
        max_point = {
            'distance': 99999,
            'x': 0,
            'y': 0,
            'value': 0,
            'uknowns': 0,
            'obstacles': 0
        }

        for i in range(len(rand_points)):
            dist = self.get_dist(rand_points[i]['x'], rand_points[i]['y'])
            rand_points[i]['distance'] = dist
            if (rand_points[i]['obstacles'] > max_point['obstacles']):
                max_point = rand_points[i]
            elif (rand_points[i]['obstacles'] == max_point['obstacles']):
                if (rand_points[i]['distance'] < max_point['distance']):
                    max_point = rand_points[i]
            # if (dist < max_point['distance']):
            #     max_point = rand_points[i]
            #     max_point['distance'] = dist
            # elif (dist == max_point['distance']):
            #     if (rand_points[i]['uknowns'] < max_point['uknowns']):
            #         max_point = rand_points[i]
            #         max_point['distance'] = dist
        return (max_point['x'], max_point['y'])

    def get_dist(self, x, y):
        return (sqrt(pow(x, 2) + pow(y, 2)))


def main():
    """ The main() function """
    rospy.init_node('explore', log_level=rospy.DEBUG)
    # rospy.init_node('explore')
    Explore()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException
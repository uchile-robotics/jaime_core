#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

import rospy
import rospkg
import cv2


class GenObstaclesMap(object):
    """
    """

    def __init__(self):

        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()

        # get the file path for rospy_tutorials
        pkg_path = rospack.get_path('uchile_maps')

        self.obstacle_color = rospy.get_param("~obstacle_color", 215)
        original_filename = rospy.get_param("~original_filename", "map_full.pgm")
        new_filename = rospy.get_param("~new_filename", "map_obstacles.pgm")

        self.original_fullfilename = pkg_path + "/maps/" + original_filename
        self.new_fullfilename = pkg_path + "/maps/" + new_filename

    def do_work(self):

        # load map
        original_map = cv2.imread(self.original_fullfilename, 0)

        # gen map
        new_map = original_map
        new_map[new_map != self.obstacle_color] = 255
        new_map[new_map == self.obstacle_color] = 0

        # save
        cv2.imwrite(self.new_fullfilename, new_map)


if __name__ == '__main__':
    try:
        rospy.init_node('gen_obstacles_map')
        handler = GenObstaclesMap()
        handler.do_work()
    except rospy.ROSInterruptException:
        print("Stopping node due to rospy shutdown signal.")

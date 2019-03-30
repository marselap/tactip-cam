#!/usr/bin/env python
import numpy as np
from numpy import linalg as LA
from multi_agent_simulation.msg import IndividualPositions, Contains, EntOcc
import rospy
from occupancy_entropy_plot import Aggent_Occupancy_Entropy
import sys
import time
from matplotlib import path
import matplotlib.pyplot as plt

class contains:
    def __init__(self):

        '''
        Initalize Data
        '''

        self.AOE = Aggent_Occupancy_Entropy()
        self.agents = self.AOE.agents
        self.x_min_dist = self.AOE.x_min_dist #distribtuion limits
        self.x_max_dist = self.AOE.x_max_dist
        self.y_min_dist = self.AOE.y_min_dist
        self.y_max_dist = self.AOE.y_max_dist
        self.delta = self.AOE.delta
        self.all_positions_tuple = self.AOE.all_positions_tuple
        self.center_dict = {}
        self.dim_x = 0
        self.dim_y = 0
        self.ent_list = [0]*self.agents
        for i in range(0, self.agents):
            name = 'agent%s' % (i + 1)
            self.center_dict[name] = 0.0
        '''
        Subscribers and publisher
        '''
        self.agent_dictionary = {}
        self.contains_dict = {}
        for i in range(0, self.agents):
            name = 'agent%s' % (i + 1)
            self.agent_dictionary[name] = [0, 0]
            rospy.Subscriber('/' + name, IndividualPositions, self.individual_callback, queue_size=1, buff_size=2**24)

    def individual_callback(self, msg):
        '''
        DO CALCULATION
        '''




def main():
    while not rospy.is_shutdown():
         '''
         DO STUFF/PUBLISH DATA
         '''


if __name__ == '__main__':
    rospy.init_node('contains')
    C = contains()
    C.main()

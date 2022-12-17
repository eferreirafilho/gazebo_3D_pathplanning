#! /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import pickle
import os, rospkg

class PRM(object):
        def __init__(self):
                rospack = rospkg.RosPack()
                self.path_to_package = os.path.join(rospack.get_path("hector_quadrotor_demo"))
                self.load_data()
                self.test_milestone_visibility()
                self.save_networkx_graph()
                
        def load_data(self):
                self.G = pickle.load(open(self.path_to_package + '/map_files/milestones_roadmap_prm.pickle', 'rb'))
                self.np_point_cloud_xyz = np.load(self.path_to_package + '/map_files/point_cloud_prm.npy')

        def check_collision(self,points):
                COLLISION_THRESHOLD = 3.5
                for i in range(points.shape[1]):
                        for j in self.np_point_cloud_xyz:
                                dist_point_to_pointcloud = np.linalg.norm(points[:,i]  - j)
                                if dist_point_to_pointcloud < COLLISION_THRESHOLD:
                                        return False
                return True                                 

        def test_milestone_visibility(self):
                for i in self.G:
                        NODE_PROXIMITY_THRESHOLD = 6
                        N_POINTS_BETWEEN_MILESTONES = 10
                        print('Veryfing node: ', i)
                        for j in self.G:
                                if i != j:
                                        dist = np.linalg.norm(np.asarray(nx.get_node_attributes(self.G, 'pos')[i])  - np.asarray(nx.get_node_attributes(self.G, 'pos')[j]))
                                        if dist < NODE_PROXIMITY_THRESHOLD:
                                                print('Nodes are close')
                                                points_x=np.linspace(nx.get_node_attributes(self.G, 'pos')[i][0],nx.get_node_attributes(self.G, 'pos')[j][0],N_POINTS_BETWEEN_MILESTONES)
                                                points_y=np.linspace(nx.get_node_attributes(self.G, 'pos')[i][1],nx.get_node_attributes(self.G, 'pos')[j][1],N_POINTS_BETWEEN_MILESTONES)
                                                points_z=np.linspace(nx.get_node_attributes(self.G, 'pos')[i][2],nx.get_node_attributes(self.G, 'pos')[j][2],N_POINTS_BETWEEN_MILESTONES)
                                                points = np.array([points_x,points_y,points_z])
                                                if(self.check_collision(points)):
                                                        print('No collision - add node to roadmap')
                                                        self.G.add_edge(i,j)
                                                        
        def save_networkx_graph(self):
                pickle.dump(self.G, open(self.path_to_package + '/map_files/prm_graph.pickle', 'wb'))                
          

if __name__ == "__main__":
        PRM()
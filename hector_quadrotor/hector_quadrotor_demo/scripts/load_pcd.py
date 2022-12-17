#! /usr/bin/env python

import open3d
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from mpl_toolkits.mplot3d import Axes3D
import pickle

class PointCloud(object):
        def __init__(self):
                N_SAMPLES=1000
                self.load_pdc_file()
                self.compute_pcd_bounds()
                self.sample_point(self.lower_bound,self.upper_bound)
                self.samples_x=[]
                self.samples_y=[]
                self.samples_z=[]
                self.milestones=[]
                self.sample_n_points(N_SAMPLES)
                self.add_milestones_to_roadmap()
                self.save_roadmap()
                self.init_figure()
                # self.plot_samples()
                self.plot_milestones()
                self.plot_pcd()
                plt.show()
                
        def load_pdc_file(self):
                # Load the PCD file
                pcd_file = ("/home/edson_20_04/hector_noetic/src/hector_quadrotor_noetic/hector_gazebo/hector_gazebo_worlds/Media/models/sg27_subsampled2.pcd")
                self.pcd = open3d.io.read_point_cloud(pcd_file)
                self.point_cloud_xyz = np.asarray(self.pcd.points)
                np.save('/home/edson_20_04/hector_noetic/src/hector_quadrotor_noetic/hector_quadrotor/hector_quadrotor_demo/map_files/point_cloud_prm.npy', self.point_cloud_xyz)
        
        def vizualize_pcd(self):
                open3d.visualization.draw_geometries([self.pcd])
                
        def compute_pcd_bounds(self):
                self.upper_bound=np.max(self.point_cloud_xyz, axis=0)
                self.lower_bound=np.min(self.point_cloud_xyz, axis=0)
                self.lower_bound[0]=-20 # adjust x axis
                self.upper_bound[0]=10 # adjust x axis

                self.lower_bound[2]=-17 # adjust z axis
                self.upper_bound[2]=-5 # adjust z axis

                                
        def sample_point(self,upper_bound,lower_bound):
                return (upper_bound-lower_bound)*np.random.rand(1)+lower_bound
        
        def sample_n_points(self,N):
                for i in range(N):
                        self.samples_x = np.append(self.samples_x,self.sample_point(self.upper_bound[0],self.lower_bound[0]))
                        self.samples_y = np.append(self.samples_y,self.sample_point(self.upper_bound[1],self.lower_bound[1]))
                        self.samples_z = np.append(self.samples_z,self.sample_point(self.upper_bound[2],self.lower_bound[2]))
                        
        def init_figure(self):
                self.fig = plt.figure(figsize=(12, 12))
                self.ax = self.fig.add_subplot(projection='3d')
                self.ax = plt.axes(projection='3d')
                plt.xlabel('x')
                plt.ylabel('y')
                plt.title('PRM milestones and point cloud')
        
        def plot_samples(self):
                self.ax.scatter3D(self.samples_x, self.samples_y, self.samples_z, c=self.samples_z, cmap='autumn')

        def plot_milestones(self):
                self.ax.scatter3D(list(list(zip(*self.milestones))[0]), list(list(zip(*self.milestones))[1]), list(list(zip(*self.milestones))[2]), c=list(list(zip(*self.milestones))[2]), cmap='autumn')

        def plot_pcd(self):
                PLOT_INTERVAL=50
                self.ax.scatter3D(self.point_cloud_xyz[::PLOT_INTERVAL,0], self.point_cloud_xyz[::PLOT_INTERVAL,1], self.point_cloud_xyz[::PLOT_INTERVAL,2], c=self.point_cloud_xyz[::PLOT_INTERVAL,2], cmap='bone');
                
        def check_collision(self,point_checked):
                SAFETY_DISTANCE=3.5
                for xyz in self.point_cloud_xyz:
                        dist = np.linalg.norm(point_checked - xyz)
                        if dist<SAFETY_DISTANCE:
                                return True
                        
        def add_milestones_to_roadmap(self):
                self.G = nx.Graph()
                for i in range(len(self.samples_x)):
                        point_checked = np.array([self.samples_x[i],self.samples_y[i],self.samples_z[i]])
                        print('Checking point: ' + str(i))
                        if not self.check_collision(point_checked):
                                print('Collision not detected - adding milestone')
                                self.G.add_node(i, pos=(self.samples_x[i],self.samples_y[i],self.samples_z[i]))
                                self.milestones.append([self.samples_x[i],self.samples_y[i],self.samples_z[i]])

                print('Nodes added to roadmap: ' + str(self.G.nodes))
                print('Number of nodes: ' + str(self.G.number_of_nodes()))
                
        def save_roadmap(self):
                pickle.dump(self.G, open('/home/edson_20_04/hector_noetic/src/hector_quadrotor_noetic/hector_quadrotor/hector_quadrotor_demo/map_files/milestones_roadmap_prm.pickle', 'wb'))

if __name__ == "__main__":
        map = PointCloud()
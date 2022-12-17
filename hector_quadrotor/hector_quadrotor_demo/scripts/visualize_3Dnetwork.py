#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from mpl_toolkits.mplot3d import Axes3D
import pickle
import plotly.graph_objects as go
import os, rospkg

class Vizualize3Dnetwork(object):
        def __init__(self):
                rospack = rospkg.RosPack()
                self.path_to_package = os.path.join(rospack.get_path("hector_quadrotor_demo"))
                self.load_data()
                self.prm_nodes = nx.get_node_attributes(self.G, 'pos')

                self.fig = plt.figure(figsize=(10,7))
                self.ax = Axes3D(self.fig)
                
                self.prm_nodes = np.array(list(self.prm_nodes.values()))
                # self.network_plot_3D(graph=self.G,graph_node_color='red',graph_lines_color='black',graph_size=2)
                self.network_plot_3D(graph=self.solution_graph,graph_node_color='gray',graph_lines_color='blue',graph_size=10)
                self.plot_pcd()
                plt.show()
                                
        def load_data(self):
                self.G = pickle.load(open(self.path_to_package +'/map_files/prm_graph.pickle', 'rb'))
                self.solution_graph = pickle.load(open(self.path_to_package + '/map_files/solution_graph.pickle', 'rb'))
                self.np_point_cloud_xyz = np.load(self.path_to_package + '/map_files/point_cloud_prm.npy')
                print(self.solution_graph.nodes)
                                  
        def network_plot_3D(self,graph,graph_node_color,graph_lines_color,graph_size):

                pos = nx.get_node_attributes(graph, 'pos')
                
                NODE_SIZE_MULTIPLIER=10

                # 3D network plot
                with plt.style.context(('ggplot')):
                        # Loop on the pos dictionary to extract the x,y,z coordinates of each node
                        for key, value in pos.items():
                                xi = value[0]
                                yi = value[1]
                                zi = value[2]
                                
                                self.ax.scatter(xi, yi, zi, s = NODE_SIZE_MULTIPLIER*graph_size, c=graph_node_color, edgecolors='k', alpha=0.7)
                                
                        # Loop on the list of edges to get the x,y,z, coordinates of the connected nodes
                        # Those two points are the extrema of the line to be plotted
                        for i,j in enumerate(graph.edges()):
                                x = np.array((pos[j[0]][0], pos[j[1]][0]))
                                y = np.array((pos[j[0]][1], pos[j[1]][1]))
                                z = np.array((pos[j[0]][2], pos[j[1]][2]))
                                
                                self.ax.plot(x, y, z, c=graph_lines_color, alpha=0.5, linewidth=graph_size)
                                
                # Set the initial view
                self.ax.view_init(120, 0)
                
        def plot_pcd(self):
                PLOT_INTERVAL=100
                self.ax.scatter3D(self.np_point_cloud_xyz[::PLOT_INTERVAL,0], self.np_point_cloud_xyz[::PLOT_INTERVAL,1], self.np_point_cloud_xyz[::PLOT_INTERVAL,2], c=self.np_point_cloud_xyz[::PLOT_INTERVAL,2], cmap='bone');
                                  
if __name__ == "__main__":
        Vizualize3Dnetwork()        


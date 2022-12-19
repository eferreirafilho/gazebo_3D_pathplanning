#! /usr/bin/env python3

import numpy as np
import networkx as nx
import pickle
import rospy
from geometry_msgs.msg import PoseStamped, Twist, Point
import os, rospkg

class QueryPRM(object):
        def __init__(self):
                rospy.init_node('prm_path_planning', anonymous=True)
                self.rate = rospy.Rate(10) # 10hz
                rospack = rospkg.RosPack()
                self.path_to_package = os.path.join(rospack.get_path("hector_quadrotor_demo"))
                self.load_prm_file()
                self.load_point_cloud()
                self.start_node='start'
                self.current_position = Point()
                self.cmd_velocity_to_publish = Twist()
                self.current_target_wp = 1
                self.solution_graph=nx.Graph()
                self.cumulative_error_position = Point()
                goal_position_param = [rospy.get_param("/goal")]
                goal_position = Point(goal_position_param[0][0],goal_position_param[0][1],goal_position_param[0][2])
                if self.check_collision(goal_position):
                        rospy.logwarn('Goal position is in collision!')
                self.connect_node_to_prm('goal',goal_position)     
                
                rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, self._pose_gt_cb, queue_size=10)
                self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
                
                self.wait(5) # Wait for the UAV to publish position
                
                print(self.current_position)
                self.connect_node_to_prm('start',self.current_position)
                self.compute_shortest_path()
                self.wait(1)
                self.save_solution_graph()
                
                while not rospy.is_shutdown():
                        while not self.goal_reached():
                                self.send_uav_to_wp(self.current_target_wp)
                                # self.rate.sleep()
                        HOVER_TIME=200
                        for i in range(HOVER_TIME): # Hover in goal position for some time
                                self.rate.sleep()
                                self.send_uav_to_wp(self.current_target_wp)
                        rospy.logwarn_once('Goal Reached!')
                        return
                
        def wait(self, n_rate):
                for _ in range(n_rate):
                    self.rate.sleep()
                    
        def load_prm_file(self):
                self.G = pickle.load(open(self.path_to_package + '/map_files/prm_graph.pickle', 'rb'))
        def load_point_cloud(self): 
                self.np_point_cloud_xyz = np.load(self.path_to_package + '/map_files/point_cloud_prm.npy')
                
        def check_collision(self,point_checked):
                SAFETY_DISTANCE=4
                for xyz in self.np_point_cloud_xyz:
                        dist = np.linalg.norm(np.asarray([point_checked.x, point_checked.y, point_checked.z]) - np.asarray(xyz))
                        if dist<SAFETY_DISTANCE:
                                return True
                
        def compute_shortest_path(self):
                self.path_solution = nx.shortest_path(self.G, self.start_node, 'goal')
                rospy.logwarn('Path Solution: ' + str(self.path_solution))
                self.path_x=[]
                self.path_y=[]
                self.path_z=[]
                for i in self.path_solution:
                        self.path_x.append(nx.get_node_attributes(self.G, 'pos')[i][0])
                        self.path_y.append(nx.get_node_attributes(self.G, 'pos')[i][1])
                        self.path_z.append(nx.get_node_attributes(self.G, 'pos')[i][2])
                        self.solution_graph.add_node(i, pos=(nx.get_node_attributes(self.G, 'pos')[i][0], nx.get_node_attributes(self.G, 'pos')[i][1], nx.get_node_attributes(self.G, 'pos')[i][2]))
                for i in range(len(self.path_solution)-1):
                        self.solution_graph.add_edge(self.path_solution[i], self.path_solution[i+1])
                   
        def _pose_gt_cb(self, msg):
                self.current_position=msg.pose.position
                
        def connect_node_to_prm(self,node,position):
                dist_uav_to_prm=100000
                closest_node = []
                for i in self.G:
                        dist_node_i_to_prm = np.linalg.norm(np.asarray([position.x, position.y, position.z]) - np.asarray(nx.get_node_attributes(self.G, 'pos')[i]))
                        if dist_node_i_to_prm < dist_uav_to_prm:
                                dist_uav_to_prm = dist_node_i_to_prm
                                closest_node = i
                self.G.add_node(node, pos=(position.x, position.y, position.z))
                self.G.add_edge(node, closest_node)
                           
        def send_uav_to_wp(self, wp):
                KP=0.2 # Proporcional gain
                KI=0.0001 # Integral gain
                dist = np.linalg.norm(np.asarray([self.path_x[wp], self.path_y[wp], self.path_z[wp]]) - np.asarray([self.current_position.x, self.current_position.y, self.current_position.z]))
                self.cmd_velocity_to_publish.linear.x = KP*(self.path_x[wp]-self.current_position.x) + KI*self.cumulative_error_position.x
                self.cmd_velocity_to_publish.linear.y = KP*(self.path_y[wp]-self.current_position.y) + KI*self.cumulative_error_position.y
                self.cmd_velocity_to_publish.linear.z = KP*(self.path_z[wp]-self.current_position.z) + KI*self.cumulative_error_position.z
                if dist<5:
                        self.cumulative_error_position.x = self.cumulative_error_position.x + self.path_x[wp]-self.current_position.x
                        self.cumulative_error_position.y = self.cumulative_error_position.y + self.path_y[wp]-self.current_position.y
                        self.cumulative_error_position.z = self.cumulative_error_position.z + self.path_z[wp]-self.current_position.z
                # print('self.cumulative_error_position.x: ' + str(self.cumulative_error_position.x))
                # print('self.cumulative_error_position.y: ' + str(self.cumulative_error_position.y))
                # print('self.cumulative_error_position.z: ' + str(self.cumulative_error_position.z))
                
                self.pub.publish(self.cmd_velocity_to_publish)
                WP_ARRIVED_THREADSHOLD = 1
                rospy.loginfo_throttle(3, 'Distance to waypoint ' + str(wp) + ': ' + str(dist))
                rospy.loginfo_throttle(2, 'Target waypoint: ' + str(self.current_target_wp) + ' of ' + str(len(self.path_x)-1) + ' waypoints')
                rospy.loginfo_throttle(2, 'Current position: ' + str([self.path_x[wp], self.path_y[wp], self.path_z[wp]]))
                rospy.loginfo_throttle(2, 'Target waypoint position: ' + str([self.current_position.x, self.current_position.y, self.current_position.z]))
                if dist < WP_ARRIVED_THREADSHOLD:
                        self.current_target_wp=self.current_target_wp+1
                        if self.current_target_wp > len(self.path_x)-1:
                                self.current_target_wp = len(self.path_x)-1
                        rospy.logwarn_once('Waypoint ' + str(wp) + ' reached')
                        self.cumulative_error_position = Point() # reset the cumulative error
                        return
        
        def goal_reached(self):
                WP_ARRIVED_THREADSHOLD = 1
                dist = np.linalg.norm(np.asarray([self.current_position.x, self.current_position.y, self.current_position.z]) - np.asarray(nx.get_node_attributes(self.G, 'pos')['goal']))
                if dist < WP_ARRIVED_THREADSHOLD:
                        return True
                
        def save_solution_graph(self):
                pickle.dump(self.solution_graph, open(self.path_to_package + '/map_files/solution_graph.pickle', 'wb'))                
                
if __name__ == "__main__":
        QueryPRM()        
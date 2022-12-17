import os, rospkg
rospack = rospkg.RosPack()

path_to_file = os.path.join(rospack.get_path("hector_quadrotor_demo"))
print(path_to_file + '/map_files/prm_graph.pickle')


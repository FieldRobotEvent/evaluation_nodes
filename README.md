# fre_counter
read in the gazebo model states and count destroyed plants and robot distance traveled
how to use:
clone package into your ros workspace and compile:

cd ~/catkin_ws/src

git clone https://github.com/hohenheimdr/fre_counter.git

cd  ~/catkin_ws
catkin_make

start node with:
rosrun fre_counter fre_counter_node

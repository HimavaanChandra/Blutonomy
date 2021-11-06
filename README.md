# Blutonomy: Range Vector Localisation (RVL) for AUVs 

## Code Setup Instructions

1. Install and setup Ubuntu 20.04.
2. Install and setup ROS Noetic.
3. Install VRX simulator following GitHub repository instructions: https://github.com/osrf/vrx/wiki/host_install_tutorial
4. Clone Blutonomy repository by opening the terminal window in vrx_ws/src folder   
   ```git clone https://github.com/HimavaanChandra/Blutonomy.git ```

## Code Running Instructions

1. Open terminal window in vrx_ws folder
2. Source workspace ```source /opt/ros/noetic/setup.bash” and “source devel/setup.bash```
3. Launch VRX simulation ```roslaunch Blutonomy blutonomy.launch```
4. Run RVIZ to visualise localisation calculations ```roslaunch Blutonomy rviz.launch```
5. Run localisation code ```rosrun Blutonomy Blutonomy```
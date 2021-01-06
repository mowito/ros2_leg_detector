# ros2_leg_detector

This repository is developed for Mowito Obstacle Detection and Human Detection module in ROS2 Foxy Foxtrot

To run Leg detector in ROS2 ensure you have OpenCV 3.4.12 installed

Here are the steps to Run leg_detector in ros2

1.  clone the repository 
2. build the code with the following command

    `$> colcon build`
3. source the newly buit package

    `$> source <path to ros2_leg_detector>/install/setup.bash`

4. run the code with the launch file using the following command

    `$> ros2 launch leg_detector demo_stationary_simple_environment.launch.py`


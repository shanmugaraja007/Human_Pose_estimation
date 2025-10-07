YoloPose v8-ROS2 Pose Estimation with TurtleBot3 in Gazebo:

This guide walks you through setting up a ROS2 workspace, running TurtleBot3 in Gazebo, and applying YoloPose v8 for human pose estimation and gesture-based robot control.


Step 1:Install Required Dependencies

Ensure you have the correct numpy version:

    pip install --upgrade "numpy<2"
Required to ensure compatibility with YoloPose v8 and ROS2 bridges.



Step 2: Build the ROS2 Workspace

Navigate to your workspace and build:

    cd ros2_ws
    colcon build
    source install/setup.bash
This compiles your ROS2 packages and makes them available to ROS.



Step:3 Launch TurtleBot3 in Gazebo

Set TurtleBot3 model to burger and launch Gazebo:

     export TURTLEBOT3_MODEL=burger
     ros2 launch turtlebot3_gazebo empty_world.launch.py
     
we should now see the empty Gazebo world:
<img width="872" height="548" alt="Screenshot 2025-09-25 111308" src="https://github.com/user-attachments/assets/bbeee268-0199-495a-8c79-adbcd9e8982c" />



Step4: Run YoloPose v8 Pose Estimation Node
In a new terminal:

      ros2 run pose_estimation human_pose_node

This node subscribes to /camera/image_raw, processes the frame with YoloPose v8, and outputs keypoints and skeleton markers.




Step5: Visualize in RViz

Run RViz with your pose configuration:

      rviz2 -d ~/ros2_ws/src/pose_estimation/config/config.rviz
we should see a pose skeleton visualization:
      
<img width="1988" height="858" alt="Screenshot 2025-09-30 131343" src="https://github.com/user-attachments/assets/8ef81ccf-916e-4da7-9482-a4de16d25d5d" />



Implement Gesture-Based Robot Behavior:

With YoloPose v8, we detect body keypoints to control the robot:

Right hand raised - robot starts moving forward.

Both hands raised - robot stops.

These behaviors are implemented in the pose_gesture_node

close all the terminal.

repeat step2 in new terminal



Step6: Run the Pose Gesture Node

In another new terminal:

      ros2 run image_publisher image_publisher_node --ros-args -p filename:=/home/user/ros2_ws/src/pose_estimation/media/WIN_20251006_13_28_05_Pro.mp4

This publishes the custom video stream to /image_raw for YoloPose processing.



Step7: Run the pose gesture node in a new terminal

      ros2 run pose_estimation pose_gesture_node
Now the robot should react to gestures detected by YoloPose v8:

<img width="2353" height="1283" alt="Screenshot 2025-10-07 152940" src="https://github.com/user-attachments/assets/a46b9856-c2f3-4480-9c43-ceed5ec73813" />



     



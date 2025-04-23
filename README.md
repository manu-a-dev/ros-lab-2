# WWU CSCI 497F / 597F Lab 2

## Goals

1. Become familiar with the basic functionality of ROS2 and Gazebo, an open source 3D physics simulator.

2. Become familiar with sensor modelling and feedback system design.

## Deliverables

You are expected to upload a typed lab report. Please fork this repository and include a link to your forked version in your lab report. 

For each numbered item in the lab [assignment](#Assignment), the report should contain: 

- Methods: a summary of what you did in this lab. Should be written in your own words, can be short but needs to include enough details so that I understand what decisions you made and why. 
- Results: a summary of what you learned, in your own words.

At the end of your lab report, include: 

- Next Steps: a list of questions that you have after completing this lab, and/or what you would do next to fix your system if it’s not working. 

## Usage

1. Build the project

    ```bash
    colcon build --cmake-args -DBUILD_TESTING=ON
    ```

1. Source the workspace

    ```bash
    source ./install/setup.sh
    ```

1. Launch the simulation

    ```bash
    ros2 launch ros_gz_bringup diff_drive.launch.py
    ```

For a more detailed guide on using this package see [the Gazebo documentation](https://gazebosim.org/docs/latest/ros_gz_project_template_guide).

## Assignment

1. Launch the simulation. Two windows should launch:  the Gazebo simulation of a differential drive robot
   in a maze and RViz, a ROS program that visualizes topics such as pose and odometry estimates and sensor readings.
   - **Note:** Each time you launch the simulation, to you need to press the arrow-shaped "Play" button in the lower left hand corner of the Gazebo simulator.
   - In a new terminal with ROS2 configured, run the command

   ```bash
   ros2 topic pub /diff_drive/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.1}}"
   ```

   - In another terminal with ROS2 configured, run the command

   ```bash
   ros2 topic echo /diff_drive/scan
   ```

   What do you notice about the output of the second command as the robot moves
   around the simulated space? This topic publishes the current distance measured
   by a single-laser range detector.

2. Using publisher and subscriber nodes, implement a wall-following strategy
that uses the laser scan data to ensure the robot can follow the wall all the
way around the environment. Submit a screenshot of the RViz window with the
odometry path of your robot in your writeup. The figure below shows an example of a high-level
strategy for wall-following. The numbers are arbitrary and you may structure your controller differently.
For example, you may design a continuous controller instead of discrete.

Notably, our robot lacks a side-facing sensor in the original version. You may choose to turn the robot (while in motion, or in place) 
to simulate such a measurement, or you may choose to edit the sensor model to allow for a wider "sweep" of the 
laser rangefinder. You can find our sensor model in [ros_gz_gazebo/worlds/diff_drive.sdf](ros_gz_gazebo/worlds/diff_drive.sdf) (search for "lidar").
The documentation for the different description fields for the lidar sensor can be found [here](http://sdformat.org/spec?ver=1.12&elem=sensor#sensor_lidar).

![](figs/automata.png)

For an A grade, your robot only needs to be able to circumnavigate the larger room; you will be graded based upon the level of understanding in your writeup.
If your robot misses a turn or gets stuck, try to explain why you think it is happening.

3. Use CTRL+C to close the simulator and RViz. Open the file `ros_gz_description/models/diff_drive/model.sdf` in a text
editor. Search for the word "sensor" to find the definition of the laser
scanner.

   - **a.** In the `<range>` block, change the max range to be 1.0 instead of 100.0 .
      Save the file, rebuild the entire lab2 ROS package, and restart your simulation.
      What is the effect on your wall-following strategy? How can you change your
      strategy to compensate for the reduced sensing range? Reset your system to have a
      maximum range of 100.0 when you are done.

   - **b.** In the `<noise>` block, change the standard deviation of the sensor noise
      (`stddev`) to be 1.0 instead of 0.1. Save, rebuild, and restart your simulation.
      What is the effect on your wall-following strategy? How can you compensate for
      the large noise level? Reset your system to a noise level of 0.1 when you are done.
   
   - **c.** In the `<noise>` block, change the `mean` to be nonzero. What is the effect
      on your wall-following strategy? What happens as the value of the mean gets
      larger? What happens as the mean gets smaller (the sensor experiences a stronger
      negative bias)?


4. (Graduate students only) Implement a low-pass filter on your rangefinder data in your
wall-following code. Analyze the performance of your wall-following strategy
with respect to both:
   - **a.** Varying the standard deviation of the rangefinder noise; and
   - **b.** Varying the baseline speed of the robot.
Ideally, include a plot each for a) and b), graphing a metric of the performance of your system
over time. Screenshots of RViz are OK too, as well as precise technical descriptions
of the resulting robot motion.

## Recommended Reading

* Chapter 2, through section 2.4: Sensing for a Point Robot of *Computational Principles of Mobile Robotics (CPMR)*.
* Chapter 3.1 of *CPMR*, specifically the parts on *differential drive* robots.
* Chapter sections 11.1, 11.2, and 11.6 (for graduate students) of *Planning Algorithms*.

## Included packages

* `ros_gz_description` - holds the sdf description of the simulated system and any other assets.

* `ros_gz_gazebo` - holds gazebo specific code and configurations.

* `ros_gz_app` - holds ros2 specific code and configurations.

* `ros_gz_bringup` - holds launch files and high level utilities.

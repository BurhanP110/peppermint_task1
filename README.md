# Peppermint Robotics - Task 1: Color Based Navigation

## Overview
This repository contains a ROS 2 package designed to autonomously navigate a TurtleBot3 towards a specific colored object (a green sphere) in a Gazebo simulated environment. 

The system uses 2 nodes that is based on the working principles:  Computer Vision (OpenCV) and Motion Control (P-Controller), ensuring modular, responsive, and robust robot behavior.

   
https://github.com/user-attachments/assets/a5295881-fdd3-4a65-81c6-d7e312d6aff0


<br>
Video: Turtlebot3 following a green sphere, Final Output


## System Architecture & Approach

###  Computer Vision Prototyping

It is very easy to assume that the color range for a green sphere would be making (RGB - (0,1,0). That is what i thought too. But I failed to account for the difference the lighting makes when it comes to interpreteation of the color. So a color like green goes from a solid color to a number of gradients in one shape as well. 

So, I did what I was taught in CV lab last semester. I launched turtlebot, opened rqt , subscribed to camera/img_raw saved an image where the robot observers the whole sphere. Though my methods are quite crude, they did the job of giving the color space and the rough estimate of the range of the color green. I have attached the .ipynb file to view for the same and have attached the results below. 


<img width="673" height="399" alt="image" src="https://github.com/user-attachments/assets/dcbacfcd-547e-48dd-959a-86d8507c6f3c" />
<br>
Fig 1: The Image plotted in matplotlib
<br>
<br>
<br>
<img width="759" height="731" alt="image" src="https://github.com/user-attachments/assets/4a8a513b-9322-4b4d-bd8f-c14e8183b75b" />
<br>
Fig 2: The RGB 3D Plot 
<br>
Axis and Labes(x: Red, y: Green, z: Blue)

<br>
<br>
<br>
<img width="697" height="668" alt="image" src="https://github.com/user-attachments/assets/53358b5a-3609-447c-8fcc-831e9b425da4" />
<br>
Fig 3: The HSV 3D Plot 
<br>
Axis and Labes(x: Hue, y: Saturation, z: Value)
<br><br><br>

Once I got confidence, that this is how the robot is going to break down the task, I got into writing the nodes. 

When I was writing the vision_node, I got stuck on how is ROS transporting such large image data continously and quicly. When I read and researched about it, I got to know, that ROS isn't directly sending the image, but converting into data bytes optimized for transporting such heavy data, and when I want to work on it I will use the Opencv objet (cv2) and perform all the operations like edge detection, contours, blurring, etc. and in future I have to transport the processed image again, i will have to use the ros to cv bridge again (msgs-> images).


### 1. Vision Node (`vision_node.py`)
This node is responsible for the robot's "eyes". It subscribes to the `/camera/image_raw` topic and processes the BGR feed to find the green sphere.
* **Color Space Conversion:** Converts BGR to HSV for robust color detection under Gazebo's lighting conditions.
* **Thresholding & Morphology:** Applies a mask to isolate green pixels and uses morphological operations (Opening) to remove noise.
* **Centroid Calculation:** Finds the largest green contour and uses `cv2.moments` to mathematically calculate its center of mass `(cx, cy)`.
* **Error Calculation:** Calculates the horizontal pixel error (`image_center_x - cx`) and publishes it to a custom `/error` topic as a `std_msgs/Float64`.
* **Out of Frame Recovery:** If no green contours are detected, it publishes a sentinel value (`9999.0`) to trigger a search behavior.



### 2. Controller Node (`controller_node.py`)
This node acts as the "brain". It subscribes to the `/error` topic from the camera and the `/scan` topic from the LiDAR.
* **Visual Servoing (P-Controller):** Multiplies the horizontal pixel error by a tuned proportional gain (`Kp = 0.002`) to calculate smooth angular velocity (`cmd_vel.twist.angular.z`).
* **Forward Kinematics:** If the object is centered within a specific pixel tolerance, it commands forward linear velocity.
* **LiDAR Stopping:** Parses the 20-degree front cone of the LaserScan. If an object is detected within `35cm` (0.35m), it halts the robot completely, successfully completing the task.
* **Search State:** If it receives the `9999.0` sentinel value, it commands a slow, continuous rotation to search for the lost object.

---

### How would you account for the object going out of frame? How would you recover?

In my implementation, I accounted for the object going out of frame by building a dedicated 'search state' into my control logic.

If the sphere leaves the camera's Field of View (FOV), the Vision Node's contour detection fails. Instead of crashing or doing nothing, I programmed the Vision Node to immediately publish a specific sentinel error value (9999.0) to the /error topic.

My Controller Node constantly monitors this topic. When it receives this 9999.0 value, it knows the robot is 'blind' to the target. To recover, the controller automatically stops all forward linear movement and commands a steady angular velocity (cmd_vel.twist.angular.z = 0.3). This causes the robot to slowly rotate in place, actively scanning its environment until the green sphere comes back into the camera's frame, at which point the standard P-controller seamlessly takes over again.

### How would you figure out that the robot is close to the obstacle and stop? Is there any other data you can use?

To figure out when the robot was close to the obstacle, I decided to use sensor fusion rather than relying solely on the camera.

While I could have used the camera data to estimate distance—for example, by calculating the area of the green contour and stopping when it filled a certain percentage of the screen—that method is highly susceptible to lighting changes and varying object sizes.

Instead, I used the TurtleBot's 2D LiDAR. I programmed the Controller Node to subscribe to the /scan topic and parse the specific array elements corresponding to the front 20 degrees of the robot's view. By isolating the minimum valid distance in this front cone, the robot gains a highly precise measurement of how far away the sphere is. Once that LiDAR distance reading drops to 35 centimeters (0.35 meters), the controller overrides all other commands, sets the velocities to exactly 0.0, and safely halts the robot.

<br><br>

## Setup and Installation

### Prerequisites
* [Ubuntu 24.04](https://ubuntu.com/download/desktop)
* [ROS 2 (Jazzy)](https://docs.ros.org/en/jazzy/Installation.html)
* [Gazebo (Harmonic/gz-sim)](https://gazebosim.org/docs/latest/getstarted/)
* [TurtleBot3 ROS2 Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
* OpenCV and cv_bridge

### 1. Workspace Setup
Clone this package into your ROS 2 workspace `src` directory or just copy paste all the commands in your terminal: 
```bash
# Source ROS Jazzy environment
source /opt/ros/jazzy/setup.bash

# Create workspace
mkdir -p ~/peppermint_ws/src
cd ~/peppermint_ws/src

# Clone your repository
git clone https://github.com/BurhanP110/peppermint_task1.git

# Move back to workspace root and install dependencies
cd ..
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build and Launch
colcon build --packages-select peppermint_task1
source install/setup.bash
ros2 launch peppermint_task1 task.launch.py

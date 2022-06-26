# Aruco Navigation

This project aims to make a aruco navigation robot which follow a particular Aruco Marker also keeping a minimum safe distance from it.

The software stacks implemented in this project are ROS and Python. We also made use of OpenCV for the Aruco detection part.

An ArUco marker is a synthetic square marker composed by a wide black border and an inner binary matrix which determines its identifier (id). The black border facilitates its fast detection in the image and the binary codification allows its identification and the application of error detection and correction techniques. The marker size determines the size of the internal matrix. 


<br>

# Installation

## Pre-Requisites :
- ROS noetic : Refer to the [official documentation](http://wiki.ros.org/noetic/Installation/Ubuntu) for installation of ROS noetic.
               
- Catkin workspace : A catkin workspace is a folder where you modify, build, and. install catkin packages. Take a look ak the [official documentation](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) for instructions regarding creation of a catkin workspace


<br>

## Installation of Virtualenvwrapper, OpenCV, and CV_bridge

Your can refer to [A.T.O.M's wiki](https://atom-robotics-lab.github.io/wiki/setup/virtualenv.html) for installation of the above mentioned packages and libraries.

<br>

## Clone the Aruco Navigation package
Now go ahead and clone this repository inside the "src" folder of the catkin workspace you just created by executing the command given below in your terminal.
```bash
git clone git@github.com:atom-robotics-lab/aruco_navigation.git
```

<br>

## Clone the MR-Robot package
this package provide us the bot which we are gonig to use.
Go inside the "src" folder of the catkin workspace and executing the command given below in your terminal.
```bash
git clone git@github.com:atom-robotics-lab/MR-Robot.git
```

<br>

Now out robot does not have camera in this package so we have to chang branch from main to with_camera.   
Now then go inside MR-Robot package you just created by executing the above command then executing the command given below in your terminal.
```bash
git checkout with_camera 
```

<br>

## Make the package
We'll need to "make" everything in our catkin workspace so that the ROS environment knows about our new package.  (This will also compile any necessary code in the package). Execute the given commands in your terminal.

```bash
cd ~/catkin_ws
catkin_make
```


<br>

## Edit your bashrc file

You need to add these lines in your bashrc file to path aruco marker in gazebo.

To open your bashrc file
```bash
nano ~/.bashrc
```


```bash
source /usr/share/gazebo/setup.bash
export GAZEBO_MODEL_PATH="${CATKIN_ENV_HOOK_WORKSPACE}/../src/aruco_navigation/models/:${GAZEBO_MODEL_PATH}"
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/path_to_your_catkin_workspace/src/aruco_navigation/models
```

```bash
source ~/.bashrc
```

<br><br>
VAMOS!! The installation is done and now its time to play around with the robot :)



## Launch

```bash
roslaunch aruco_navigation aruco_navigation.launch
```
The above command when executed in the terminal will launch the gazebo simulation and will also start ROS Master.






## Run the node

```bash
rosrun aruco_navigation aruco_controlloop.py
```

The given command will run the controller script which controls the robot's movements.

<img src = "https://github.com/atom-robotics-lab/aruco_navigation/blob/main/assets/Untitled%20design%20(1).gif" >



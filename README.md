# Rufous
This repo is dedicated to the development of an autonomous quadcopter.

The goal is to make a sneaky companion that can be instructed to follow people or track objects. This would be great for filming outdoor activities of all sorts, the drone must be able to move with speed and accuracy as to not loose its target. In addition the drone must be able to perform real time computer vision and make situational inferences about the task at hand. This repository is organized to help develop and test the software systems of the device as well as resources for development.

## See it yourself
# Install
The repo is designed to be built inside of the dyse-robotics repo which includes general tools for projects, but can be built on its own. (This project requires a prior ROS installation)

First clone the repo: 

    git clone https://github.com/mithellscott/Rofous

Then cd into the project and run the setup script

    cd Rofous
    ./setup.bash 
    	Usage and Hints:

      [Option]	 Description

	-e	 specifies the path to the python3 environment for this project
	-n	 decides the name of the python env
	-p	 declares the name of the project (required)
	-g	 specifies the extension of a git repo to attach to the project's root (make sure to update .gitignore before)
	-o	 specifies file path to load the above parameters from
	-b	 will build the project with catkin build
	-h	 prints this help message (program will exit after)

    
Now the project has an alias to load the tools in .bashrc run those like 

    source ~/.bashrc
    
Now your python enviroment is configured and your catkin_ws should be initialized you can check that this was successful by 

    echo $PYTHON_PATH
    catkin config
    
The simulation is runable with roslaunch
    
    roslaunch rofous_gazebo main.launch
    
    
You can run the python version with 

    cd Python3_Notebooks
    jupyter notebook

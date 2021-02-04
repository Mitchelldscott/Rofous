# Rofous
This repo is dedicated to the development of an autonomous quadcopter.

The goal is to make a sneaky companion that can be instructed to follow people or track objects. This would be great for filming outdoor activities of all sorts, the drone must be able to move with speed and accuracy as to not loose its target. In addition the drone must be able to perform real time computer vision and make situational inferences about the task at hand. This repository is organized to help develop and test the software systems of the device as well as resources for development.

## See it yourself
# Install
The repo is designed to be built inside of the dyse-robotics repo which includes general tools for projects, but can be built on its own. (This project requires a prior ROS installation)

First clone the repo: 

    git clone https://github.com/mithellscott/Rofous

Then cd into the project and run the setup script (pass the script the desired path to the python environment it will create) 

    cd Rofous
    ./setup.bash -e <ENVIRONMENT PATH>
    
Now the project has an alias to load the tools in .bashrc run those like 

    source ~/.bashrc
    load_Rofous
    
Now your python enviroment is configured and your catkin_ws should be initialized you can check that this was successful by 

    echo $PYTHON_PATH
    catkin build
    
The simulation is runable with roslaunch
    
    roslaunch rofous_gazebo main.launch
    
    
You can run the python version with 

    cd Python3_Notebooks
    jupyter notebook

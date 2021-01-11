#!/usr/bin/env bash
# This is the setup template for new_projects
# This file adds the tools package to PYTHONPATH
# it also creates an alias for adding the package

# initialize some variables for setup
DIR=$PWD
P_MODE=0
P_REAL=0
ME=Installer
PROJECT_NAME=Rofous
ENV_NAME=${PROJECT_NAME}_env

echo [${ME}] installing ${PROJECT_NAME} from ${DIR}

helpFunction()
{
   echo -e "\n \t-e Decides the location of the default python3 environment for this project"
   echo -e "\t-n Decides the name of the python env"
   echo -e "\t-p will purge the project setup\n"
   exit 1 # Exit script after printing help
}

checkInputs()
{
	if [ -z ${ENV_PATH} ]; then
		echo -e [${ME}] Missing default python3 environment path
		helpFunction
	fi
}

purgeFunction()
{
	checkInputs
	echo
	echo [${ME}] purging the setup ...
	echo
	if [[ ${PTHONPATH} == */${ENV_PATH}/* ]]; then
		deactivate
	fi
	export PYTHONPATH=
	echo -e [${ME}] removing:" \n\t $ENV_PATH/$ENV_NAME \n\t build \n\t devel \n\t src/CMakeLists.txt"
	sudo rm -rf ${ENV_PATH}/${ENV_NAME}
	catkin clean
	if [[ P_MODE == 3.141592 ]]; then
		read -p "Are you sure this is sketchy? (y/n)" P_REAL
	fi
	if [[ P_REAL == y ]]; then
		echo -e '\n\n[${ME}] Fairwell Sir\n'
		git checkout master
		sudo rm -rf src/* Python3_Notebooks/* ${PROJECT_NAME}_tools/*
		git add .
		git commit -m "See Ya!"
		git push origin master
	fi
	echo
	echo [${ME}] Done purging!
	echo
	exit 1
}

# make sure that the script is executing from Rofous root
if [[ ${DIR} == */${PROJECT_NAME} ]]; then
	echo [${ME}] Initializing repository ;
else
	echo [${ME}] Please run this script from the top directory of this repository ; 
	helpFunction
fi

# get user inputs, null will quit
while getopts e:n:p: opt
do
	case ${opt} in
    	e) ENV_PATH=${OPTARG} ;;
		n) ENV_NAME=${OPTARG} ;;
		p) P_MODE=${OPTARG} purgeFunction ;;
    	?) helpFunction ;; # Print helpFunction in case parameter is non-existent
	esac
done

# force user to provide python environment path
checkInputs

if [[ ${DIR} == */Dyse-Robotics/Projects/* ]] || [[ ${DIR} == */dyse-robotics/Projects/* ]]; then
	echo [${ME}] pulling Dyse-Tools ;
	cp ../../dyse_tools/*.py ${PROJECT_NAME}_tools
else
	echo [${ME}] Isolated project, no tools available -e\n\t{$DIR};
fi

# install python3 dependencies
sudo apt-get update
python3 -m venv ${ENV_PATH}/${ENV_NAME}
source ${ENV_PATH}/${ENV_NAME}/bin/activate
echo [${ME}] Creating your default environment
pip install --upgrade pip
pip install --ignore-installed -r ${DIR}/python3_requirements.txt
deactivate
export PYTHONPATH=/opt/ros/melodic/lib/python2.7/dist-packages

# initialize the workspace
echo [${ME}] initializing catkin workspace with ${ENV_PATH}/${ENV_NAME}/bin/python3 as executable path
catkin config --init
catkin config -DPYTHON_EXECUTABLE=${ENV_PATH}/${ENV_NAME}/bin/python3
catkin build

export PYTHONPATH=/opt/ros/melodic/lib/python2.7/dist-packages:${DIR}/${PROJECT_NAME}_tools:${DIR}/src
echo "alias load_${PROJECT_NAME}='export PYTHONPATH=$PYTHONPATH && source ${DIR}/devel/setup.bash && source ${ENV_PATH}/${ENV_NAME}/bin/activate'" >> ~/.bashrc

echo
echo [${ME}] Successful Install
echo
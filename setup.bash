#!/usr/bin/env bash
# This is the setup file for Rofous
# This file adds the tools package to PYTHONPATH
# it also creates an alias for adding the package

# initialize some variables for setup
DIR=$PWD
ME=Installer
ENV_NAME=drone-env

echo [${ME}] installing from ${DIR}

helpFunction()
{
   echo -e "\n \t-e Decides the location of the default python3 environment for this project"
   echo -e "\t-name Decides the name of the python env the default is drone-env"
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
	PYTHONPATH=
	echo -e [${ME}] removing:" \n\t $ENV_PATH/$ENV_NAME \n\t build \n\t devel \n\t src/CMakeLists.txt"
	sudo rm -rf ${ENV_PATH}/${ENV_NAME} build devel src/CMakeLists.txt
	echo
	echo [${ME}] Done purging!
	echo
	exit 1
}

# make sure that the script is executing from Rofous root
if [[ ${DIR} == */Rofous ]] || [[ ${DIR} == */rofous ]]; then
	echo [${ME}] Initializing repository ;
else
	echo [${ME}] Please run this script from the top directory of this repository ; 
	helpFunction
fi

# get user inputs, null will quit
while getopts e:name:p: opt
do
	case ${opt} in
    	e) ENV_PATH=${OPTARG} ;;
		name) ENV_NAME=${OPTARG} ;;
		p) purgeFunction ;;
    	?) helpFunction ;; # Print helpFunction in case parameter is non-existent
	esac
done

# force user to provide python environment path
checkInputs

# install python3 dependencies
sudo apt-get update
python3 -m venv ${ENV_PATH}/${ENV_NAME}
source ${ENV_PATH}/${ENV_NAME}/bin/activate
echo [${ME}] Creating your default environment
pip install --upgrade pip
pip install --ignore-installed -r ${DIR}/python3_requirements.txt
deactivate
PYTHONPATH=/opt/ros/melodic/lib/python2.7/dist-packages

# initialize the workspace
echo [${ME}] initializing catkin workspace with ${ENV_PATH}/${ENV_NAME}/bin/python3 as executable path
catkin_make -DPYTHON_EXECUTABLE=${ENV_PATH}/${ENV_NAME}/bin/python3

echo "alias load_rofous='PYTHONPATH=$PYTHONPATH:${DIR}/rofous_tools && source ${DIR}/devel/setup.bash && source ${ENV_PATH}/${ENV_NAME}/bin/activate'" >> ~/.bashrc

echo
echo [${ME}] Successful Install
echo

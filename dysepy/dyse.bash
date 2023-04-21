#! /bin/bash

export UBUNTU_VERSION=$(cut -f2 <<< $(lsb_release -r))

#		Setup robot params
export DOCKER=False
export PROJECT_ROOT=${PWD}
export ROBOT_PACKAGE="${PROJECT_ROOT}/dysepy"
export HOSTNAME=$HOSTNAME 
export SUDO='sudo'

if [[ -f /.dockerenv ]]; then
	SUDO=''
	DOCKER=True
	PROJECT_ROOT=/home/cu-robotics/rufous
fi

if [[ "${UBUNTU_VERSION}" == "20.04" ]]; then
	export ROS_DISTRO=noetic				# ROS for Ubuntu18
elif [[ "${UBUNTU_VERSION}" == "18.04" ]]; then
	export ROS_DISTRO=melodic
fi


if [[ "${DOCKER}" == "False" ]]; then
	alias spinup="cd ${PROJECT_ROOT}/containers && \
		docker compose run "
	
else
	export LC_ALL=C.UTF-8
    export LANG=C.UTF-8
fi

if [[ "${PROJECT_ROOT}" != */rufous ]]; then
	echo -e "running from ${PROJECT_ROOT}, is this your project root?"
	return
fi

PYTHONPATH=

# If ROS is installed source the setup file

if [[ -f /opt/ros/${ROS_DISTRO}/setup.bash ]]; then
	source /opt/ros/${ROS_DISTRO}/setup.bash
fi

#		setup Cargo tools

if [[ "${PATH}" != *"/.cargo"*  && -f ${HOME}/.cargo/env ]]; then
	source ${HOME}/.cargo/env
fi

#		Setup python tools

if [[ "${PATH}" != *"${ROBOT_PACKAGE}/scripts"* ]]; then
	export PATH="${ROBOT_PACKAGE}/scripts:${PATH}"
fi 

# Only export if if not already in path

if [[ "${PYTHONPATH}" != *"${ROBOT_PACKAGE}/lib:"* ]]; then	
	export PYTHONPATH="${ROBOT_PACKAGE}/lib:${PYTHONPATH}" 
fi

# set ROS package path to buff-code so it can see dysepy

if [[ "${ROS_PACKAGE_PATH}" != *"rufous"* ]]; then
	export ROS_PACKAGE_PATH="${PROJECT_ROOT}:${ROS_PACKAGE_PATH}"
fi

alias bc="cd ${PROJECT_ROOT}"
alias br="cd ${PROJECT_ROOT}/src/buff_rust"
alias fw="cd ${PROJECT_ROOT}/src/firmware"
alias bn="cd ${PROJECT_ROOT}/src/rknn_buffnet"

if [[ "${HOSTNAME}" == "edge"* ]]; then
	export ROS_IP=$(/sbin/ip -o -4 addr list wlan0 | awk '{print $4}' | cut -d/ -f1)
	export ROS_MASTER_URI=http://${ROS_IP}:11311
else
	# if not on jetson set the user IP
	# should figure out how to set it if it is on the jetson
	# export USER_IP=$(/sbin/ip -o -4 addr list wlp3s0 | awk '{print $4}' | cut -d/ -f1) # Needs testing

	alias buildr="dysepy -b rust-debug"
	alias buildf="dysepy -b fw"
	alias builda="dysepy -b all"
	alias buff-test="br && cargo test"
	alias sshbot="ssh -X cu-robotics@edgek.local"
	set-ros-master () {
		export ROS_MASTER_URI=http://$1:11311
	}
fi


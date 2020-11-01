#!/usr/bin/env bash
# This is the setup file for Rofous
# This file adds the tools package to PYTHONPATH
# it also creates an alias for adding the package

helpFunction()
{
   echo -e "\n\t-e Decides the location of the default python3 environment for this project\n"
   exit 1 # Exit script after printing help
}

DIR=$PWD
ME="Installer"
echo "[$ME] installing from $DIR"

if [[ "$DIR" == *"/Rofous" ]] || [[ "$DIR" == *"/rofous" ]]; then
	echo "[${ME}] Initializing Repository" ;
else
	echo "[${ME}] Please run this script from the top directory of this repo" ; 
	helpFunction
fi

while getopts "e:" opt
do
   case "$opt" in
      e ) ENV_PATH="$OPTARG" ;;
      ? ) helpFunction ;; # Print helpFunction in case parameter is non-existent
   esac
done


if [ -z "$ENV_PATH" ]; then
	echo -e "[${ME}] Missing default python3 environment path" 
	helpFunction 
fi

sudo apt-get update
python3 -m venv "${ENV_PATH}/drone-env"
source "${ENV_PATH}/drone-env/bin/activate"
echo "[${ME}] Creating your default environment"
pip install --upgrade pip
pip install --ignore-installed -r "$DIR/python3_requirements.txt"

if [[ *"$DIR/rofous_tools"* == "$PYTHONPATH" ]]; then
	echo "[${ME}] Path already exported!"
else
	export PYTHONPATH=$PYTHONPATH:"${DIR}/";
	echo "[${ME}] Added ${DIR}/rofous_tools to PYTHONPATH";
fi

echo "alias load_rofous='source ${ENV_PATH}/drone-env/bin/activate && export PYTHONPATH=$PYTHONPATH:${DIR}/rofous_tools'" >> ~/.bashrc
echo " "
echo "[ME] Successful Install "
echo " "

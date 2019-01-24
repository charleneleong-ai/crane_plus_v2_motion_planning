#!/bin/bash -i


printf "\n==========  Updating Ubuntu  ==========\n"
sudo apt-get update -y
sudo apt-get upgrade -y

printf "\n==========  Installing Pip  ==========\n"
sudo apt-get install python-pip
sudo apt-get install python3-pip

printf "\n==========  Installing Pip Reqs  ==========\n"
source ~/.bashrc
pip install -r requirements.txt

printf "\n==========  Installing SMAC  ==========\n"
sudo apt-get install build-essential swig
curl https://raw.githubusercontent.com/automl/smac3/master/requirements.txt | xargs -n 1 -L 1 pip3 install
pip3 install smac

find ./ -name "*.py" -exec chmod +x {} \;
#!/bin/bash

current_dir=$(pwd)
echo "source ${current_dir}/metr4202_ws/install/setup.bash" >> ~/.bashrc
echo "export PATH=$current_dir/bin:$PATH" >> ~/.bashrc
source ~/.bashrc

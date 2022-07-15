## How to pick a mobile robot simulator: A quantitative comparison of CoppeliaSim, Gazebo, MORSE and Webots with a focus on accuracy of motion

> Authors: Andrew Farley, Jie Wang, Joshua Marshall  
> Ingenuity Labs Research Institute  
> Queen's University at Kingston, Canada

This repository includes the assets for each of four simulators used in a Husky A200 mobile robot simulation comparision. In each folder is the models used in each environment, the environment file itself, the model of the Husky A200 mobile robot used for each simulator, as well as other odds and ends that are needed.

### CoppeliaSim
To use the assets for CoppeliaSim, you must first install and open CoppeliaSim. Once this is done you should be able to open the environment file with File->Open and then choosing the .ttt file included in the "my_scenes" folder in the included CoppeliaSim folder. This should load the environment with the Husky and lab pieces already setup. The SDF files for each lab piece model is included in the "models" folder and the model of the Husky is included in the "urdf" folder.

### Gazebo
To use the assets for Gazebo, you must first install Gazebo. Then, you can run the launch file named "husky_lab.launch" included in the "husky_environment" folder. Similar to CoppeliaSim, the lab piece models are included in the "models" folder and the Husky model is included in the "urdf" folder.

### MORSE
To use the assets for MORSE, you must first install MORSE. Then, you can run the MORSE lab world by navigating to the included MORSE folder and running `morse run lab_world`. The models for the lab pieces are apart of the lab_world.blend file. To copy and edit these you will neeed to open this file in blender. The model for the Husky is included in the "data/lab_world/robots" folder.

### Webots
To use the assets for Webots, you must first install and open Webots. Then, you should be able to load the environment with File->Open and selecting the lab_world.wbt file in the "worlds" folder in the included Webots folder. This should open the lab world in Webots with the scene already setup. The models used in Webots are included in this environment file. The model for the Husky is in the folder "protos" with the name Husky.proto.

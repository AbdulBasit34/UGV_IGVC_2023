# UGV-DTU

UGV-DTU ROS workspace for IGVC 2023.

## Instructions to use

Prerequisites:
- Install ROS Noetic on Ubuntu 20.04
- Install ZED SDK 3.x
- (optional) Install CUDA 11.x
- (optional) Build openCV 4.7 for python with CUDA

Then execute the following code in the home folder:

```
git clone https://github.com/AbdulBasit34/UGV_IGVC_2023.git
cd UGV_IGVC_2023
catkin_make
```

Add the following in your `.bashrc` at the end:

```
source /home/<your username>/UGV_IGVC_2023/devel/setup.bash
```

Now either restart your terminal or run `source ~/.bashrc`.

### Launch sequence

Ping one of the team members for the latest `launch.sh`. Place it in the home folder. Open a terminal in the home folder and run

```
./launch.sh
```

Make modifications to the launch file as needed.

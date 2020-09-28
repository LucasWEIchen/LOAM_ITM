# LOAM_ITM
LOAM with posture recovery using GPS trajectory

## A modified implementation of A-LOAM

LOAM_ITM combines LOAM with ITM method. It enhances LOAM robutness through trajectory matching. 
The modification is based on [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)

**Modifier:** [Weichen WEI](weichen.wei@monash.edu)

<img src="https://github.com/LucasWEIchen/LOAM_ITM/blob/master/picture/itm_1.gif" width = 100% height = 100%/>
<img src="https://github.com/LucasWEIchen/LOAM_ITM/blob/master/picture/itm_2.gif" width = 100% height = 100%/>

## Install
LOAM_ITM requires ROS, PCL and Ceres installed. The instaliation procedure is same as listed on [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)

## Test
The current launch file works with topics in KITTI dataset. It uses GPS in KITTI as the reference frame. However, the package can also work with any bag file which output a Lidar topic and a reference odometry topic.
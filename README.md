# Introduction
The repo is pure localization for fastlio with given initial pose from rviz,
## DEMO
https://github.com/iDonghq/FAST_LIO_LOCALIZATION_PLUS/assets/23080413/a58d2e9c-12cb-4efa-84b5-a8dac1df0324

# Convergence and Map Bending
## CONVERGENCE
Thanks to the robust of [FASTLIO](https://github.com/hku-mars/FAST_LIO), in the following video the initial pose that deviates greatly from the true one is given, **FAST_LIO_LOCALIZATION_PLUS** can also quickly converge to near the truth value.
![relocalization](https://github.com/iDonghq/FAST_LIO_LOCALIZATION_PLUS/assets/23080413/c7b60bbc-1412-45a6-a958-20abc1d82558)
## DEMO 
https://github.com/iDonghq/FAST_LIO_LOCALIZATION_PLUS/assets/23080413/d18ee041-649b-4b39-9082-54fa4fa4a5e5

# MAP BENDING
Not enough observations in the z direction from lidar leads to the map bending to z direction. In the following fig, the number of normal vector, which is the normal of plane fitted by neraby points, in the same as gravity direction is counted
![output](https://github.com/iDonghq/FAST_LIO_LOCALIZATION_PLUS/assets/23080413/07f1a65a-04cf-4c6f-8d0a-e5c992237376)
Correspondingly to the generted map is:
![output (1)](https://github.com/iDonghq/FAST_LIO_LOCALIZATION_PLUS/assets/23080413/ed9d9090-c26f-4bc9-8066-f591b10a7a80)
In  **FAST_LIO_LOCALIZATION_PLUS** mapping, the number of plane's normal fitted by neraby points is monitored, the it is less than the threshhold, I set the translation **z=0** from pose, the result is:
![new_sides](https://github.com/iDonghq/FAST_LIO_LOCALIZATION_PLUS/assets/23080413/8eba522d-3adc-40cb-b145-83578d6e9fcd)


# BUILD and DEPENDENCE 
The BUILD and DEPENDENCE is completely the same as [fastlio-Prerequisites](https://github.com/hku-mars/FAST_LIO)

# RUN
## MAPPING
1. roslaunch fast_lio mapping_avia.launch(*It is slightly different from the fastlio*)
2. Using [FASTLIO](https://github.com/hku-mars/FAST_LIO), and then copy the generted map into **FAST_LIO_LOCALIZATION_PLUS/PCD**
## LOCALIZATION
1. roslaunch fast_lio localization_avia.launch
2. give the initial pose from rviz **2D Nav Goal** , otherwise, the hints '**** wait for initial pose'
![Selection_029](https://github.com/iDonghq/FAST_LIO_LOCALIZATION_PLUS/assets/23080413/a93c8700-27ab-4353-9080-50cc1c3b6a93)

# NOTE !!!
 1. Using **FAST_LIO_LOCALIZATION_PLUS** mapping, please set param **is_vehicle = false** in **avia.yaml** file if your robot is UAV.
 2. if your robot is vehicle, set the param **is_vehicle** true or not, it's up to your demands.

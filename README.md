# Introduction
The repo is pure localization for [FASTLIO](https://github.com/hku-mars/FAST_LIO) with given initial pose from rviz. It's completely developed in c++，and the usage is very easy. 
## DEMO
https://github.com/iDonghq/FAST_LIO_LOCALIZATION_PLUS/assets/23080413/5c0370d9-cbb6-41ee-b7e5-4cfc86806a35


https://github.com/iDonghq/FAST_LIO_LOCALIZATION_PLUS/assets/23080413/021bc6ec-9c35-4cdc-be67-bc0f195b61e3



# Convergence and Map Bending
## CONVERGENCE
Thanks to the robust of [FASTLIO](https://github.com/hku-mars/FAST_LIO), in the following video the initial pose that deviates greatly from the true one is given, **FAST_LIO_LOCALIZATION_PLUS** can also quickly converge to near the truth value.
![relocalization](https://github.com/iDonghq/FAST_LIO_LOCALIZATION_PLUS/assets/23080413/c7b60bbc-1412-45a6-a958-20abc1d82558)
## DEMO 
https://github.com/iDonghq/FAST_LIO_LOCALIZATION_PLUS/assets/23080413/d18ee041-649b-4b39-9082-54fa4fa4a5e5

# MAP BENDING
Not enough observations in the z direction from lidar leads to the map bending to z direction. In the following fig, the number of normal vector, which is the normal of plane fitted by neraby points, in the same as gravity direction is counted. The following pic shows that fewer and fewer z-axis observations with  the vehicle moves. 
![output](https://github.com/iDonghq/FAST_LIO_LOCALIZATION_PLUS/assets/23080413/07f1a65a-04cf-4c6f-8d0a-e5c992237376)
Correspondingly to the generted map is:
![Selection_033](https://github.com/iDonghq/FAST_LIO_LOCALIZATION_PLUS/assets/23080413/492f57c2-8a5e-4ae6-a908-fe675f4e73b6)
In  **FAST_LIO_LOCALIZATION_PLUS** mapping, the number of plane's normal fitted by neraby points is monitored, which is less than the threshhold I set, I brutely set the translation **z=0** from pose, the result is:
![Selection_034](https://github.com/iDonghq/FAST_LIO_LOCALIZATION_PLUS/assets/23080413/f90f98fe-d1d6-41a6-b13a-93c30c38401b)



# BUILD and DEPENDENCE 
The BUILD and DEPENDENCE is completely the same as [fastlio-Prerequisites](https://github.com/hku-mars/FAST_LIO)

# RUN
## MAPPING
1. roslaunch fast_lio mapping_avia.launch (*It is slightly different from the fastlio*).
2. Using [FASTLIO](https://github.com/hku-mars/FAST_LIO), and then copy the generted map into **FAST_LIO_LOCALIZATION_PLUS/PCD**
## LOCALIZATION
1. roslaunch fast_lio localization_avia.launch
2. give the initial pose from rviz **2D Nav Goal** , otherwise, the hints '**** wait for initial pose'
![Selection_029](https://github.com/iDonghq/FAST_LIO_LOCALIZATION_PLUS/assets/23080413/a93c8700-27ab-4353-9080-50cc1c3b6a93)



# NOTE !!!
 1. **In order to finish the initialization faster and more accuracy, the initial pose from rviz should be strictly limited within the area of 1 square meter(x*y).**
 2. Using **FAST_LIO_LOCALIZATION_PLUS** mapping, please set param **is_vehicle = false** in **avia.yaml** file if your robot is UAV.
 3. if your robot is vehicle, set the param **is_vehicle** true or not, it's up to your demands.

*Thanks Very Much To SHANGHAI TENGHAOSHIXIAN Tech For Providing ALL Datasets.*

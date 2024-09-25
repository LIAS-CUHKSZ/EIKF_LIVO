# Evaluation

## 1. Install ILIVE
Preliminary:
````c++
mkdir ILIVE/src
git clone https://github.com/LIAS-CUHKSZ/ILIVE.git
catkin_make
source devel/setup.bash
````



## 2. Prepare for the Chosen Dataset

Our project requests for sensor data in rosbag format. The format of Lidar should be PointCloud2. 

### Chosen datasets

| Dataset   | Parts of Sensors                                             | Evaluation        | Paper                                                        | code/website                                                 | Usage                                   |
| --------- | ------------------------------------------------------------ | ----------------- | ------------------------------------------------------------ | ------------------------------------------------------------ | --------------------------------------- |
| NTU VIRAL | IMU/Ouster16 LiDAR/monochome fisheye Camera/UWB/Leica(groundtruth) | matlab (provided) | NTU VIRAL: A Visual-Inertial-Ranging-Lidar Dataset, From an Aerial Vehicle Viewpoint [IJRR] | https://ntu-aris.github.io/ntu_viral_dataset/sensors_and_usage.html | Fast-LIVO                               |
| NCLT      | IMU/Velodyne 32E LiDAR/ Ladybug camera (color)/ RTK GPS/Standard GPS | evo               | University of Michigan North Campus long-term vision and lidar dataset [IJRR] | http://robots.engin.umich.edu/nclt/                          | Fast lio2, sr-lio, r3live++, faster-lio |
| URHK      | IMU/Velodyne32 LiDAR/ camera (fisheye color)/ SPAN-CPT       | evo               | UrbanLoco: A Full Sensor Suite Dataset for Mapping and Localization in Urban Scenes [ICRA2020] | https://github.com/weisongwen/UrbanLoco                      | sr-lio, fast-lio2, faster-lio           |
| UTBM      |                                                              | evo               |                                                              |                                                              | sr-lio, fast-lio2, faster-lio           |
| CUHKSZ    |                                                              | evo               |                                                              |                                                              |                                         |

### Information look ahead

To get more information please refer to the link of datasets.

#### NTU VIRAL

![image-20230709120512099](C:\Users\11811\AppData\Roaming\Typora\typora-user-images\image-20230709120512099.png)

#### NCLT

<img src="C:\Users\11811\AppData\Roaming\Typora\typora-user-images\image-20230709122900059.png" alt="image-20230709122900059" style="zoom:80%;" />

Transfer NCLT to rosbag

```python
git clone https://github.com/LIAS-CUHKSZ/DataProcessTools4SLAM.git
```

- *vel2pclrosbag.py*: convert LiDAR and IMU data to rosbag as pointcloud form and msg::IMU**.  这个只写了LiDAR的也可以记录IMU数据，需要一个人改编! 之前的IMU是我从别的lio算法里面已经转好的中间下载下来合并进去的！**
- *convert_image2rosbag.py*: convert Imaga data to rosbag.
- *merge_rosbag.py*: merge the selected topics of the selected rosbags to get one rosbag.

NCLT provides many scripts to turn the raw sensor data into rosbag, however the converted format is not we want. So we provide new scripts to get rosbag.

#### URHK

![image-20230709123539503](C:\Users\11811\AppData\Roaming\Typora\typora-user-images\image-20230709131006852.png)

![image-20230709131028836](C:\Users\11811\AppData\Roaming\Typora\typora-user-images\image-20230709131028836.png)

#### UTBM

to be filled





#### CUHKsz

to be filled



## 2. Create launch file and config file

### Create launch file

Please refer urbanloco.launch to modify

```
<param name="IMU_topic" type="string" value= "          " />
<param name="LiDAR_pointcloud_topic" type="string" value= "           " />
<param name="Image_topic" type="string" value= "         " />
<arg name="dataset"   default="    " />
<arg name="bag_path"   default="           " />
<arg name="path_save"   default="           " />
<node pkg="rosbag" type="play" name="play" args="$(arg bag_path)" required="true" /> -->
<node pkg="rosbag" type="record" name="record" args="                 -O $(arg path_save)/$(arg dataset)/test$(arg filter).bag" required="true" /> 
```

### Create Config file:

```python
ilive_vio:
   image_width: 
   image_height: 
   camera_intrinsic:[]
   camera_dist_coeffs: [] #k1, k2, p1, p2, k3
   camera_ext_R:[]
   camera_ext_t: []
```

```python
Lidar_front_end:
   lidar_type:    
   	# 0-mid, 1-Livox-horizon, 2-velodyne，3-Ouster
   N_SCANS: 
   using_raw_point: 
   point_step: 
```

```python
ilive_lio:        
   lidar_time_delay:        
   ext_t_lid_in_imu: [ ]
   ext_R_lid_in_imu: [ ]
   long_rang_pt_dis: 500   #300-800 for indoor, 800-2000 for outdoor   
```

```
ilive_IMU:
   max_IMU_calibration_num:
   COV_ACC_NOISE_DIAG:  
   COV_OMEGA_NOISE_DIAG: 
   COV_BIAS_ACC_NOISE_DIAG: 
   COV_BIAS_GYRO_NOISE_DIAG: 
```



### Start one run without evaluation:

```shell
roslaunch ilive      .launch  method:=      dataset:=   bag_path:=   iter:=   path_save:=
```

method: 'EIKF'  'EKF'  'InEKF'



## 3. Evaluation

### Use evo to evaluate

https://github.com/MichaelGrupp/evo

#### evaluation steps

1. once you run launch, the predicted trajectory and gps will be recorded as rosbag in  /output/output_{dataset name}/{bag_name}/{method}/test.bag

    ba, bg will be recorded in ba.txt and bg.txt saved in the same path with rosbag.

2. when the launch finish, use rostopic to save the predicted trajectory as csv in      /output/output_{dataset name}/{bag_name}/{method}/          .csv

   save the predicted trajectory as csv in      /output/output_{dataset name}/{bag_name}/{method}/        .csv

3. automatically transfer original format of gps to relative xyz and save in            /output/output_{dataset name}/{method}/{bag_name}/         .csv

4. use evo to transfer the  groundtruth and predicted path to the preferrable format [e.g. tum, euroc]

5. use evo to get the evaluation results: plot aligned path, ape, rmse, rpe

6. time evaluation: average time cost each frame

##### Remarks:

- NTU_viral: you can choose to use either matlab provided by dataset owner or evo to evaluate.

- Time offset between gps and odometry for each dataset are:

  *To be done*

  

- method combinition: EKF_lio_1. EKF_lio_2, EKF_lio_3, InEKF_lio_1, InEKF_lio_2, InEKF_lio_3, EIKF_lio_1,EKF_liv_1, EKF_liv_2, EKF_liv_3, InEKF_liv_1, InEKF_liv_2, InEKF_liv_3, EIKF_liv_1, InEKF_EIKF_liv_1,  InEKF_EIKF_liv_2, InEKF_EIKF_liv_3

- odometry comparison: Fast_lio2, Fast_livo, open_vins, orb_slam, r3live

- parameters to modify: IMU intrinsic(n_ba, n_bg, n_wa, n_wg), time offset (camera-IMU, Lidar-IMU, gps-odometry), initial covariance, long_rang_pt_dis

- To run evaluation_all.sh, you need to modify the path of config   path of launch and change the configurations of each round in method_data_config.txt:

  for each row, please clarify:   method dataset  bagpath  bagname   



#### Start Evaluation

```shell
source /opt/intel/oneapi/setvars.sh     
source devel/setup.bash

bash evaluation_one.sh
bash evaluation_all.sh
```



### Use Matlab to Evaluate--Specially for NTU_VIRAL

Please refer to https://github.com/ntu-aris/viral_eval and https://ntu-aris.github.io/ntu_viral_dataset/sensors_and_usage.html for tutorial.







## 分工

|                      |                |      |
| -------------------- | -------------- | ---- |
| NTU_Viral            | haoying li     |      |
| URHK                 | haoying Li     |      |
| NCLT                 | zimin+shenyuan |      20120615，20121201，20130110，20130405|
| UTBM                 | Shenyuan       |     20180719，20190131 |
| EKF bug              | qingcheng      |      |
| Eigen 加速           | yanglin        |      |
| origin上运行时间测速 | yanglin        |      |






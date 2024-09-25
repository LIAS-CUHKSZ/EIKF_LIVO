# evaluation steps:
# 1. once you run launch, the predicted trajectory and gps will be recorded as rosbag in  /output/output_{dataset name}/{bag_name}/{method}/test.bag
#    ba, bg will be recorded in ba.txt and bg.txt saved in the same path with rosbag. 
# 2. when the launch finish, use rostopic to save the predicted trajectory as csv in      /output/output_{dataset name}/{bag_name}/{method}/predict_odom.csv
#                                            save the predicted trajectory as csv in      /output/output_{dataset name}/{bag_name}/{method}/gt_gps_origin.csv
# 3. automatically transfer original format of gps to relative xyz and save in            /output/output_{dataset name}/{method}/{bag_name}/gt_gps_xyz.csv
# 4. use evo to transfer the predict_odom.csv, gt_gps_xyz.csv to the preferrable format [e.g. tum, euroc]
# 5. use evo to get the evaluation results: plot aligned path, ape, rmse, rpe 
# 6. time evaluation: average time cost each frame

# Remarks:
# NTU_viral: you can choose to use either matlab provided by dataset owner or evo to evaluate.
# time offset between gps and odometry for each dataset are: 
# dataset choice: NTU_viral, urhk, NCLT, cuhksz
# method choice: EKF_lio_1. EKF_lio_2, EKF_lio_3, InEKF_lio_1, InEKF_lio_2, InEKF_lio_3, EIKF_lio_1
#                EKF_liv_1, EKF_liv_2, EKF_liv_3, InEKF_liv_1, InEKF_liv_2, InEKF_liv_3, EIKF_liv_1, InEKF_EIKF_liv_1,  InEKF_EIKF_liv_2, InEKF_EIKF_liv_3
# odometry comparison: Fast_lio2, Fast_livo, open_vins, orb_slam, r3live 
# parameters to modify: IMU intrinsic(n_ba, n_bg, n_wa, n_wg), time offset (camera-IMU, Lidar-IMU, gps-odometry), initial covariance, long_rang_pt_dis
# To run evaluation_all.sh, you need to modify the method_data_config.txt:
#        for each row, please clarify: method dataset  bagpath  bagname   pathofconfig   pathoflaunch


#==============================================================START EVALUATION=============================================================
  
source ./devel/setup.bash
CURRENT_DIR=$(cd `dirname $0`; pwd)
echo ${CURRENT_DIR}
# 1.
# must no space around =  !!!
# dataset='urca'
# bag_path='/home/slam/workspace/dataset/locourban'
# bag_name='CA1'
# filter='InEKF' 
# odom='lio'
# max_iter=3


dataset='urhk'
# dataset='urca'
bag_path='/home/slam/workspace/dataset/locourban'
bag_name='urhk_1'
filter='InEKF' 
odom='lio'  #如果开lio就写这个  如果liv 就写这个
max_iter=2

# duration=     To do 200m      600m         800m         2000m對應的時間
output_dir="${CURRENT_DIR}/output/output_${dataset}/${bag_name}/${filter}_${odom}_${max_iter}"
gt_tr
# output_dir='/home/slam/workspace/ws_ILIVE/Evaluation_ILIVE/Fastlio2'
if [ ! -d $output_dir  ];then
mkdir -p $output_dir
echo new output_dir
else
echo output_dir exist
fi

# roslaunch ilive newer_college128.launch    method:=$filter    dataset:=$bag_name   bag_path:=$bag_path  iters:=$max_iter    path_save:=$output_dir
# roslaunch fast_lio mapping_velodyne.launch
roslaunch ilive urbanloco.launch    method:=$filter    dataset:=$bag_name   bag_path:=$bag_path  iters:=$max_iter    path_save:=$output_dir
echo  roslaunch finish, start saving to csv
# 
# # 2.       
rostopic echo -b ${output_dir}/test.bag  -p --nostr --noarr /aft_mapped_to_init > ${output_dir}/predict_odom.csv
rostopic echo -b ${output_dir}/test.bag  -p  --nostr --noarr  /novatel_data/inspvax > ${output_dir}/gt_gps_origin.csv


#rostopic echo -b ${output_dir}/test.bag  -p --nostr --noarr /aft_mapped_to_init > ${output_dir}/predict_odom.csv
# # # # 3. 

# python  ${CURRENT_DIR}/src/ilive/src/evaluation/RTK2XYZ/do_rtk2xyz.py  --input "${output_dir}/gt_gps_origin.csv"   --output  "${output_dir}/gt_gps_xyz.csv"


##### if don't compare z
# python  /home/slam/workspace/ws_ILIVE/Evaluation_ILIVE/ILIVE/src/ilive/src/evaluation/do_z0.py  --input "${output_dir}/gt_gps_xyz.csv"     --output  "${output_dir}/gt_gps_xyz_z0.csv"
# python  /home/slam/workspace/ws_ILIVE/Evaluation_ILIVE/ILIVE/src/ilive/src/evaluation/do_z0.py  --input "${output_dir}/predict_odom.csv"   --output  "${output_dir}/predict_odom_z0.csv"


# # 4. 
cd ${output_dir}
# evo_traj euroc ${output_dir}/gt_gps_xyz_z0.csv   --save_as_tum      
# evo_traj euroc ${output_dir}/predict_odom_z0.csv  --save_as_tum      
evo_traj euroc ${output_dir}/gt_gps_xyz.csv   --save_as_tum      
evo_traj euroc ${output_dir}/predict_odom.csv  --save_as_tum      


# 5.  
time_offs=-0.0
t_maxd=0.10

evo_traj tum ${output_dir}/gt_gps_xyz.tum  ${output_dir}/predict_odom.tum  --ref ${output_dir}/gt_gps_xyz.tum    -as  --t_offset ${time_offs}    --t_max_diff  ${t_maxd}  -pa     --no_warnings 
# # evo_rpe tum ${output_dir}/gt_gps_xyz_z0.tum  ${output_dir}/predict_odom_z0.tum        --t_offset ${time_offs} -p  -as   --no_warnings   --no_warnings 

evo_ape tum ${output_dir}/gt_gps_xyz.tum  ${output_dir}/predict_odom.tum     -r trans_part  --t_offset ${time_offs} -pa   -as  --t_max_diff ${t_maxd}    --no_warnings 



# evo_traj tum ${output_dir}/gt_gps_xyz.tum  ${output_dir}/predict_odom.tum  --ref ${output_dir}/gt_gps_xyz.tum    -as  --t_offset ${time_offs}    --t_max_diff  ${t_maxd}  -pa     --no_warnings 
# # evo_rpe tum ${output_dir}/gt_gps_xyz_z0.tum  ${output_dir}/predict_odom_z0.tum        --t_offset ${time_offs} -p  -as   --no_warnings   --no_warnings 

# evo_ape tum ${output_dir}/gt_gps_xyz.tum  ${output_dir}/predict_odom.tum     -r trans_part  --t_offset ${time_offs} -pa   -as  --t_max_diff ${t_maxd}    --no_warnings 

# time_offs=0.0
# t_maxd=0.5

# # evo_traj tum ${bag_path}/gt_math_medium.csv --ref ${bag_path}/gt_math_medium.csv -as --t_offset ${time_offs} --t_max_diff ${t_maxd} -pa --no_warnings
# evo_traj tum  ${output_dir}/predict_odom.tum  --ref  ${output_dir}/predict_odom.tum  -as --t_offset ${time_offs} --t_max_diff ${t_maxd} -pa --no_warnings
# evo_ape tum ${bag_path}/gt_math_medium.csv   ${output_dir}/predict_odom.tum     -r trans_part  --t_offset ${time_offs} -pa   -as  --t_max_diff ${t_maxd}
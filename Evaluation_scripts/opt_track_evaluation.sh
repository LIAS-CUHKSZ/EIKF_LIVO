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


dataset='LAB'
bag_path='/home/haoying/dataset'
bag_name='easy1'
filter='EKF' 
odom='liv'
max_iter=3


output_dir="${CURRENT_DIR}/output/output_${dataset}/${bag_name}/${filter}_${odom}_${max_iter}"

if [ ! -d $output_dir  ];then
mkdir -p $output_dir
echo new output_dir
else
echo output_dir exist
fi

roslaunch ilive ilive_mid.launch    method:=$filter   dataset:=$bag_name  bag_path:=$bag_path  iters:=$max_iter    path_save:=$output_dir   dataset:=$bag_name   bag_path:=$bag_path  iters:=$max_iter    path_save:=$output_dir

# echo  roslaunch finish, start saving to csv
# 
# # # 2.       
rostopic echo -b ${output_dir}/test.bag  -p --nostr --noarr /aft_mapped_to_init > ${output_dir}/predict_odom.csv

# # ## 3. 
python3 /home/haoying/Codes/ILIVE-lxh1/ILIVE-lxh/src/ilive/src/evaluation/align_tracker.py  -i "${output_dir}/predict_odom.csv"  -t "/home/haoying/Desktop/LAB_gt/gt_easy1.csv"  -o  "${output_dir}/gt_tracker.csv"
# # # # 4.  
cd ${output_dir}
evo_traj euroc ${output_dir}/predict_odom.csv  --save_as_tum     
evo_traj euroc ${output_dir}/gt_tracker.csv  --save_as_tum     

# # # # 5.  

# for j in  3.1
# do
# for m in   0.1  
# do 

# echo  j ${j}  m ${m}
# time_offs=${j}
# t_maxd=${m}

# evo_traj tum ${output_dir}/predict_odom.tum   --ref ${output_dir}/gt_tracker.tum      --t_offset ${time_offs}    --t_max_diff  ${t_maxd}  -a     --no_warnings  --correct_scale
# evo_ape tum ${output_dir}/gt_tracker.tum  ${output_dir}/predict_odom.tum     -r trans_part  --t_offset ${time_offs} -a    --t_max_diff ${t_maxd}    --no_warnings   --correct_scale

# done
# done

time_offs=3.1

t_maxd=0.1

evo_traj tum ${output_dir}/predict_odom.tum   --ref ${output_dir}/gt_tracker.tum      --t_offset ${time_offs}    --t_max_diff  ${t_maxd}  -pa     --no_warnings 
evo_ape tum ${output_dir}/gt_tracker.tum  ${output_dir}/predict_odom.tum     -r trans_part  --t_offset ${time_offs} -pa    --t_max_diff ${t_maxd}    --no_warnings  

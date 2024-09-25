# Dataset website: https://mcdviral.github.io/Download.html
# Usage:
# 1. merge rosbags.
# 2. Gt

#####First step: The settings of running the code
source ./devel/setup.bash
CURRENT_DIR=$(cd `dirname $0`; pwd)
echo ${CURRENT_DIR}

######Second step: The settings of the dataset and the output directory and the method
bag_path='/home/slam/workspace/dataset/ntu2/tuhh'
DATA='NTU_MCD'
bag_name='tuhh_lidar_camera_imu'
filter='InEKF' 
odom='liv'  #如果开lio就写这个  如果liv 就写这个
max_iter=2
output_dir="${CURRENT_DIR}/output/output_${DATA}/${bag_name}/${filter}_${odom}_${max_iter}"
sudo cp -r ${bag_path}/pose_inW.csv ${output_dir}
echo ${output_dir}

if [ ! -d $output_dir  ];then
mkdir -p $output_dir
echo new output_dir
else
echo output_dir exist
fi

######Third step: The settings of the rosbag, and run the dataset
roslaunch ilive NTU_Viral2.launch      method:=$filter   dataset:=$bag_name  bag_path:=$bag_path  iters:=$max_iter    path_save:=$output_dir   dataset:=$bag_name   bag_path:=$bag_path  iters:=$max_iter    path_save:=$output_dir
echo  roslaunch finish, start saving to csv
# # # 

######Forth step: Extract the estimation in the Output.bag   
rostopic echo -b ${output_dir}/test.bag  -p --nostr --noarr /aft_mapped_to_init > ${output_dir}/predict_odom.csv
echo ${bag_path}

######Fifth step: Time alignment of the estimation and the ground truth
python3 /home/slam/workspace/ws_ILIVE/Evaluation_ILIVE/ILIVE/srcilive/src/evaluation/align_tracker.py -i "${output_dir}/predict_odom.csv"  -t "${output_dir}/pose_inW0.csv"  -o  "${output_dir}/pose_inW1.csv"
cd ${output_dir}/

######Sixth step: Convert the csv to tum format for evo
evo_traj euroc ${output_dir}/pose_inW1.csv  --save_as_tum       
evo_traj euroc ${output_dir}/predict_odom.csv  --save_as_tum      


#####Seventh step: Evaluate the estimation
time_offs=-0.0
t_maxd=100
evo_config set plot_seaborn_style whitegrid
evo_traj tum ${output_dir}/predict_odom.tum   --ref ${output_dir}/pose_inW1.tum   -as  -pa



############Back up Commands
# evo_traj tum ${output_dir}/predict_odom1.tum   ${output_dir}/predict_odom2.tum   --ref ${output_dir}/pose_inW1.tum   -as  -pa

# evo_traj tum ${output_dir}/pose_inW0.tum  ${output_dir}/predict_odom.tum  --ref ${output_dir}/gt_gps_xyz.tum    -as  --t_offset ${time_offs}    --t_max_diff  ${t_maxd}  -pa     --no_warnings 
# # # evo_rpe tum ${output_dir}/gt_gps_xyz_z0.tum  ${output_dir}/predict_odom_z0.tum        --t_offset ${time_offs} -p  -as   --no_warnings   --no_warnings 

# evo_ape tum ${output_dir}/gt_gps_xyz.tum  ${output_dir}/predict_odom.tum     -r trans_part  --t_offset ${time_offs} -pa   -as  --t_max_diff ${t_maxd}    --no_warnings 



# # evo_traj tum ${output_dir}/gt_gps_xyz.tum  ${output_dir}/predict_odom.tum  --ref ${output_dir}/gt_gps_xyz.tum    -as  --t_offset ${time_offs}    --t_max_diff  ${t_maxd}  -pa     --no_warnings 
# # evo_rpe tum ${output_dir}/gt_gps_xyz_z0.tum  ${output_dir}/predict_odom_z0.tum        --t_offset ${time_offs} -p  -as   --no_warnings   --no_warnings 

# evo_ape tum ${output_dir}/gt_gps_xyz.tum  ${output_dir}/predict_odom.tum     -r trans_part  --t_offset ${time_offs} -pa   -as  --t_max_diff ${t_maxd}    --no_warnings 

# time_offs=0.0
# t_maxd=0.5

# # evo_traj tum ${bag_path}/gt_math_medium.csv --ref ${bag_path}/gt_math_medium.csv -as --t_offset ${time_offs} --t_max_diff ${t_maxd} -pa --no_warnings
# evo_traj tum  ${output_dir}/predict_odom.tum  --ref  ${output_dir}/predict_odom.tum  -as --t_offset ${time_offs} --t_max_diff ${t_maxd} -pa --no_warnings
# evo_ape tum ${bag_path}/gt_math_medium.csv   ${output_dir}/predict_odom.tum     -r trans_part  --t_offset ${time_offs} -pa   -as  --t_max_diff ${t_maxd}
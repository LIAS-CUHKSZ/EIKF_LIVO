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
#        for each row, please clarify: dataset,bag_path,bag_name,filter,odom,max_iter, launch file, config file
# You can only compare one dataset at one time

#==============================================================START EVALUATION=============================================================
source ./devel/setup.bash
CURRENT_DIR=$(cd `dirname $0`; pwd)
dataset_arr=()
bag_path_arr=()
bag_name_arr=()
filter_arr=()
odom_arr=()
max_iter_arr=()
lanuch_file=()

output_dir_arr=()

cnt=1
row=0



for line in `cat  run_config.txt`
do
    if [ $((${cnt} % 7)) == 1 ]; then
        dataset_arr[${row}]=${line}
        
    fi
    if [ $((${cnt} % 7)) == 2 ]; then
        bag_path_arr[${row}]=${line}
    
    fi
    if [ $((${cnt} % 7)) == 3 ]; then
        bag_name_arr[${row}]=${line}
        
    fi
    if [ $((${cnt} % 7)) == 4 ]; then
        filter_arr[${row}]=${line}
        
    fi
    if [ $((${cnt} % 7)) == 5 ]; then
        odom_arr[${row}]=${line}
        
    fi
    if [ $((${cnt} % 7)) == 6 ]; then
        max_iter_arr[${row}]=${line}
      

    fi 

    if [ $((${cnt} % 7)) == 0 ]; then
        lanuch_file[${row}]=${line}
        row=$(($row+1))

    fi 
    cnt=$(($cnt+1))
done


RERUN=1

echo ========number of runs==========
echo ${row}   # row is actually the number of runs
echo ================================
for i in $(seq 0  `expr ${row} - 1`)
do 
echo ~~~~~~~~~~~Rounds ${i}~~~~~~~~~~
dataset=${dataset_arr[${i}]}
bag_path=${bag_path_arr[${i}]}
bag_name=${bag_name_arr[${i}]}
filter=${filter_arr[${i}]}
odom=${odom_arr[${i}]}
max_iter=${max_iter_arr[${i}]}
launch_now=${lanuch_file[${i}]}
echo PARMETERS:
echo ${dataset}
echo ${bag_path}
echo ${bag_name}
echo ${filter}
echo ${odom}
echo ${max_iter}
echo ${launch_now}
echo ${CURRENT_DIR}
echo ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
output_dir="${CURRENT_DIR}/output/output_${dataset}/${bag_name}/${filter}_${odom}_${max_iter}"
output_dir_arr[${i}]=${output_dir}

if [ ! -d $output_dir  ];then
mkdir -p $output_dir
echo new output_dir
else
echo output_dir exist
fi


if [ ${RERUN} == 1 ]; then
#1.
# roslaunch ilive ${launch_now}    method:=$filter    dataset:=$bag_name   bag_path:=$bag_path  iters:=$max_iter    path_save:=$output_dir
echo  roslaunch finish, start saving to csv
# 
# # 2.       
rostopic echo -b ${output_dir}/test.bag  -p --nostr --noarr /aft_mapped_to_init > ${output_dir}/predict_${filter}_${odom}_${max_iter}.csv
rostopic echo -b ${output_dir}/test.bag  -p --nostr --noarr  /novatel_data/inspvax > ${output_dir}/gtr_${filter}_${odom}_${max_iter}.csv

# # 3. 
python  ${CURRENT_DIR}/src/ilive/src/evaluation/RTK2XYZ/do_rtk2xyz.py  --input "${output_dir}/gtr_${filter}_${odom}_${max_iter}.csv"   --output  "${output_dir}/gt_${filter}_${odom}_${max_iter}.csv"   
##### if don't compare z
python  /home/slam/workspace/ws_ILIVE/Evaluation_ILIVE/ILIVE/src/ilive/src/evaluation/do_z0.py  --input "${output_dir}/gt_${filter}_${odom}_${max_iter}.csv"     --output  "${output_dir}/gt_${filter}_${odom}_${max_iter}_z0.csv"
python  /home/slam/workspace/ws_ILIVE/Evaluation_ILIVE/ILIVE/src/ilive/src/evaluation/do_z0.py  --input "${output_dir}/predict_${filter}_${odom}_${max_iter}.csv"   --output  "${output_dir}/predict_${filter}_${odom}_${max_iter}_z0.csv"


fi

# # 4. 
cd ${output_dir}
evo_traj euroc ${output_dir}/gt_${filter}_${odom}_${max_iter}.csv   --save_as_tum      
evo_traj euroc ${output_dir}/predict_${filter}_${odom}_${max_iter}.csv  --save_as_tum      

# 5.  
time_offs=0
t_maxd=0.1

evo_traj tum ${output_dir}/gt_${filter}_${odom}_${max_iter}.tum  ${output_dir}/predict_${filter}_${odom}_${max_iter}.tum  --ref ${output_dir}/gt_${filter}_${odom}_${max_iter}_z0.tum     --align  --t_offset ${time_offs}    --t_max_diff  ${t_maxd}  -as  
evo_ape tum ${output_dir}/gt_${filter}_${odom}_${max_iter}.tum  ${output_dir}/predict_${filter}_${odom}_${max_iter}.tum       --t_offset ${time_offs}   -as   --t_max_diff ${t_maxd}      -r trans_part   --save_results ${CURRENT_DIR}/output/output_${dataset}/${bag_name}/${filter}_${odom}_${max_iter}.zip   --no_warnings
# evo_rpe tum ${output_dir}/gt_${filter}_${odom}_${max_iter}_z0.tum  ${output_dir}/predict_${filter}_${odom}_${max_iter}_z0.tum        --t_offset ${time_offs}   -as   --no_warnings

done




# # # echo .........Comparison...........o~TAT~o..............................  
echo ${dataset_arr[0]}
evo_res ${CURRENT_DIR}/output/output_${dataset}/${bag_name}/*.zip  -p --save_table ${CURRENT_DIR}/output/output_${dataset}/comparison_table.csv  

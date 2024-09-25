source ./devel/setup.bash
# gnome-terminal -- 

# 'eee_01' 'eee_02'  'eee_03'  'nya_01'   'nya_02' 
cnt=1
for i in 'eee_01'
#  'nya_03'   'sbs_01'   'sbs_02'   'sbs_03'  'spms_01'   'tnp_03'   'eee_01' 'eee_02'  'eee_03'  'nya_01'   'nya_02' 
do 
   for covA in  100 
   do 
   for covB in 100
   do

    for f in 'InEKF' 
    do 
        for j in 3
        do
            for m in  1
            do 
                
                echo ============================================== 
                echo start processing no. ${cnt} :
                echo  rosbag:        ${i}                                         
                echo  method:        ${f}                                          
                echo  max iteration: ${j}      
                echo  use vio:       ${m}                          
                echo ==============================================                              

                output_dir="/home/slam/workspace/ws_ILIVE/Evaluation_ILIVE/ILIVE/output/NTUviral_output/LIV${f}iter${j}f2f${m}f2m${m}/${i}"
                if [ ! -d $output_dir  ];then
                mkdir -p $output_dir
                echo new output_dir
                else
                echo output_dir exist
                fi
            
                roslaunch ilive NTU_VIRAL.launch   method:=$f    dataset:=$i    iters:=$j    path_save:=$output_dir   if_vio_f2f:=$m    if_vio_f2map:=$m
                
                echo roslaunch finish, start saving to csv

                eva_dir="/home/slam/workspace/ws_ILIVE/Evaluation_Data/NTU_eva/viral_eval-master/LIV${i}/${f}Iter${j}f2f${m}f2m${m}_COVA${covA}B${covB}"
                if [ ! -d $eva_dir  ];then
                mkdir -p $eva_dir
                echo new evaluation_dir
                else
                echo evaluation_dir exist
                fi
                
                rostopic echo -b ${output_dir}/test.bag  -p --nostr --noarr /aft_mapped_to_init > ${eva_dir}/predict_odom.csv
                rostopic echo -b ${output_dir}/test.bag  -p --nostr --noarr /leica/pose/relative > ${eva_dir}/leica_pose.csv

                echo ...............saving finished.................
                cnt=$(($cnt+1))
       
            done
        done
        done
        done
    done




    #    for f in 'EIKF'  
    #     do 
    #     for j in 1  

    #     do
    #         for m in 0  1
    #         do 
          
                
    #             echo ============================================== 
    #             echo start processing no. ${cnt} :
    #             echo  rosbag:        ${i}                                         
    #             echo  method:        ${f}                                          
    #             echo  max iteration: ${j}      
    #             echo  use vio:       ${m}                          
    #             echo ==============================================                              

    #             output_dir="/home/slam/workspace/ws_ILIVE/Evaluation_ILIVE/ILIVE/output/NTUviral_output/formal${f}iter${j}f2f${m}f2m${m}/${i}"
    #             if [ ! -d $output_dir  ];then
    #             mkdir -p $output_dir
    #             echo new output_dir
    #             else
    #             echo output_dir exist
    #             fi
            
    #             roslaunch ilive NTU_VIRAL.launch  method:=$f    dataset:=$i    iters:=$j    path_save:=$output_dir   if_vio_f2f:=$m    if_vio_f2map:=$m
                
    #             echo roslaunch finish, start saving to csv

    #             eva_dir="/home/slam/workspace/ws_ILIVE/Evaluation_Data/NTU_eva/viral_eval-master/formal_${i}/${f}Iter${j}f2f${m}f2m${m}_{formal}"
    #             if [ ! -d $eva_dir  ];then
    #             mkdir -p $eva_dir
    #             echo new evaluation_dir
    #             else
    #             echo evaluation_dir exist
    #             fi
                
    #             rostopic echo -b ${output_dir}/test.bag  -p --nostr --noarr /aft_mapped_to_init > ${eva_dir}/predict_odom.csv
    #             rostopic echo -b ${output_dir}/test.bag  -p --nostr --noarr /leica/pose/relative > ${eva_dir}/leica_pose.csv

    #             echo ...............saving finished.................
    #             cnt=$(($cnt+1))
       
    #         done
    #     done
    # done

done



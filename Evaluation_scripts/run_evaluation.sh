source ./devel/setup.bash
# gnome-terminal -- 

#for i in  'eee_03'    'nya_01'    'rtp_03'    'tnp_03'    'sbs_01'
for i in 'sbs_01'
do 

    for f in 'EKF' 'InEKF' 'EIKF'  #iter = 1
    do 
        for j in 1
        do
  

            output_dir="/home/slam/workspace/ws_ILIVE/Evaluation_ILIVE/ILIVE/output/NTUviral_output/${f}iter${j}/${i}/"
            if [ ! -d $output_dir  ];then
            mkdir -p $output_dir
            echo new output_dir
            else
            echo output_dir exist
            fi
        
            roslaunch ilive NTU_VIRAL_bag.launch  method:=$f    dataset:=$i    iters:=$j    path_save:=$output_dir
            
            echo roslaunch finish, start saving to csv

            eva_dir="/home/slam/workspace/ws_ILIVE/Evaluation_Data/NTU_eva/viral_eval-master/result_${i}/eva_${f}iter${j}"
            if [ ! -d $eva_dir  ];then
            mkdir -p $eva_dir
            echo new evaluation_dir

            else
            echo evaluation_dir exist
            fi
           
            rostopic echo -b ${output_dir}/test.bag  -p --nostr --noarr /aft_mapped_to_init > ${eva_dir}/predict_odom.csv
            rostopic echo -b ${output_dir}/test.bag  -p --nostr --noarr /leica/pose/relative > ${eva_dir}/leica_pose.csv

            echo saving finished
            
        done
    done


    # for f in 'EKF'    'InEKF'
    # do 
    #     for j in 2  3
    #     do
            
    #         output_dir="/home/slam/workspace/ws_ILIVE/Evaluation_ILIVE/ILIVE/output/NTUviral_output/${f}iter${j}/${i}/"
    #         if [ ! -d $output_dir  ];then
    #         mkdir -p $output_dir
    #         echo new output_dir
    #         else
    #         echo output_dir exist
    #         fi
    #         roslaunch ilive NTU_VIRAL_bag.launch    method:=$f    dataset:=$i    iters:=$j    path_save:=$output_dir
    #         echo roslaunch finish, start saving to csv

    #         eva_dir="/home/slam/workspace/ws_ILIVE/Evaluation_Data/NTU_eva/viral_eval-master/result_${i}/eva_${f}iter${j}"
    #         if [ ! -d $eva_dir  ];then
    #         mkdir -p $eva_dir
    #         echo new evaluation_dir
    #         else
    #         echo evaluation_dir exist
    #         fi
           
    #         rostopic echo -b ${output_dir}/test.bag  -p --nostr --noarr /aft_mapped_to_init > ${eva_dir}/predict_odom.csv
    #         rostopic echo -b ${output_dir}/test.bag  -p --nostr --noarr /leica/pose/relative > ${eva_dir}/leica_pose.csv

    #         echo saving finished
    #         cnt=cnt+1

    #     done
    # done 
done



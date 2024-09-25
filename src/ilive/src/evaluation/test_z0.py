import pandas as pd




def z0(file,out):
   
    data = pd.read_csv(file, dtype=float)
    print(data)
    data.iloc[:,5] = data.iloc[:,5]-data.iloc[:,5];   
    data.to_csv(out,index=False)



file="/home/slam/workspace/ws_ILIVE/Evaluation_ILIVE/ILIVE/output/output_urca/urca2r/InEKF_lio_2/gt_gps_xyz.csv"
outfile="/home/slam/workspace/ws_ILIVE/Evaluation_ILIVE/ILIVE/output/output_urca/urca2r/InEKF_lio_2/gt_gps_xyz_z0.csv"

z0(file, outfile)
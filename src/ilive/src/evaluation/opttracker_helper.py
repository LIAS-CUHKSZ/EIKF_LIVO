import pandas as pd




def align_time(file,opt_file,out_csv):
    pd.set_option('display.float_format',lambda x : '%.2f' % x)
    data = pd.read_csv(file, dtype=float)
    initial_time=data.iloc[0,0]
    odom_time=data.iloc[:,0]-initial_time

    print(initial_time)  
    
    
    gt_data=pd.read_csv(opt_file, dtype=float)
    print(gt_data)

    time_before=gt_data.iloc[:,0]
    time_after=time_before*10e8 + initial_time
    print(time_after)
    gt_data.iloc[:,0]=time_after
 
    gt_data.to_csv(out_csv,index=False,header=None)






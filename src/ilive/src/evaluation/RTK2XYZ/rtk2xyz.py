from pyproj import CRS
from pyproj import Transformer
import pandas as pd
try:
    import numpy as np
except:
    print("You can install numpy to improve the accuracy of the result.")

def rtk2xyz(input_csv, output_csv, epsg = 32650, flag = False, base = []):
    """[RTK].csv to [XYZ].csv

    Args:
        header (string list): header of the csv file (can only includes the columns you need)
        input_csv (string): path of RTK csv file
        output_csv (string): path of XYZ csv file
        epsg (int): epsg code of the utm zone or other projected coordinate (default: 32650 which is the code of UTM50N)
        flag (bool): whether to use numpy to improve the accuracy using float64 (default: False)
        base (number list): [longitude, latitude] of the base point (default: [])
    """
    
    rtk_crs = CRS.from_epsg(4326) # WGS84 -- GPS所用坐标系统
    utm_crs = CRS.from_epsg(epsg) # UTM50N -- UTM投影坐标系统（正东方向为x轴正方向，正北方向为y轴正方向）
    transformer = Transformer.from_crs(rtk_crs, utm_crs)

    # 从csv读取
    if(flag):
        data = pd.read_csv(input_csv,usecols=[0,1,2,15,16,17],dtype=np.float64)
    else:
        data = pd.read_csv(input_csv,usecols=[0,1,2,15,16,17],   dtype=float)
    # print(data[2][3])
    
    # 基准点
    if(base==[]):
        base_longitude =data.iloc[0,4]
        base_latitude = data.iloc[0,3]
        base_x , base_y = transformer.transform(base_latitude,base_longitude)

    else:
        base_x , base_y = transformer.transform(base[1],base[0])
        

    # 整体运算
    data.iloc[:,4],data.iloc[:,3] = transformer.transform(data.iloc[:,3],data.iloc[:,4])
    data.iloc[:,4]= data.iloc[:,4] - base_x
    data.iloc[:,3] =data.iloc[:,3]- base_y

    # data.rename(columns={'longitude':'x','latitude':'y','altitude':'z'},inplace=True)

    # 输出到csv
    data.to_csv(output_csv,index=False)

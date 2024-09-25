## [RTK] .bag -> [XYZ] .csv

to convert the RTK data from .bag file to the projected utm coordinate system (XYZ) in .csv file

### 0. reindex

```bash
# run the following command
$ rosbag play xxx.bag
# if output:
ERROR bag unindexed: xxx.bag.  Run rosbag reindex.
# then run 
$ rosbag reindex xxx.bag
# this will generate 2 .bag file:
$ ls
xxx.orig.bag xxx.bag
# xxx.orig.bag is the origin bag and we should use xxx.bag
```

### 1. [RTK] .bag -> [RTK] .csv

```bash
# command
# BAGFILE是bag文件，TOPIC为数据所在的topic
$ rostopic echo -b <BAGFILE> -p <TOPIC> > <output>.csv

# e.g.
$ rostopic echo -b ./src/nmea_msgs/rs_mid_xs_2023-5-17-15-58-43.bag -p /nmea_sentence > output.csv
```

### 2. [RTK] .csv -> [XYZ] .csv

using the [python script](./rtk2xyz.py).

and the [jupyter notebook file](./rtk2xyz.ipynb) may be helpful if you want to test or read the code.

```bash
pip install pandas
pip install pyproj
pip install numpy (optional)
cd RTK2XYZ
python -i rtk2xyz.py
```
```python
>>> rtk2xyz(['longitude','latitude','altitude'],'input.csv', 'output.csv')
>>> quit()
```

Parameters:

- **header**

    a list of string including the header of the csv file (can only includes the columns you need).
    must include the following labels:
    - longitude
    - latitude
    - altitude

- **input_csv**
- **output_csv**

    the input and output csv file path (support both relative path and absolute path)

- **epsg**: 

    the epsg code of the utm zone (or other projected coordinates) you want to convert to, which can be found [here](https://epsg.io/).
    
    - to identify the utm zone, you can use [this website](https://awsm-tools.com/lat-long-to-utm)

    - to find the epsg code corresponding to the utm zone, you can use [this website](https://epsg.io/)

    default: epsg=32650 (utm zone 50N)

    Area of use: Between 114°E and 120°E, northern hemisphere between equator and 84°N, onshore and offshore

    Brunei. China. Hong Kong. Indonesia. Malaysia - East Malaysia - Sarawak. Mongolia. Philippines. Russian Federation. Taiwan.

- **flag**
    
    whether to use numpy to improve the accuracy using float64 (default: False)

- **base**

    a size 2 list of number representing the base coordinate in RTK (default: None)

    base[0] should be the longitude of the base coordinate, base[1] should be the latitude of the base coordinate

    if base is None, the base coordinate will be the first coordinate in the csv file

    if base is not None, the base coordinate will be the coordinate you set

    e.g. base = [114.123456, 22.123456]

&copyright; 2023 [LysanderT](https://github.com/LysanderT/RTK2XYZ)
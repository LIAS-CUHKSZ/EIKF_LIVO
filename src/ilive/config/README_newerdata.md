How to get intrinsic and extrinsic parameters?


1. spatio-temporal calibration between the cameras and 2 IMUs embedded in the realsense and the Ouster. [Kalibr][calibration dataset provided]
- Use Ouster IMU: [rooster_2020-03-11-10-05-35_0.bag]
(calibration rosbag, including RealSense camera images and Realsense and Ouster IMU messages captured in front of an april board (Note: Realsense IMU messages are at 650 Hz)
<I didn't run this bag, just use the txt file directly>

also results after Kalibr are provided: [results-ouster_imu-cam_2020-03-11-10-05-35_0.txt]
(this file contains the intrinsic and extrinsic parameters between the right and the left cameras relative to the Ouster IMU.)
<We use cam0 here>


2. they found that the beam intrinsics of the Ouster we used differ from the designed values.
[beam_intrinsics.json] <But I don't know how this "beam" affect the results>

3. the extrinsic parameters between Ouster and IMU is given by handware configuration. <I don't know if there exists other ways provided by the authors.>

4.LiDAR-IMU

-ouster 64

```rosrun tf tf_echo /base /os_sensor```

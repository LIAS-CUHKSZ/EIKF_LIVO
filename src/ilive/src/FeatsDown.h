
#ifndef ILIVE_FEATS_DOWN_H
#define ILIVE_FEATS_DOWN_H

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
struct FeatsDown{
    PointCloudXYZINormal::Ptr original;
    PointCloudXYZINormal::Ptr updated;
    int size;
};
#endif //ILIVE_FEATS_DOWN_H

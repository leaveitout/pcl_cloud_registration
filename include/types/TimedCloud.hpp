//
// Created by sean on 07/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_TIMEDCLOUD_HPP
#define PCL_CLOUD_REGISTRATION_TIMEDCLOUD_HPP

#include <pcl/point_cloud.h>
#include "TimedData.hpp"

template <typename PointType>
class TimedCloud : public TimedData<typename pcl::PointCloud<PointType>::ConstPtr> {
    typedef typename pcl::PointCloud<PointType>::ConstPtr CloudConstPtr;

public:
    TimedCloud(CloudConstPtr cloud, size_t timestamp) :
            TimedData<CloudConstPtr>(cloud, timestamp) {
    }
};

#endif //PCL_CLOUD_REGISTRATION_TIMEDCLOUD_HPP

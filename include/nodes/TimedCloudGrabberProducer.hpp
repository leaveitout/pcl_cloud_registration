//
// Created by sean on 06/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_TIMEDCLOUDGRABBERPRODUCER_HPP
#define PCL_CLOUD_REGISTRATION_TIMEDCLOUDGRABBERPRODUCER_HPP

#include <pcl/point_cloud.h>
#include "nodes/TimedDataGrabberProducer.hpp"

template <typename PointType>
class TimedCloudGrabberProducer : public TimedDataGrabberProducer<typename pcl::PointCloud<PointType>::ConstPtr> {
    typedef typename pcl::PointCloud<PointType>::ConstPtr CloudConstPtr;

    static constexpr const char* DESCRIPTION = "Cloud Grabber";

public:
    TimedCloudGrabberProducer(const std::shared_ptr<GuardedGrabber> &grabber,
                              size_t id,
                              size_t buffer_size = Buffer<CloudConstPtr>::DEFAULT_BUFF_SIZE) :
            Node(id, DESCRIPTION),
            TimedDataGrabberProducer<typename pcl::PointCloud<PointType>::ConstPtr>(grabber,
                                                                                    id,
                                                                                    DESCRIPTION,
                                                                                    buffer_size) {
    }
};

#endif //PCL_CLOUD_REGISTRATION_TIMEDCLOUDGRABBERPRODUCER_HPP


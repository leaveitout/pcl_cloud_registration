//
// Created by sean on 09/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_TIMEDCLOUDANDIMAGEPAIR_HPP
#define PCL_CLOUD_REGISTRATION_TIMEDCLOUDANDIMAGEPAIR_HPP

#include "TimedImage.hpp"
#include "TimedCloud.hpp"

template <typename PointType>
class TimedCloudAndImagePair {
    typedef std::shared_ptr<TimedCloud<PointType>> TimedCloudPtr;
    typedef std::shared_ptr<TimedImage> TimedImagePtr;

public:
    TimedCloudAndImagePair(TimedCloudPtr cloud, TimedImagePtr image) :
            cloud_ (cloud),
            image_ (image) {
    }

    TimedCloudPtr getTimedCloud() const {
        return cloud_;
    }

    TimedImagePtr getTimedImage() const {
        return image_;
    }

private:
    TimedCloudPtr cloud_;
    TimedImagePtr image_;
};
#endif //PCL_CLOUD_REGISTRATION_TIMEDCLOUDANDIMAGEPAIR_HPP

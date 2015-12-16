//
// Created by sean on 07/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_TIMEDIMAGE_HPP
#define PCL_CLOUD_REGISTRATION_TIMEDIMAGE_HPP

#include <pcl/io/image.h>
#include "TimedData.hpp"

class TimedImage : public TimedData<pcl::io::Image::Ptr> {
    typedef pcl::io::Image::Ptr ImagePtr;

public:
    TimedImage(pcl::io::Image::Ptr image, size_t timestamp) :
            TimedData<ImagePtr>(image, timestamp) {
    }
};

#endif //PCL_CLOUD_REGISTRATION_TIMEDIMAGE_HPP

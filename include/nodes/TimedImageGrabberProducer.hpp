//
// Created by sean on 06/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_TIMEDIMAGEGRABBERPRODUCER_HPP
#define PCL_CLOUD_REGISTRATION_TIMEDIMAGEGRABBERPRODUCER_HPP

#include <pcl/io/image.h>
#include "nodes/TimedDataGrabberProducer.hpp"

class TimedImageGrabberProducer : public TimedDataGrabberProducer<pcl::io::Image::Ptr> {
    typedef pcl::io::Image::Ptr ImagePtr;

    static constexpr const char* DESCRIPTION = "Image Grabber";

public:
    TimedImageGrabberProducer(const std::shared_ptr<GuardedGrabber> &grabber,
                              size_t id,
                              size_t buffer_size = Buffer<ImagePtr>::DEFAULT_BUFF_SIZE) :
            Node(id, DESCRIPTION),
            TimedDataGrabberProducer<pcl::io::Image::Ptr>(grabber, id, DESCRIPTION, buffer_size) {
    }
};

#endif //PCL_CLOUD_REGISTRATION_TIMEDIMAGEGRABBERPRODUCER_HPP

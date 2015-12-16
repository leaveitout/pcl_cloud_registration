//
// Created by sean on 07/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_GUARDEDGRABBER_HPP
#define PCL_CLOUD_REGISTRATION_GUARDEDGRABBER_HPP

#include <pcl/io/grabber.h>
#include <mutex>

class GuardedGrabber {
public:
    GuardedGrabber(std::shared_ptr<pcl::Grabber> grabber_) : grabber_(grabber_) {
        mutex_ = std::make_shared<std::mutex>();
    }

    std::shared_ptr<pcl::Grabber> getGrabber() const {
        return grabber_;
    }

    std::shared_ptr<std::mutex> getMutex() const {
        return mutex_;
    }

private:
    std::shared_ptr<pcl::Grabber> grabber_;
    std::shared_ptr<std::mutex> mutex_;
};

#endif //PCL_CLOUD_REGISTRATION_GUARDEDGRABBER_HPP

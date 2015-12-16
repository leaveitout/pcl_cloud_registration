//
// Created by sean on 06/12/15.
//

#include "util/Timer.hpp"

#include <pcl/common/time.h>
#include "util/Logger.hpp"

Timer::Timer(const std::string &description, size_t interval_millis) :
        description_ (description),
        interval_millis_ (interval_millis) {
    last_ = pcl::getTime();
}

void Timer::reset() {
    last_ = pcl::getTime();
}

bool Timer::time() {
    double now = pcl::getTime();
    ++count_;
    if (now - last_ > static_cast<double>(interval_millis_)/1000) {
        std::stringstream ss;
        ss << "Average framerate (" << description_ << "): " << double(count_)/(now - last_) << " Hz." << std::endl;
        Logger::log(Logger::INFO, std::move(ss.str()));
        count_ = 0;
        last_ = now;
        return true;
    }
    return false;
}

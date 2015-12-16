//
// Created by sean on 06/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_TIMER_HPP
#define PCL_CLOUD_REGISTRATION_TIMER_HPP

#include <string>

// TODO: pcl::EventFrequency may be a better choice, investigate
class Timer {
    static constexpr size_t DEFAULT_INTERVAL_MILLIS = 1000;

public:
    Timer(const std::string& description, size_t interval_millis = DEFAULT_INTERVAL_MILLIS);

    void reset();

    bool time();

private:
    double last_;
    std::string description_;
    int count_;
    size_t interval_millis_;
};


#endif //PCL_CLOUD_REGISTRATION_TIMER_HPP

//
// Created by sean on 06/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_TIME_HPP
#define PCL_CLOUD_REGISTRATION_TIME_HPP

#include <stddef.h>

class Timing {

protected:
    Timing(size_t timestamp) : stamp_ (timestamp) { }

public:
    size_t getStamp() const {
        return stamp_;
    }

    virtual ~Timing() {}

private:
    size_t stamp_;
};

#endif //PCL_CLOUD_REGISTRATION_TIMEDDATA_HPP

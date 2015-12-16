//
// Created by sean on 08/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_TIMEDDATA_HPP
#define PCL_CLOUD_REGISTRATION_TIMEDDATA_HPP

#include "Timing.hpp"

template <typename DataType>
class TimedData : public Timing {
public:
    TimedData(DataType data, size_t timestamp): Timing(timestamp), data_(data) { }

    DataType getData() const {
        return data_;
    }

    void setData(DataType data) {
        data_ = data;
    }

private:
    DataType data_;
};

#endif //PCL_CLOUD_REGISTRATION_TIMEDDATA_HPP

//
// Created by sean on 09/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_PALETTE_HPP
#define PCL_CLOUD_REGISTRATION_PALETTE_HPP

#include <algorithm>
#include <random>

#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

class Palette {
private:
    static constexpr int DEFAULT_NUM_COLORS = 10;
    static constexpr int DEFAULT_HUE = 0;
    static constexpr int DEFAULT_SATURATION = 255/2;
    static constexpr int DEFAULT_VALUE = 255;
    static constexpr int MAX_HUE = 180;

public:
    Palette(int num_colors = DEFAULT_NUM_COLORS);

    inline cv::Scalar getColorAt(size_t i) {
        return colors_.at(i % colors_.size());
    }

private:
    std::vector<cv::Scalar> colors_;
};


#endif //PCL_CLOUD_REGISTRATION_PALETTE_HPP

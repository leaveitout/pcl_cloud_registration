//
// Created by sean on 21/01/16.
//

#ifndef PCL_CLOUD_REGISTRATION_CAMERAEXTRINSICSLOADER_HPP
#define PCL_CLOUD_REGISTRATION_CAMERAEXTRINSICSLOADER_HPP

#include <cstddef>
#include <Eigen/Dense>

namespace CameraExtrinsicsLoader {
    bool getExtrinsics(size_t camera_num_1,
                         size_t camera_num_2,
                         Eigen::Matrix4f &RT);
};

#endif //PCL_CLOUD_REGISTRATION_CAMERAEXTRINSICSLOADER_HPP

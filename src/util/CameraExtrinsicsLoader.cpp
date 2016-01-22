//
// Created by sean on 21/01/16.
//

#include "util/CameraExtrinsicsLoader.hpp"
#include <opencv2/core/persistence.hpp>
#include <sstream>
#include <util/Logger.hpp>

bool CameraExtrinsicsLoader::getExtrinsics(size_t camera_num_1,
                                             size_t camera_num_2,
                                             Eigen::Matrix4f &RT) {
    std::string path = "/home/sean/Documents/cameraparams/";
    std::stringstream ss;
    ss << path << camera_num_1 << camera_num_2 << "stereo.yml";

    cv::FileStorage fs_params;
    if(!fs_params.open(ss.str().c_str(), cv::FileStorage::READ) )
        return false;

    cv::Mat R = cv::Mat_<double>::zeros(3, 3);
    cv::Mat T = cv::Mat_<double>::zeros(3, 1);

    fs_params["R"] >> R;
    fs_params["T"] >> T;

    for(int row=0; row<3; ++row)
        for(int col=0; col<3; ++col)
            RT(row, col) = static_cast<float>(R.at<double>(row, col));

    for(int row = 0; row<3; ++row)
        RT(row, 3) = static_cast<float>(T.at<double>(row, 0));

    RT(3, 0) = 0.0;
    RT(3, 1) = 0.0;
    RT(3, 2) = 0.0;
    RT(3, 3) = 1.0;

    std::stringstream ss1;
    ss1 << "Loaded RT (" << camera_num_1 << ", " << camera_num_2 << ") :\n" << RT << std::endl;

    Logger::log(Logger::INFO, ss1.str());

    return true;
}

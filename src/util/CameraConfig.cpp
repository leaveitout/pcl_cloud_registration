//
// Created by sean on 11/12/15.
//

#include "util/CameraConfig.hpp"
#include <string>
#include <vector>
#include <mutex>
#include <util/CameraId.hpp>

namespace CameraConfig {

    namespace {

        std::vector<std::string> cameras;

        // TODO: Put this in an xml or json file
        void initializeMap() {
            cameras.push_back(CameraId::center);
            cameras.push_back(CameraId::right);
            cameras.push_back(CameraId::left);
        }
    }

    std::once_flag flag;

    std::string getCameraIdAt(size_t index) {
        std::call_once(flag, initializeMap);
        return cameras.at(index);
    }

}
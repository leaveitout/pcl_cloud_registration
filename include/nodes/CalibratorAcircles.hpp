//
// Created by sean on 15/01/16.
//

#ifndef PCL_CLOUD_REGISTRATION_CALIBRATOR_ACIRCLES_HPP
#define PCL_CLOUD_REGISTRATION_CALIBRATOR_ACIRCLES_HPP

#include <pcl/PointIndices.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/filter.h>
#include "Producer.hpp"
#include "Processor.hpp"
#include "types/TimedCloudAndImagePair.hpp"
#include "util/SquareDetector.hpp"
#include "util/Palette.hpp"
#include "util/CameraConfig.hpp"

template<typename PointType>
class Calibrator :
        public Processor
                <std::shared_ptr<std::vector<std::shared_ptr<TimedCloudAndImagePair<PointType>>>>>,
        public Producer
                <std::shared_ptr<std::vector<std::shared_ptr<TimedCloudAndImagePair<PointType>>>>> {

private:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    typedef std::shared_ptr<TimedCloudAndImagePair<PointType>> TimedCloudAndImagePairPtr;
    typedef std::vector<TimedCloudAndImagePairPtr> TimedCloudAndImagePairPtrVector;
    typedef std::shared_ptr<TimedCloudAndImagePairPtrVector> TimedCloudAndImagePairPtrVectorPtr;

    static constexpr const int NUM_ITERATIONS_DEFAULT = 15;
    static constexpr const size_t INTERVAL_MILLIS_DEFAULT = 3000;

public:
    Calibrator(size_t id,
               const std::string &description,
               std::shared_ptr<Buffer<TimedCloudAndImagePairPtrVectorPtr>> buffer,
               int num_iterations = NUM_ITERATIONS_DEFAULT,
               size_t interval_millis = INTERVAL_MILLIS_DEFAULT) :
            Node(id, description),
            Processor<TimedCloudAndImagePairPtrVectorPtr>(id, description, buffer),
            Producer<TimedCloudAndImagePairPtrVectorPtr>(id, description) {
        // TODO: Hard-coded number to be removed
        num_iterations_to_do_ = num_iterations;
        interval_millis_ = interval_millis;
    }

protected:
    virtual void processBufferElement(TimedCloudAndImagePairPtrVectorPtr element) {
        std::call_once(flag, [&]() {
            num_clouds_in_element = element->size();
            transformations_ =
                    std::make_shared<std::vector<std::shared_ptr<Eigen::Matrix4f>>>
                            (num_clouds_in_element - 1);
            num_iterations_done_ = std::make_shared<std::vector<int>>(num_clouds_in_element - 1);
            for (auto &num : *num_iterations_done_)
                num = 0;
        });

        if(element) {

        }
    }

private:
    size_t num_clouds_in_element;
    std::shared_ptr<std::vector<std::shared_ptr<Eigen::Matrix4f>>> transformations_;
    std::shared_ptr<std::vector<int>> num_iterations_done_;
    int num_iterations_to_do_;
    size_t interval_millis_;
    size_t prev_timestamp_;
    std::once_flag flag;
    std::shared_ptr<Palette> palette_;

}

#endif //PCL_CLOUD_REGISTRATION_CALIBRATOR_ACIRCLES_HPP

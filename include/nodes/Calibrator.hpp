//
// Created by sean on 09/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_CALIBRATOR_HPP
#define PCL_CLOUD_REGISTRATION_CALIBRATOR_HPP

#include <pcl/PointIndices.h>
#include <util/SquareDetector.hpp>
#include <util/CameraId.hpp>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include "Producer.hpp"
#include "Processor.hpp"
#include "types/TimedCloudAndImagePair.hpp"
#include "util/Palette.hpp"
#include "util/CameraConfig.hpp"

template <typename PointType>
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

public:
    Calibrator(size_t id,
               const std::string &description,
               std::shared_ptr<Buffer<TimedCloudAndImagePairPtrVectorPtr>> buffer) :
            Node(id, description),
            Processor<TimedCloudAndImagePairPtrVectorPtr>(id, description, buffer),
            Producer<TimedCloudAndImagePairPtrVectorPtr>(id, description) {
        palette_ = std::make_shared<Palette>(16);
    }

protected:
    virtual void processBufferElement(TimedCloudAndImagePairPtrVectorPtr element) {
        std::call_once(flag, [&](){
            num_clouds_in_element = element->size();
            transformations_ =
                    std::make_shared<std::vector<std::shared_ptr<Eigen::Matrix4f>>>
                            (num_clouds_in_element-1);
        });


        // Check that we have a full element
        for(size_t idx = 0; idx < num_clouds_in_element; ++idx) {
            if(!element->at(idx)->getTimedImage() || !element->at(idx)->getTimedCloud()) {
                this->getBuffer()->pushBack(element);
                return;
            }
        }

        std::vector<pcl::PointIndices::Ptr> point_indices(num_clouds_in_element);
        for(size_t idx = 0; idx < num_clouds_in_element; ++idx) {
            auto image = element->at(idx)->getTimedImage()->getData();
            point_indices.at(idx) =
                    SquareDetector::getPointIndicesOfCorners(image,
                                                             CameraConfig::getCameraIdAt(idx),
                                                             palette_);
//            Logger::log(Logger::INFO, "Found %d corners\n", point_indices.at(idx)->indices.size());
        }

        bool use_svd = true;

        auto target = element->at(0)->getTimedCloud()->getData();
        auto target_indices = point_indices.at(0);
        if(target_indices)
            if(target_indices->indices.size() > 0)
                for(size_t cloud_num = 1; cloud_num < num_clouds_in_element; ++cloud_num) {
                    auto source = element->at(cloud_num)->getTimedCloud()->getData();
                    auto source_indices = point_indices.at(cloud_num);
                    if(source_indices)
                        if(source_indices->indices.size() == target_indices->indices.size()) {
                            std::shared_ptr<Eigen::Matrix4f> transformation =
                                    std::make_shared<Eigen::Matrix4f>();
                            if(use_svd) {
                                pcl::registration::TransformationEstimationSVD<PointType, PointType>
                                        svd;
                                svd.estimateRigidTransformation(
                                        *source,
                                        source_indices->indices,
                                        *target,
                                        target_indices->indices,
                                        *transformation);
                            }
                            else {
                                pcl::registration::TransformationEstimationLM<PointType, PointType>
                                        svd;
                                svd.estimateRigidTransformation(
                                                                *target,
                                                                target_indices->indices,
                                                                *source,
                                                                source_indices->indices,
                                                                *transformation);

                            }
                            std::stringstream ss;
                            ss << "Transformation Matrix (" << cloud_num << "): " << std::endl <<
                                    *transformation << std::endl;
                            Logger::log(Logger::INFO, ss.str());
                            if(!transformation->hasNaN() && !transformation->isIdentity()) {
                                transformations_->at(cloud_num - 1) = transformation;
                            }
                        }
                }


        for(size_t cloud_num = 1; cloud_num < num_clouds_in_element; ++cloud_num) {
            auto current_transformation = transformations_->at(cloud_num -1);
            if(current_transformation)
                if(!current_transformation->hasNaN() &&
                   !current_transformation->isIdentity()) {
                    auto source = element->at(cloud_num)->getTimedCloud()->getData();
                    auto transformed_cloud = boost::make_shared<Cloud>();
                    pcl::transformPointCloud(*source,
                                             *transformed_cloud,
                                             *current_transformation);
                    Logger::log(Logger::INFO, "Transformed cloud!\n");
                    auto target_result = boost::make_shared<Cloud>();
                    *target_result += *target;
                    *target_result += *transformed_cloud;
                    CloudConstPtr const_target_result = target_result;

                    element->at(0)->getTimedCloud()->setData(const_target_result);

                }
        }

        this->getBuffer()->pushBack(element);
    }

private:
    size_t num_clouds_in_element;
    std::shared_ptr<std::vector<std::shared_ptr<Eigen::Matrix4f>>> transformations_;
    std::once_flag flag;
    std::shared_ptr<Palette> palette_;
};

#endif //PCL_CLOUD_REGISTRATION_CALIBRATOR_HPP


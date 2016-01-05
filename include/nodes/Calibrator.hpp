//
// Created by sean on 09/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_CALIBRATOR_HPP
#define PCL_CLOUD_REGISTRATION_CALIBRATOR_HPP

#include <pcl/PointIndices.h>
#include <util/CameraId.hpp>
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

    static constexpr const int NUM_ITERATIONS_DEFAULT = 1;

public:
    Calibrator(size_t id,
               const std::string &description,
               std::shared_ptr<Buffer<TimedCloudAndImagePairPtrVectorPtr>> buffer,
               int num_iterations = NUM_ITERATIONS_DEFAULT) :
            Node(id, description),
            Processor<TimedCloudAndImagePairPtrVectorPtr>(id, description, buffer),
            Producer<TimedCloudAndImagePairPtrVectorPtr>(id, description) {
        // TODO: Hard-coded number to be removed
        palette_ = std::make_shared<Palette>(16);
        num_iterations_to_do_ = num_iterations;
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

        if (element) {
            std::vector<pcl::PointIndices::Ptr> point_indices(num_clouds_in_element);
            // Need target cloud and image to find translation
            if (element->at(0)->getTimedImage() && element->at(0)->getTimedCloud()) {
                for (size_t idx = 1; idx < num_clouds_in_element; ++idx) {
                    if (num_iterations_done_->at(idx - 1) < num_iterations_to_do_) {
                        // Need target cloud and image
                        if (element->at(idx)->getTimedImage() &&
                            element->at(idx)->getTimedCloud()) {
                            // Get indices for target cloud
                            if (!point_indices.at(0)) {
                                auto image = element->at(0)->getTimedImage()->getData();
                                point_indices.at(0) = SquareDetector::getPointIndicesOfCorners(
                                        image,
                                        CameraConfig::getCameraIdAt(0),
                                        palette_);
                            }

                            // Only continue if we found correct indices
                            // TODO: Change the indices size check to be the number of indices we need
                            auto target_indices = point_indices.at(0);
                            auto target = element->at(0)->getTimedCloud()->getData();

                            if (target_indices) {
                                if (target_indices->indices.size() > 0) {
                                    // Get indices for source cloud
                                    auto image = element->at(idx)->getTimedImage()->getData();
                                    point_indices.at(idx) =
                                            SquareDetector::getPointIndicesOfCorners(
                                                    image,
                                                    CameraConfig::getCameraIdAt(idx),
                                                    palette_);

                                    auto source_indices = point_indices.at(idx);
                                    auto source = element->at(idx)->getTimedCloud()->getData();
                                    std::shared_ptr<Eigen::Matrix4f> transformation =
                                            std::make_shared<Eigen::Matrix4f>();

                                    if (source_indices) {
                                        if (source_indices->indices.size() ==
                                            target_indices->indices.size()) {
                                            pcl::registration::TransformationEstimationSVD<PointType,
                                                    PointType> svd;
                                            svd.estimateRigidTransformation(*source,
                                                                            source_indices->indices,
                                                                            *target,
                                                                            target_indices->indices,
                                                                            *transformation);
                                            std::stringstream ss;
                                            ss << "Transformation Matrix (" << idx << "): " <<
                                            std::endl << *transformation << std::endl;
                                            Logger::log(Logger::INFO, ss.str());
                                        }
                                    }

                                    if (!transformation->hasNaN() &&
                                            !transformation->isIdentity()) {
                                        auto transformed_cloud = boost::make_shared<Cloud>();
                                        pcl::transformPointCloud(*source,
                                                                 *transformed_cloud,
                                                                 *transformation);
                                        // Remove all NaNs from clouds
                                        std::vector<int> mapping;
                                        pcl::removeNaNFromPointCloud(*transformed_cloud,
                                                                     *transformed_cloud,
                                                                     mapping);

                                        auto target_clean = boost::make_shared<Cloud>();
                                        pcl::removeNaNFromPointCloud(*target,
                                                                     *target_clean,
                                                                     mapping);

                                        pcl::IterativeClosestPoint<PointType, PointType> icp;
                                        icp.setInputSource(transformed_cloud);
                                        icp.setInputTarget(target_clean);
                                        auto target_result = boost::make_shared<Cloud>();
//                                        icp.align(*target_result);
//                                        std::stringstream ss;
//                                        ss << "Has converged: " << icp.hasConverged() <<
//                                                " score: " << icp.getFitnessScore() << std::endl;
//                                        ss << icp.getFinalTransformation() << std::endl;
//                                        Logger::log(Logger::INFO, ss.str());


                                        std::shared_ptr<Eigen::Matrix4f> final_transformation =
                                                std::make_shared<Eigen::Matrix4f>();
//                                        *final_transformation = *transformation;
//                                        final_transformation.
//                                        *final_transformation = icp.getFinalTransformation() *
//                                                *transformation;
                                        *final_transformation = *transformation;

                                        transformations_->at(idx - 1) = final_transformation;
                                        num_iterations_done_->at(idx - 1)++;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // Check if we need to transform clouds
            bool to_transform = false;
            for (auto tr : *transformations_)
                if (tr)
                    to_transform = true;

            if (to_transform) {
                if (element->at(0)->getTimedCloud()) {
                    auto target_result = boost::make_shared<Cloud>();
                    *target_result += *element->at(0)->getTimedCloud()->getData();
                    for (size_t idx = 1; idx < num_clouds_in_element; ++idx) {
                        if (element->at(idx)->getTimedCloud() && transformations_->at(idx - 1)) {
                            auto transformed_cloud = boost::make_shared<Cloud>();
                            pcl::transformPointCloud(*element->at(idx)->getTimedCloud()->getData(),
                                                     *transformed_cloud,
                                                     *transformations_->at(idx - 1));
                            *target_result += *transformed_cloud;
                        }
                    }

                    CloudConstPtr const_target_result = target_result;
                    element->at(0)->getTimedCloud()->setData(const_target_result);
                }

            }
        }

        this->getBuffer()->pushBack(element);

//        if (element->at(idx)->getTimedCloud() &&
//            num_iterations_done_->at(idx - 1) > 0 &&
//            transformations_->(idx - 1)) {

    }

//        // Check that we have a full element
//        for(size_t idx = 0; idx < num_clouds_in_element; ++idx) {
//            if(!element->at(idx)->getTimedImage() || !element->at(idx)->getTimedCloud()) {
//                this->getBuffer()->pushBack(element);
//                return;
//            }
//        }
//
//        std::vector<pcl::PointIndices::Ptr> point_indices(num_clouds_in_element);
//        for(size_t idx = 0; idx < num_clouds_in_element; ++idx) {
//            auto image = element->at(idx)->getTimedImage()->getData();
//            point_indices.at(idx) =
//                    SquareDetector::getPointIndicesOfCorners(image,
//                                                             CameraConfig::getCameraIdAt(idx),
//                                                             palette_);
//        }
//
//        bool use_svd = true;
//
//        auto target = element->at(0)->getTimedCloud()->getData();
//        auto target_indices = point_indices.at(0);
//        if (target_indices) if (target_indices->indices.size() > 0)
//            for (size_t cloud_num = 1; cloud_num < num_clouds_in_element; ++cloud_num) {
//                auto source = element->at(cloud_num)->getTimedCloud()->getData();
//                auto source_indices = point_indices.at(cloud_num);
//                if (source_indices) if (source_indices->indices.size() ==
//                                        target_indices->indices.size()) {
//                    std::shared_ptr<Eigen::Matrix4f> transformation =
//                            std::make_shared<Eigen::Matrix4f>();
//                    if (use_svd) {
//                        pcl::registration::TransformationEstimationSVD<PointType, PointType>
//                                svd;
//                        svd.estimateRigidTransformation(
//                                *source,
//                                source_indices->indices,
//                                *target,
//                                target_indices->indices,
//                                *transformation);
//                    }
//                    else {
//                        pcl::registration::TransformationEstimationLM<PointType, PointType>
//                                svd;
//                        svd.estimateRigidTransformation(
//                                *target,
//                                target_indices->indices,
//                                *source,
//                                source_indices->indices,
//                                *transformation);
//
//                    }
//                    std::stringstream ss;
//                    ss << "Transformation Matrix (" << cloud_num << "): " << std::endl <<
//                    *transformation << std::endl;
//                    Logger::log(Logger::INFO, ss.str());
//                    if (!transformation->hasNaN() && !transformation->isIdentity()) {
//                        transformations_->at(cloud_num - 1) = transformation;
//                    }
//                }
//            }
//
//        for (size_t cloud_num = 1; cloud_num < num_clouds_in_element; ++cloud_num) {
//            auto current_transformation = transformations_->at(cloud_num - 1);
//            if (current_transformation) if (!current_transformation->hasNaN() &&
//                                            !current_transformation->isIdentity()) {
//                auto source = element->at(cloud_num)->getTimedCloud()->getData();
//                auto transformed_cloud = boost::make_shared<Cloud>();
//                pcl::transformPointCloud(*source,
//                                         *transformed_cloud,
//                                         *current_transformation);
////                    pcl::IterativeClosestPoint<PointType, PointType> icp;
//                // Remove all NaNs from clouds
//                std::vector<int> mapping;
//                pcl::removeNaNFromPointCloud(*transformed_cloud, *transformed_cloud, mapping);
//                auto target_clean = boost::make_shared<Cloud>();
//                pcl::removeNaNFromPointCloud(*target, *target_clean, mapping);
//                pcl::IterativeClosestPoint<PointType, PointType> icp;
//                icp.setInputSource(transformed_cloud);
//                icp.setInputTarget(target_clean);
//                auto target_result = boost::make_shared<Cloud>();
//                icp.align(*target_result);
//                std::stringstream ss;
//                ss << "Has converged: " << icp.hasConverged() << " score: " <<
//                icp.getFitnessScore() << std::endl;
//                ss << icp.getFinalTransformation() << std::endl;
//                Logger::log(Logger::INFO, ss.str());
//
////                    Logger::log(Logger::INFO, "Transformed cloud!\n");
////                    auto target_result = boost::make_shared<Cloud>();
////                    *target_result += *target;
////                    *target_result += *transformed_cloud;
//                CloudConstPtr const_target_result = target_result;
//
//                element->at(0)->getTimedCloud()->setData(const_target_result);
//
//            }
//        }
//
//        this->getBuffer()->pushBack(element);
//    }

    bool getRigidTransform() {
        return false;
    }

    bool checkValidElement() {
        return false;
    }

private:
    size_t num_clouds_in_element;
    std::shared_ptr<std::vector<std::shared_ptr<Eigen::Matrix4f>>> transformations_;
    std::shared_ptr<std::vector<int>> num_iterations_done_;
    int num_iterations_to_do_;
    std::once_flag flag;
    std::shared_ptr<Palette> palette_;
};

#endif //PCL_CLOUD_REGISTRATION_CALIBRATOR_HPP


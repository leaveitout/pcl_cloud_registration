//
// Created by sean on 09/12/15.
//
#ifndef PCL_CLOUD_REGISTRATION_MULTICLOUDANDIMAGEPAIRVIEWER_HPP
#define PCL_CLOUD_REGISTRATION_MULTICLOUDANDIMAGEPAIRVIEWER_HPP

#include "nodes/Viewer.hpp"
#include "types/TimedCloudAndImagePair.hpp"

template <typename PointType>
class MultiCloudAndImagePairViewer :
        public Viewer<std::shared_ptr<std::vector<std::shared_ptr<TimedCloudAndImagePair<PointType>>>>> {
    using TimedCloudAndImagePairPtr = std::shared_ptr<TimedCloudAndImagePair<PointType>>;
    using TimedCloudAndImagePairPtrVector = std::vector<TimedCloudAndImagePairPtr>;
    using TimedCloudAndImagePairPtrVectorPtr = std::shared_ptr<TimedCloudAndImagePairPtrVector>;

public:
    MultiCloudAndImagePairViewer(size_t id, const std::string &description,
                                 std::shared_ptr<Buffer<TimedCloudAndImagePairPtrVectorPtr>> buffer) :
            Node(id, description),
            Viewer<TimedCloudAndImagePairPtrVectorPtr>(id, description, buffer) {
    }

protected:
    virtual void processBufferElement(TimedCloudAndImagePairPtrVectorPtr element) {
        std::call_once(flag, [&]() {
            num_clouds_in_element = element->size();
            for (int i = 0; i < num_clouds_in_element; ++i) {
                pcl::visualization::PCLVisualizer::Ptr viz = boost::make_shared<pcl::visualization::PCLVisualizer>();
                cloud_viewers_.push_back(viz);
                this->registerKeyboardAndMouseCallbacks(viz);
                viz->setCameraFieldOfView (1.02259994f);

                pcl::visualization::ImageViewer::Ptr img_viz = boost::make_shared<pcl::visualization::ImageViewer>();
                image_viewers_.push_back(img_viz);
                this->registerKeyboardAndMouseCallbacks(viz);
//                img_viz->registerMouseCallback(&MultiCloudAndImagePairViewer::mouseCallback, *this);
//                img_viz->registerKeyboardCallback(&MultiCloudAndImagePairViewer::keyboardCallback, *this);
            }
        });

        for (size_t idx = 0; idx < num_clouds_in_element; ++idx) {
            auto viz = cloud_viewers_.at(idx);
            auto img_viz = image_viewers_.at(idx);
            viz->spinOnce();
            auto cloud_image_pair = element->at(idx);
            if (cloud_image_pair) {
                auto timed_cloud = cloud_image_pair->getTimedCloud();
                auto timed_image = cloud_image_pair->getTimedImage();
                if (timed_cloud) {
                    auto cloud = timed_cloud->getData();
                    if (!viz->updatePointCloud(cloud, "OpenNICloud")) {
                        viz->setPosition(idx * 800, 0);
                        if(timed_image)
                            img_viz->setPosition(idx * 800, 800);
                        viz->setSize(640, 480);
                        viz->addPointCloud(cloud, "OpenNICloud");
                        viz->resetCameraViewpoint("OpenNICloud");
                        viz->setCameraPosition(
                                0, 0, 0,        // Position
                                0, 0, 1,        // Viewpoint
                                0, -1, 0);    // Up
                        viz->addText(std::to_string(idx), 0, 0, "Camera Index");
                    }
                }
                if(timed_image) {
                    auto image = timed_image->getData();
                    if(image->getEncoding() == pcl::io::Image::RGB) {
//                        img_viz->setPosition(idx * 640, 600);
                        img_viz->addRGBImage(reinterpret_cast<const unsigned char*>(image->getData()),
                                             image->getWidth(),
                                             image->getHeight());

                    }
                }
            }
            img_viz->spinOnce();
        }
    }

    void mouseCallback(const pcl::visualization::MouseEvent &mouse_event, void *) {
        if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress &&
            mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton) {
            Logger::log(Logger::INFO, "Left button pressed @ (%i, %i).\n", mouse_event.getX(), mouse_event.getY());
        }
    }

    void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *) {
        std::string special_key = event.getKeyCode() ? "" : "special ";
        if (event.keyDown())
            Logger::log(Logger::INFO, "The %skey %c was pressed.", special_key.c_str(), event.getKeyCode());
        else
            Logger::log(Logger::INFO, "The %skey %c was released.", special_key.c_str(), event.getKeyCode());
    }

private:
    std::vector<pcl::visualization::PCLVisualizer::Ptr> cloud_viewers_;
    std::vector<pcl::visualization::ImageViewer::Ptr> image_viewers_;
    size_t num_clouds_in_element;
    std::once_flag flag;
};
#endif //PCL_CLOUD_REGISTRATION_MULTICLOUDANDIMAGEPAIRVIEWER_HPP


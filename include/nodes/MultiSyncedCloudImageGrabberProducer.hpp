//
// Created by sean on 06/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_MULTISYNCEDCLOUDIMAGEGRABBERPRODUCER_HPP
#define PCL_CLOUD_REGISTRATION_MULTISYNCEDCLOUDIMAGEGRABBERPRODUCER_HPP

#include <pcl/point_cloud.h>
#include <pcl/io/image.h>
#include <pcl/io/grabber.h>
#include <thread>
#include <types/TimedCloud.hpp>
#include <types/TimedImage.hpp>
#include <types/TimedCloudAndImagePair.hpp>
#include "nodes/TimedImageGrabberProducer.hpp"
#include "nodes/TimedCloudGrabberProducer.hpp"
#include "Synchronizer.hpp"
#include "Processor.hpp"

using namespace std;

template<typename PointType>
class MultiSyncedCloudImageGrabberProducer :
        public Producer<shared_ptr<vector<shared_ptr<TimedCloudAndImagePair<PointType>>>>>,
        public Processor<shared_ptr<std::vector<shared_ptr<Timing>>>> {
    typedef typename pcl::PointCloud<PointType>::ConstPtr CloudConstPtr;
    typedef pcl::io::Image::Ptr ImagePtr;
    typedef std::shared_ptr<Timing> TimingPtr;
    typedef std::vector<TimingPtr> TimingPtrVector;
    typedef std::shared_ptr<TimingPtrVector> TimingPtrVectorPtr;

    static constexpr const char *DESCRIPTION = "Multi Synced Cloud Image Grabber";

public:
    MultiSyncedCloudImageGrabberProducer(const vector<shared_ptr<pcl::Grabber>>& grabbers, size_t id) :
            Node(id, DESCRIPTION),
            Producer<shared_ptr<vector<shared_ptr<TimedCloudAndImagePair<PointType>>>>>(id, DESCRIPTION),
            Processor<shared_ptr<vector<shared_ptr<Timing>>>>(id, DESCRIPTION) {
        // Create all of the individual producers
        size_t grabber_id = 1;
        for(auto grabber : grabbers) {
            auto guarded_grabber = make_shared<GuardedGrabber>(grabber);
            auto cloud_producer =
                    make_shared<TimedCloudGrabberProducer<PointType>>(guarded_grabber, 10*id + grabber_id);
            cloud_producers_.push_back(cloud_producer);
            auto image_producer = make_shared<TimedImageGrabberProducer>(guarded_grabber, 10*id + grabber_id);
            image_producers_.push_back(image_producer);
            grabber_id++;
        }

        synchronizer_ = make_shared<Synchronizer>(0);
        for(auto producer : cloud_producers_) {
            synchronizer_->addInputBuffer(producer->getBuffer());
        }

        for(auto producer : image_producers_) {
            synchronizer_->addInputBuffer(producer->getBuffer());
        }

        this->attachToBuffer(synchronizer_->getBuffer());
    }

    virtual void start() {
        vector<thread> threads(cloud_producers_.size() + image_producers_.size());

        for(size_t idx = 0; idx < threads.size(); idx += 2) {
            threads.at(idx) = std::thread( [=] { cloud_producers_.at(idx/2)->start(); });
            threads.at(idx + 1) = std::thread( [=] { image_producers_.at(idx/2)->start(); });
        }

        for(auto& thread : threads)
            thread.join();

        synchronizer_->start();

        Processor<TimingPtrVectorPtr>::start();
    }

    virtual void stop() {
        vector<thread> threads(cloud_producers_.size() + image_producers_.size());

        for(size_t idx = 0; idx < threads.size(); idx += 2) {
            threads.at(idx) = std::thread( [=] { cloud_producers_.at(idx/2)->stop(); });
            threads.at(idx + 1) = std::thread( [=] { image_producers_.at(idx/2)->stop(); });
        }

        for(auto& thread : threads)
            thread.join();

        synchronizer_->stop();

        Processor<TimingPtrVectorPtr>::stop();
    }

    virtual void processBufferElement(TimingPtrVectorPtr element) {
        // Unpack and repack the data
        auto temp_output_element = make_shared<vector<shared_ptr<TimedCloudAndImagePair<PointType>>>>();
        // Order is cloud1, cloud2, ..., cloudN, image1, image2, ..., imageN
        size_t num_clouds = element->size()/2;
        for(size_t idx = 0; idx < num_clouds; ++idx) {
            auto timed_cloud_ptr = static_pointer_cast<TimedCloud<PointType>>(element->at(idx));
            auto timed_image_ptr = static_pointer_cast<TimedImage>(element->at(idx + num_clouds));
            auto timed_pair = make_shared<TimedCloudAndImagePair<PointType>>(timed_cloud_ptr, timed_image_ptr);
//            stringstream ss;
//            if(timed_cloud_ptr)
//                ss << "Timed Cloud " << timed_cloud_ptr->getStamp() << timed_cloud_ptr->getData()->header << endl;
//            if(timed_image_ptr)
//                ss << "Timed Image " << timed_image_ptr->getStamp();
//            Logger::log(Logger::INFO, ss.str());
            temp_output_element->push_back(timed_pair);
        }
        this->getBuffer()->pushBack(temp_output_element);
    }


private:
    vector<shared_ptr<TimedCloudGrabberProducer<PointType>>>    cloud_producers_;
    vector<shared_ptr<TimedImageGrabberProducer>>               image_producers_;
    std::shared_ptr<Synchronizer>                               synchronizer_;
};

#endif //PCL_CLOUD_REGISTRATION_MULTISYNCEDCLOUDIMAGEGRABBERPRODUCER_HPP


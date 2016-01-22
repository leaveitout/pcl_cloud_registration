//
// Created by sean on 06/12/15.
//

#include "apps/LiveVisualizerApp.hpp"

#include <pcl/io/openni2_grabber.h>
#include <thread>
#include <nodes/MultiSyncedCloudImageGrabberProducer.hpp>
#include <nodes/MultiCloudAndImagePairViewer.hpp>
#include <nodes/Calibrator.hpp>
#include <util/CameraIntrinsicsLoader.hpp>

LiveVisualizerApp::LiveVisualizerApp(int argc, char **argv) : App(argc, argv) {

}

int LiveVisualizerApp::exec() {
    typedef pcl::io::OpenNI2Grabber NI2Grabber;
    typedef pcl::io::openni2::OpenNI2DeviceManager NI2DeviceManager;

    auto device_manager = NI2DeviceManager::getInstance();
    if (device_manager->getNumOfConnectedDevices() > 0) {
        std::vector<std::shared_ptr<pcl::Grabber>> grabbers;

        size_t num_devices = device_manager->getNumOfConnectedDevices();
        for (size_t i = 1; i <= num_devices; ++i) {
            std::stringstream ss;
            ss << '#' << i;
            auto grabber = std::make_shared<NI2Grabber>(ss.str());
            if (grabber->isRunning())
                Logger::log(Logger::INFO, "Grabber is running\n");

            CameraIntrinsicsLoader::applyIntrinsics(i, grabber);
            grabbers.push_back(grabber);
        }
        MultiSyncedCloudImageGrabberProducer<pcl::PointXYZRGBA> producer(grabbers, 1);
        Calibrator<pcl::PointXYZRGBA> calibrator(1, "Calibrator", producer.getBuffer());
        MultiCloudAndImagePairViewer<pcl::PointXYZRGBA> viewer(1, "Viewers", calibrator.getBuffer());

        producer.start();
        calibrator.start();
        viewer.start();
        std::this_thread::sleep_for(std::chrono::seconds(10));
        producer.stop();
        calibrator.stop();
        viewer.stop();
    }
    else {
        Logger::log(Logger::INFO, "No compatible devices attached.\n");
    }
    return 0;
}

void LiveVisualizerApp::parseArguments() {
    App::parseArguments();
}

void LiveVisualizerApp::printHelp() {
    App::printHelp();
}



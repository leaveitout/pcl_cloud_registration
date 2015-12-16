//
// Created by sean on 09/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_VIEWER_HPP
#define PCL_CLOUD_REGISTRATION_VIEWER_HPP

#include "Processor.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>

template <typename Type>
class Viewer : public Processor<Type> {

public:
    Viewer(size_t id, const std::string& description, std::shared_ptr<Buffer<Type>> buffer) :
            Node(id, description),
            Processor<Type>(id, description, buffer) {
    }

    void registerMouseCallback(pcl::visualization::PCLVisualizer::Ptr visualizer) {
        visualizer->registerMouseCallback(&Viewer::mouseCallback, *this);
    }

    void registerKeyboardCallback(pcl::visualization::PCLVisualizer::Ptr visualizer) {
        visualizer->registerKeyboardCallback(&Viewer::keyboardCallback, *this);
    }

    void registerKeyboardAndMouseCallbacks(pcl::visualization::PCLVisualizer::Ptr visualizer) {
        visualizer->registerKeyboardCallback(&Viewer::keyboardCallback, *this);
        visualizer->registerMouseCallback(&Viewer::mouseCallback, *this);
    }

protected:
    void mouseCallback(const pcl::visualization::MouseEvent &mouse_event, void *) {
        if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress &&
            mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton) {
            std::stringstream ss;
            ss << "Left button pressed @ (" << mouse_event.getX() << ", " <<
                    mouse_event.getY() << ")." << std::endl;
            Logger::log(Logger::INFO, ss.str());
        }
    }

    void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *) {
        std::string special_key = event.getKeyCode() ? "" : "special ";
        std::stringstream ss;
        std::string action = event.keyDown() ? "pressed" : "released";
        ss << "The " << special_key.c_str() << " key " << event.getKeyCode() <<
                " was " << action << "." << endl;
        Logger::log(Logger::INFO, ss.str());
    }
};

#endif //PCL_CLOUD_REGISTRATION_VIEWER_HPP

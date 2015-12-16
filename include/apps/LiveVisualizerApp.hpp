//
// Created by sean on 06/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_LIVEVISUALIZERAPP_HPP
#define PCL_CLOUD_REGISTRATION_LIVEVISUALIZERAPP_HPP

#include "App.hpp"

class LiveVisualizerApp : public App {

public:
    LiveVisualizerApp(int argc, char **argv);

    virtual int exec() override;

private:
    virtual void parseArguments() override;

    virtual void printHelp() override;
};

#endif //PCL_CLOUD_REGISTRATION_LIVEVISUALIZERAPP_HPP

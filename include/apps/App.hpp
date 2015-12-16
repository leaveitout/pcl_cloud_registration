//
// Created by sean on 06/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_APP_HPP
#define PCL_CLOUD_REGISTRATION_APP_HPP

class App {
public:
    App(int argc, char **argv);

    virtual int exec() = 0;

protected:
    virtual void parseArguments();

    virtual void printHelp();
};

#endif //PCL_CLOUD_REGISTRATION_APP_HPP

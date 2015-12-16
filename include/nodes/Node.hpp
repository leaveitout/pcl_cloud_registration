//
// Created by sean on 06/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_NODE_HPP
#define PCL_CLOUD_REGISTRATION_NODE_HPP

#include <string>

class Node {
public:
    Node(size_t id, const std::string& description);

    virtual ~Node(){};

    virtual void start() = 0;

    virtual void stop() = 0;

    const std::string& getDescription() const;

    size_t getId() const;

private:
    size_t id_;
    std::string description_;
};

#endif //PCL_CLOUD_REGISTRATION_NODE_HPP

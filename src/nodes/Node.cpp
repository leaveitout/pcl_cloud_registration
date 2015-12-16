//
// Created by sean on 06/12/15.
//

#include "nodes/Node.hpp"

Node::Node(size_t id, const std::string& description) :
        id_ (id),
        description_ (description) {
}

const std::string& Node::getDescription() const {
    return description_;
}

size_t Node::getId() const {
    return id_;
}

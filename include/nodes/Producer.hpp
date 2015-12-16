//
// Created by sean on 06/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_PRODUCER_HPP
#define PCL_CLOUD_REGISTRATION_PRODUCER_HPP

#include "nodes/Node.hpp"
#include "Buffer.hpp"

template <typename Type>
class Producer : public virtual Node {
public:
    Producer(size_t id, const std::string &description, size_t buffer_size = Buffer<Type>::DEFAULT_BUFF_SIZE) :
            Node(id, description) {
        buffer_ = std::make_shared<Buffer<Type>>(buffer_size);
    }

    virtual ~Producer(){};

    std::shared_ptr<Buffer<Type>> getBuffer() const {
        return buffer_;
    }

private:
    std::shared_ptr<Buffer<Type>> buffer_;
};

#endif //PCL_CLOUD_REGISTRATION_PRODUCER_HPP

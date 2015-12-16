//
// Created by sean on 07/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_PROCESSOR_HPP
#define PCL_CLOUD_REGISTRATION_PROCESSOR_HPP

#include <memory>
#include <thread>
#include <sstream>

#include "util/Timer.hpp"
#include "Buffer.hpp"
#include "nodes/Node.hpp"
#include "util/Logger.hpp"

template <typename Type>
class Processor : public virtual Node {
public:
    Processor(size_t id, const std::string& description, const std::shared_ptr<Buffer<Type>> buffer) :
            Node(id, description),
            buffer_ (buffer),
            is_done_ (false),
            is_started_ (false),
            is_detached_ (false) {
    }

    Processor(size_t id, std::string description) :
            Processor(id, description, nullptr) {
        is_detached_ = true;
    }

    virtual void start() override {
        if(!buffer_) {
            std::stringstream ss;
            ss << "Invalid buffer for Processor " << id_ << " (" << description_ << "), " <<
            "thread finishing." << std::endl;
            Logger::log(Logger::INFO, ss.str());
            return;
        }
        is_done_ = false;
        is_started_ = true;
        std::stringstream ss;
        ss << "Processor " << id_ << ", " << description_ << "";
        timer_.reset (new Timer(ss.str()));
        thread_.reset (new std::thread(&Processor::run, this));
    }

    virtual void stop () override {
        if(is_started_) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                is_done_ = true;
            }
            thread_->join();
            is_started_ = false;
            std::stringstream ss;
            ss << "Processor " << id_ << ", " << description_ << " done." << std::endl;
            Logger::log(Logger::INFO, ss.str());
        }
    }

    void attachToBuffer(const std::shared_ptr<Buffer<Type>> buffer) {
        detachFromBuffer();
        buffer_ = buffer;
        is_detached_ = false;
    }

    void detachFromBuffer() {
        if (is_started_) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                is_done_ = true;
                is_detached_ = true;
            }
            thread_->join();
            is_started_ = false;
            buffer_ = nullptr;
            std::stringstream ss;
            ss << "Processor " << id_ << ", " << description_ << " detached." << std::endl;
            Logger::log(Logger::INFO, ss.str());
        }
    }

protected:
    virtual void initializeThreadResources() {};

    virtual void processBufferElement(Type element) = 0;

private:
    void run() {
        // TODO: Change this to a call_once inside getElementAndProcess, allowing inspection of first element
        initializeThreadResources();
        mutex_.lock();
        while(!is_done_) {
            mutex_.unlock();
            getElementAndProcess();
            mutex_.lock();
        }
        mutex_.unlock();

        if(!is_detached_) {
            std::stringstream ss;
            ss << "Processing remaining " << buffer_->getSize() << " elements in the buffer..." << std::endl;
            Logger::log(Logger::INFO, ss.str());

            while (!buffer_->isEmpty())
                getElementAndProcess();
        }
    }

    void getElementAndProcess() {
        Type element = buffer_->getFront();
        if(element) {
            processBufferElement(element);
            if (timer_->time()) {
                std::stringstream ss;
                ss << "Processor " << id_ << ", " << description_ <<
                        " buffer size: " << buffer_->getSize() << "." << std::endl;
                Logger::log(Logger::INFO, ss.str());
            }
        }
    }

private:
    std::string description_;
    size_t  id_;
    std::shared_ptr<std::thread> thread_;
    std::shared_ptr<Timer> timer_;
    std::shared_ptr<Buffer<Type>> buffer_;
    bool is_started_;
    bool is_done_;
    bool is_detached_;
    std::mutex mutex_;
};
#endif //PCL_CLOUD_REGISTRATION_PROCESSOR_HPP

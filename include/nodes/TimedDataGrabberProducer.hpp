//
// Created by sean on 06/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_TIMEDDATAGRABBERPRODUCER_HPP
#define PCL_CLOUD_REGISTRATION_TIMEDDATAGRABBERPRODUCER_HPP

#include <pcl/io/grabber.h>
#include <util/Timer.hpp>
#include <util/Logger.hpp>
#include <types/GuardedGrabber.hpp>
#include "nodes/Producer.hpp"
#include "types/TimedData.hpp"

template <typename DataType>
class TimedDataGrabberProducer : public Producer<std::shared_ptr<TimedData<DataType>>> {
    typedef std::shared_ptr<TimedData<DataType>> TimedDataPtr;

protected:
    TimedDataGrabberProducer(std::shared_ptr<GuardedGrabber> guarded_grabber,
                             size_t id,
                             const std::string& description,
                             size_t buffer_size = Buffer<TimedDataPtr>::DEFAULT_BUFF_SIZE) :
            Node(id, description),
            Producer<TimedDataPtr>(id, description, buffer_size),
            guarded_grabber_ (guarded_grabber) {
        std::lock_guard<std::mutex> lock(*guarded_grabber_->getMutex());
        if(guarded_grabber_->getGrabber()->providesCallback<void (const DataType&)>()) {
            boost::function<void (const DataType&)> callback =
                    boost::bind(&TimedDataGrabberProducer::callback, this, _1);
            connection_ = std::make_shared<boost::signals2::connection>(guarded_grabber_->getGrabber()->registerCallback (callback));
        }
        else {
            std::stringstream ss;
            ss << "Callback not provided by Device " << this->getId() << ", " << this->getDescription() << std::endl;
            Logger::log(Logger::ERROR, ss.str());
        }
    }

    virtual ~TimedDataGrabberProducer() {};

public:
    virtual void start() override {
        if(connection_) {
            std::stringstream ss;
            ss << "Device " << this->getId() << ", " << this->getDescription();
            timer_ = std::make_shared<Timer>(ss.str());
            {
                std::lock_guard<std::mutex> lock(*guarded_grabber_->getMutex());
                if (!guarded_grabber_->getGrabber()->isRunning())
                    guarded_grabber_->getGrabber()->start();
            }
            auto ms = std::chrono::duration_cast< std::chrono::milliseconds >(
                    std::chrono::system_clock::now().time_since_epoch() );
            size_t start_time = (size_t) ms.count();
            ss << ", started at " << start_time << "." << std::endl;
            Logger::log(Logger::INFO, ss.str());
        }
    }

    virtual void stop() override {
        if(connection_) {
            {
                std::lock_guard<std::mutex> lock(*guarded_grabber_->getMutex());
                if (!guarded_grabber_->getGrabber()->isRunning())
                    guarded_grabber_->getGrabber()->stop();
            }
            connection_->disconnect();
            std::stringstream ss;
            ss << "Device " << this->getId() << ", " << this->getDescription() << " done." << std::endl;
            Logger::log(Logger::INFO, ss.str());
        }
    }

    void callback(const DataType& data) {
        auto ms = std::chrono::duration_cast< std::chrono::milliseconds >(
                std::chrono::system_clock::now().time_since_epoch() );
        size_t timestamp = (size_t) ms.count();

        auto timed_data = std::make_shared<TimedData<DataType>>(data, timestamp);

        std::stringstream ss;
        ss << "Device " << this->getId() << ", " << this->getDescription() << " - ";
        if(this->getBuffer()->pushBack(timed_data))
            Logger::log(Logger::WARN, ss.str() + " Buffer was full, overwriting data.\n");
        if(timer_->time()) {
            ss << " Buffer size: " << this->getBuffer()->getSize() << std::endl;
            Logger::log(Logger::INFO, ss.str());
        }
    }

private:
    std::shared_ptr<GuardedGrabber> guarded_grabber_;
    std::shared_ptr<Timer> timer_;
    std::shared_ptr<boost::signals2::connection> connection_;
};
#endif //PCL_CLOUD_REGISTRATION_TIMEDDATAGRABBERPRODUCER_HPP

//
// Created by sean on 07/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_SYNCHRONIZER_HPP
#define PCL_CLOUD_REGISTRATION_SYNCHRONIZER_HPP

#include "types/Timing.hpp"
#include "nodes/Producer.hpp"
#include "util/Timer.hpp"

class SyncProcessor;

class Synchronizer :
        public Producer<std::shared_ptr<std::vector<std::shared_ptr<Timing>>>>,
        public std::enable_shared_from_this<Synchronizer> {
    friend class SyncProcessor;

    typedef std::shared_ptr<Timing> TimingPtr;
    typedef Buffer<TimingPtr> TimingBuffer;
    typedef std::shared_ptr<TimingBuffer> TimingBufferPtr;

    // TODO: Get this FPS using grabber.getFrameRate()
    static constexpr double FPS = 30;
    static constexpr int MILLISEC_PER_SEC = 1000;

    // TODO: Tune this parameter, trade-off is full frames vs. time variance
    static constexpr double ACCEPTABLE_RATIO = 0.5;
    static constexpr size_t TIMEOUT = static_cast<size_t>((1/FPS) * MILLISEC_PER_SEC * ACCEPTABLE_RATIO);
    static constexpr const char* DESCRIPTION = "Synchronizer Node";

public:
    Synchronizer(size_t id);

    virtual void start() override ;

    virtual void stop() override ;

    void addInputBuffer(std::shared_ptr<BufferBase> buffer);

private:
    void addElementFromInputBuffer(TimingPtr element, size_t processor_id);

    bool isTempOutputElementFullySet() const;

    bool isTempOutputElementFullyEmpty() const;

    bool isItTimeToPushTempOutputElement(TimingPtr time) const;

    bool pushTempOutputElementToBuffer();

    void printTempOutputElementTimings() const;

    std::vector<std::shared_ptr<SyncProcessor>> processors_;
    std::mutex mutex_;
    std::shared_ptr<std::vector<TimingPtr>> temp_output_element;
    std::shared_ptr<Timer> timer_;
    size_t first_set_stamp;
};

#endif //PCL_CLOUD_REGISTRATION_SYNCHRONIZER_HPP

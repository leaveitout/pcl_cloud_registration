//
// Created by sean on 07/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_SYNCHPROCESSOR_HPP
#define PCL_CLOUD_REGISTRATION_SYNCHPROCESSOR_HPP

#include "types/Timing.hpp"
#include "nodes/Processor.hpp"
#include "nodes/Synchronizer.hpp"


class SyncProcessor : public Processor<std::shared_ptr<Timing>> {
    typedef std::shared_ptr<Timing> TimingPtr;

    static constexpr const char* DESCRIPTION = "Sync Processor";

public:
    SyncProcessor(size_t id,
                  std::shared_ptr<Synchronizer> synchronizer,
                  std::shared_ptr<Buffer<TimingPtr>> buffer ) :
            Node(id, DESCRIPTION),
            Processor<TimingPtr>(id, DESCRIPTION, buffer),
            synchronizer_ (synchronizer) {
    }

protected:
    virtual void processBufferElement(std::shared_ptr<Timing> element) {
        synchronizer_->addElementFromInputBuffer(element, getId());
    }

    std::shared_ptr<Synchronizer> synchronizer_;
};

#endif //PCL_CLOUD_REGISTRATION_SYNCHPROCESSOR_HPP

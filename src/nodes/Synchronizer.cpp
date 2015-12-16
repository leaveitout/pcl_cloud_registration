//
// Created by sean on 07/12/15.
//

#include "nodes/Synchronizer.hpp"
#include "nodes/SyncProcessor.hpp"

Synchronizer::Synchronizer(size_t id) :
        Node(id, DESCRIPTION),
        Producer(id, DESCRIPTION) {
}

void Synchronizer::addInputBuffer(std::shared_ptr<BufferBase> buffer) {
    std::shared_ptr<TimingBuffer> time_buffer = std::static_pointer_cast<TimingBuffer>(buffer);
    std::shared_ptr<SyncProcessor> processor =
            std::make_shared<SyncProcessor>(processors_.size(), this->shared_from_this(), time_buffer);
    processors_.push_back(processor);
    // Re-allocate temp_output_element to new size
    temp_output_element = std::make_shared<std::vector<TimingPtr>>(processors_.size());
}

void Synchronizer::start() {
    std::stringstream ss;
    ss << "Synchronizer " << this->getId() << ", " << this->getDescription();
    timer_ = std::make_shared<Timer>(ss.str());

    std::vector<std::thread> threads(processors_.size());

    for(size_t idx = 0; idx < threads.size(); idx++ )
        threads.at(idx) = std::thread( [=] { processors_.at(idx)->start(); });

    for(auto& thread : threads)
        thread.join();
}

void Synchronizer::stop() {
    std::vector<std::thread> threads(processors_.size());

    for(size_t idx = 0; idx < threads.size(); idx++ )
        threads.at(idx) = std::thread( [=] { processors_.at(idx)->stop(); });

    for(auto& thread : threads)
        thread.join();
}

void Synchronizer::addElementFromInputBuffer(TimingPtr element, size_t processor_id) {
    std::lock_guard<std::mutex> lock(mutex_);

    bool overwrite = false;

    if(!temp_output_element->at(processor_id)) {
        if(isTempOutputElementFullyEmpty()){
            temp_output_element->at(processor_id) = element;
            first_set_stamp = element->getStamp();
        }
        else {
            if(isItTimeToPushTempOutputElement(element)) {
                Logger::log(Logger::WARN, "Pushed, timeout!!!\n");
                overwrite = pushTempOutputElementToBuffer();
                temp_output_element->at(processor_id) = element;
                first_set_stamp = element->getStamp();
            }
            else {
                temp_output_element->at(processor_id) = element;
                if(isTempOutputElementFullySet()) {
                    Logger::log(Logger::WARN, "Pushed, fully set!!\n");
                    overwrite = pushTempOutputElementToBuffer();
                }
            }
        }
    }
    else {
        Logger::log(Logger::WARN, "Pushed, already set!!\n");
        overwrite = pushTempOutputElementToBuffer();
        temp_output_element->at(processor_id) = element;
        first_set_stamp = element->getStamp();
    }

    if(overwrite)
        Logger::log(Logger::WARN, "Warning! CloudAndImage Buffer was full, overwriting data!\n");

    if(timer_->time()) {
        std::stringstream ss;
        ss << this->getDescription() << " buffer size : " << this->getBuffer()->getSize() << std::endl;
        Logger::log(Logger::INFO, ss.str());
    }
}

bool Synchronizer::isTempOutputElementFullySet() const {
    for(auto time_ptr : *temp_output_element)
        if(!time_ptr)
            return false;

    return true;
}

bool Synchronizer::isTempOutputElementFullyEmpty() const {
    for(auto time_ptr : *temp_output_element)
        if(time_ptr)
            return false;

    return true;
}

bool Synchronizer::isItTimeToPushTempOutputElement(TimingPtr timePtr) const {
    if(timePtr->getStamp() - first_set_stamp > TIMEOUT) {
        Logger::log(Logger::WARN, "Reached timeout!\n");
        return true;
    }
    return false;
}

bool Synchronizer::pushTempOutputElementToBuffer() {
    printTempOutputElementTimings();
    bool overwrite = this->getBuffer()->pushBack(temp_output_element);
    temp_output_element = std::make_shared<std::vector<TimingPtr>>(processors_.size());
    return overwrite;
}

void Synchronizer::printTempOutputElementTimings() const {
    std::stringstream ss;
    ss << "Buffer Element Timings " << std::endl;
    ss << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    for(auto time_ptr : *temp_output_element) {
        if(time_ptr)
            ss << time_ptr->getStamp() << std::endl;
        else
            ss << "-------------------------------" << std::endl;
    }
    Logger::log(Logger::INFO, ss.str());
}


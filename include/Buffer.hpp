//
// Created by sean on 06/12/15.
// Test edit
//

#ifndef PCL_CLOUD_REGISTRATION_BUFFER_HPP
#define PCL_CLOUD_REGISTRATION_BUFFER_HPP

#include <memory>
#include <condition_variable>
#include <boost/circular_buffer.hpp>

class BufferBase {
public:
    static const int TIME_OUT_MS_DEFAULT = 10;
    static const size_t DEFAULT_BUFF_SIZE = 100;

protected:
    BufferBase(int time_out_ms = TIME_OUT_MS_DEFAULT) :
            time_out_ms_ (time_out_ms) {
    }

public:
    virtual bool isFull() = 0;

    virtual bool isEmpty() = 0;

    virtual size_t getSize() = 0;

    virtual size_t getCapacity() = 0;

//    virtual ~BufferBase() { }

protected:
    std::mutex mutex_;
    std::condition_variable buff_empty_;
    BufferBase(const BufferBase&) = delete;
    BufferBase& operator = (const BufferBase&) = delete;
    std::chrono::duration<int, std::milli> time_out_ms_;
};

template <typename Type>
class Buffer : public BufferBase {
public:
    Buffer(size_t buff_size, int time_out_ms) :
            BufferBase(time_out_ms) {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.set_capacity(buff_size);
    }

    Buffer(size_t buff_size = DEFAULT_BUFF_SIZE) :
            Buffer(buff_size, TIME_OUT_MS_DEFAULT) {
    }

    inline bool isFull() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return (buffer_.full());
    }

    inline bool isEmpty() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return (buffer_.empty());
    }

    inline size_t getSize() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return (buffer_.size());
    }

    inline size_t getCapacity() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return (buffer_.capacity());
    }

    bool pushBack(Type new_item) {
        bool overwrite = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(buffer_.full())
                overwrite = true;
            buffer_.push_back(new_item);
        }
        buff_empty_.notify_one();
        return overwrite;
    }

    Type getFront() {
        std::unique_lock<std::mutex> lock(mutex_);
        if(buff_empty_.wait_for(lock, time_out_ms_, [&](){ return !buffer_.empty();})) {
            Type front_item = buffer_.front();
            buffer_.pop_front();
            return front_item;
        }
        else
            return nullptr;
    }

private:
    boost::circular_buffer<Type> buffer_;
    Buffer(const Buffer&) = delete;
    Buffer& operator = (const Buffer&) = delete;
};

#endif //PCL_CLOUD_REGISTRATION_BUFFER_HPP

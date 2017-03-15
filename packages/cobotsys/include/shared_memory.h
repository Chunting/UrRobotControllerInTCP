//
// Created by 潘绪洋 on 17-2-7.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_SHARED_MEMORY_H
#define PROJECT_SHARED_MEMORY_H

#include "common_macro.h"
#include <stdint.h>
#include <string>
#include <iostream>
#include <memory>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/vector.hpp>

namespace cobotsys {
namespace common {

using namespace boost;

class SharedMemory {
protected:
    typedef interprocess::allocator<char, interprocess::managed_shared_memory::segment_manager> char_allocator;
    typedef interprocess::vector<char, char_allocator> char_vector;
public:
    SharedMemory();
    ~SharedMemory();

    bool create(const std::string &memory_id, size_t memory_size);
    bool open(const std::string &memory_id);
    bool open(const std::string &memory_id, size_t memory_size);
    bool isValid() const;
    size_t size() const;
    bool write(const void *data_ptr, size_t data_len, size_t offset = 0);
    bool read(void *data_ptr, size_t data_len, size_t offset = 0);

    void *rawDataAt(size_t offset = 0);

    bool write(const std::string &msg);

    template<class T>
    bool read(T &t){
        return read(&t, sizeof(t));
    }

    template<class T>
    bool write(const T &t){
        return write(&t, sizeof(t));
    }

    // 如果需要读写读写，就应用这个。
    bool lock();
    bool unlock();
    bool tryLock();

    boost::interprocess::interprocess_semaphore &getSemaphore();

protected:
    void removeNamedResource(const std::string &s);

    void addRef();
    void decRef();
private:
    std::string shared_memory_id_;
    size_t shared_memory_size_;
    char_vector *ptr_vector_;
    boost::interprocess::interprocess_mutex *ptr_mutex_;
    boost::interprocess::interprocess_semaphore *ptr_semaphore_;
    uint32_t *ptr_ucounter_;

    boost::interprocess::managed_shared_memory bi_managed_;
};

template<class Ty>
class SharedStructData {
public:
    SharedStructData(){
    }

    bool init(const std::string &name_id){
        return shared_memory.create(name_id, sizeof(Ty));
    }

    bool isValid() const{ return shared_memory.isValid(); }

    Ty &read(){
        shared_memory.lock();
        shared_memory.read(local_data);
        shared_memory.unlock();
        return local_data;
    }

    void sync(){
        shared_memory.lock();
        shared_memory.write(local_data);
        shared_memory.unlock();
    }

protected:
    SharedMemory shared_memory;
    Ty local_data;
};
}
}


#endif //PROJECT_SHARED_MEMORY_H

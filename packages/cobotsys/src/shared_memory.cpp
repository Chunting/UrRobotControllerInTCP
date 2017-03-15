//
// Created by 潘绪洋 on 17-2-7.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <iomanip>
#include "include/shared_memory.h"

namespace cobotsys {
namespace common {

using namespace boost;
using boost::interprocess::interprocess_mutex;
using boost::interprocess::unique_instance;
using boost::interprocess::create_only;
using boost::interprocess::open_only;
using boost::interprocess::open_or_create;
using boost::interprocess::managed_shared_memory;


SharedMemory::SharedMemory(){
}

SharedMemory::~SharedMemory(){
    if (isValid())
        decRef();
}


bool SharedMemory::create(const std::string &memory_id, size_t memory_size){
    bool bresult = false;
    if (memory_id.empty() || memory_size == 0)
        return bresult;

    try {
        removeNamedResource(memory_id); // 删除现有的
        auto size_need = sizeof(interprocess_mutex) + sizeof(char_vector) + memory_size + 1024;

        bi_managed_ = managed_shared_memory(create_only, memory_id.c_str(), size_need);
        ptr_mutex_ = bi_managed_.construct<interprocess_mutex>((memory_id + "mtx").c_str())();
        ptr_semaphore_ = bi_managed_.construct<interprocess::interprocess_semaphore>((memory_id + "se").c_str())(0);
        ptr_ucounter_ = bi_managed_.construct<uint32_t>("uint32")(0);
        ptr_vector_ = bi_managed_.construct<char_vector>((memory_id + "vec").c_str())(
                char_allocator(bi_managed_.get_segment_manager()));
        ptr_vector_->resize(memory_size);

        shared_memory_size_ = memory_size;
        shared_memory_id_ = memory_id;
        addRef();
        bresult = true;
    } catch (boost::interprocess::interprocess_exception &ex) {
        std::cerr << ex.what() << std::endl;
    }
    return bresult;
}

bool SharedMemory::open(const std::string &memory_id, size_t memory_size){
    bool bresult = false;
    if (memory_id.empty() || memory_size == 0)
        return bresult;

    try {
        auto size_need = sizeof(interprocess_mutex) + sizeof(char_vector) + memory_size + 1024;

        bi_managed_ = managed_shared_memory(open_or_create, memory_id.c_str(), size_need);
        ptr_mutex_ = bi_managed_.find_or_construct<interprocess_mutex>((memory_id + "mtx").c_str())();
        ptr_semaphore_ = bi_managed_.find_or_construct<interprocess::interprocess_semaphore>((memory_id + "se").c_str())(0);
        ptr_ucounter_ = bi_managed_.find_or_construct<uint32_t>("uint32")(0);
        ptr_vector_ = bi_managed_.find_or_construct<char_vector>((memory_id + "vec").c_str())(
                char_allocator(bi_managed_.get_segment_manager()));
        ptr_vector_->resize(memory_size);

        shared_memory_size_ = memory_size;
        shared_memory_id_ = memory_id;
        addRef();
        bresult = true;
    } catch (boost::interprocess::interprocess_exception &ex) {
        std::cerr << ex.what() << std::endl;
    }
    return bresult;
}

bool SharedMemory::open(const std::string &memory_id){
    bool bresult = false;
    if (memory_id.empty())
        return bresult;

    try {
        bi_managed_ = managed_shared_memory(open_only, memory_id.c_str());
        ptr_mutex_ = bi_managed_.find<interprocess_mutex>((memory_id + "mtx").c_str()).first;
        ptr_semaphore_ = bi_managed_.find<interprocess::interprocess_semaphore>((memory_id + "se").c_str()).first;
        ptr_ucounter_ = bi_managed_.find<uint32_t>("uint32").first;
        ptr_vector_ = bi_managed_.find<char_vector>((memory_id + "vec").c_str()).first;

        shared_memory_size_ = ptr_vector_->size();
        shared_memory_id_ = memory_id;
        addRef();
        bresult = true;
    } catch (boost::interprocess::interprocess_exception &ex) {
        STD_CERR << ex.what() << ", " << memory_id << std::endl;
    }
    return bresult;
}

bool SharedMemory::isValid() const{
    return shared_memory_id_.size() != 0;
}

size_t SharedMemory::size() const{
    return shared_memory_size_;
}


bool SharedMemory::read(void *data_ptr, size_t data_len, size_t offset){
    if (isValid()) {
        if ((data_len + offset) <= shared_memory_size_) {
            memcpy(data_ptr, rawDataAt(offset), data_len);
            return true;
        }
    }
    return false;
}

bool SharedMemory::write(const void *data_ptr, size_t data_len, size_t offset){
    if (isValid()) {
        if ((data_len + offset) <= shared_memory_size_) {
            memcpy(rawDataAt(offset), data_ptr, data_len);
            return true;
        }
    }
    return false;
}

bool SharedMemory::write(const std::string &msg){
    if (isValid()) {
        if (msg.size() <= shared_memory_size_) {
            return write(msg.c_str(), msg.size());
        } else {
            std::cerr << "Message length is over the shared mem size." << std::endl;
        }
    }
    return false;
}

bool SharedMemory::lock(){
    ptr_mutex_->lock();
    return true;
}

bool SharedMemory::unlock(){
    ptr_mutex_->unlock();
    return true;
}

bool SharedMemory::tryLock(){
    return ptr_mutex_->try_lock();
}

void *SharedMemory::rawDataAt(size_t offset){
    return &(ptr_vector_->at(offset));
}

void SharedMemory::removeNamedResource(const std::string &s){
    boost::interprocess::shared_memory_object::remove(s.c_str());
    std::cout << "Remove Shared Resource: " << s << std::endl;
}

boost::interprocess::interprocess_semaphore &SharedMemory::getSemaphore(){
    return *ptr_semaphore_;
}

void SharedMemory::addRef(){
    ptr_mutex_->lock();
    (*ptr_ucounter_)++;
    ptr_mutex_->unlock();
}

void SharedMemory::decRef(){
    uint32_t rCount = 0;
    ptr_mutex_->lock();
    (*ptr_ucounter_)--;
    rCount = (*ptr_ucounter_);
    ptr_mutex_->unlock();
    if (rCount == 0) {
        removeNamedResource(shared_memory_id_);
    }
}




}
}

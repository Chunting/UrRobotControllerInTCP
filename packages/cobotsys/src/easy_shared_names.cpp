//
// Created by 潘绪洋 on 17-2-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "include/easy_shared_names.h"

namespace cobotsys {
namespace common {

const std::string EasySharedNames::managed_shared_memory_name = "api_easy_shared_names";

EasySharedNames::EasySharedNames(const std::string& share_names_id){
    local_name_id = share_names_id;
    local_name_root = managed_shared_memory_name + "_" + local_name_id;
    local_name_obj = managed_shared_memory_name + "_" + local_name_id + "_obj";
    ptr_shared = nullptr;
    openOrCreateSharedNames();
}

EasySharedNames::~EasySharedNames(){
    if (ptr_shared)
        decRef();
}

bool EasySharedNames::tryOpenSharedNames(){
    bool bresult = false;
    try {
        managed_memory = std::make_shared<boost::interprocess::managed_shared_memory>(
                boost::interprocess::open_only,
                localNameRoot()
        );
        ptr_shared = managed_memory->find<SharedNames>(localNameObj()).first;

        addRef();
        bresult = true;
    } catch (boost::interprocess::interprocess_exception& ex) {
        printf("error: %s\n", ex.what());
        bresult = false;
    }
    return bresult;
}

bool EasySharedNames::createSharedNames(){
    bool bresult = false;
    try {
        boost::interprocess::shared_memory_object::remove(localNameRoot());
        managed_memory = std::make_shared<boost::interprocess::managed_shared_memory>(
                boost::interprocess::create_only,
                localNameRoot(),
                1024 * 256
        );
        ptr_shared = managed_memory->construct<SharedNames>(localNameObj())(
                StringAllocator(managed_memory->get_segment_manager()));

        addRef();
        bresult = true;
    } catch (boost::interprocess::interprocess_exception& ex) {
        std::cerr << ex.what() << std::endl;
    }
    return bresult;
}

void EasySharedNames::openOrCreateSharedNames(){
    if (tryOpenSharedNames())
        return;

    if (createSharedNames())
        return;

    std::cerr << "Can't open or create shared names: " << localNameRoot() << std::endl;
    throw std::exception();
}

void EasySharedNames::addRef(){
    ptr_shared->mutex.lock();
    ptr_shared->refCount++;
    ptr_shared->mutex.unlock();
}

void EasySharedNames::decRef(){
    ptr_shared->mutex.lock();
    if (ptr_shared->refCount) {
        ptr_shared->refCount--;
    }
    auto refCount = ptr_shared->refCount;
    ptr_shared->mutex.unlock();

    if (refCount == 0) {
        auto isremoved = boost::interprocess::shared_memory_object::remove(localNameRoot());
        std::cout << "(Release)EasySharedNames: " << localNameRoot() << ":"
                  << std::boolalpha << isremoved << std::noboolalpha << std::endl;
    }
}

bool EasySharedNames::hasName(const std::string& sh_name){
    if (ptr_shared) {
        bool name_exist = false;
        ptr_shared->mutex.lock();
        for (const auto& name : ptr_shared->names) {
            if (name.c_str() == sh_name) {
                name_exist = true;
                break;
            }
        }
        ptr_shared->mutex.unlock();
        return name_exist;
    }
    return false;
}

bool EasySharedNames::removeName(const std::string& sh_name){
    if (ptr_shared) {
        ptr_shared->mutex.lock();
        ptr_shared->names.erase(std::remove_if(
                ptr_shared->names.begin(),
                ptr_shared->names.end(),
                [=](const ShmString& s){ return s.c_str() == sh_name; }
        ), ptr_shared->names.end());
        ptr_shared->mutex.unlock();
        std::cout << "EasySharedNames::removeName : " << sh_name << std::endl;
        return true;
    }
    return false;
}

bool EasySharedNames::pushName(const std::string& sh_name){
    if (hasName(sh_name))
        return false;

    if (ptr_shared) {
        ptr_shared->mutex.lock();
        ptr_shared->names.push_back(toShared(sh_name));
        ptr_shared->mutex.unlock();
        return true;
    }
    return false;
}

EasySharedNames::ShmString EasySharedNames::toShared(const std::string& s){
    ShmString name_to_push(managed_memory->get_segment_manager());
    name_to_push = s.c_str();
    return name_to_push;
}

std::vector<std::string> EasySharedNames::getNames(){
    std::vector<std::string> names;
    if (ptr_shared) {
        ptr_shared->mutex.lock();
        for (const auto& name : ptr_shared->names) {
            names.push_back(name.c_str());
        }
        ptr_shared->mutex.unlock();
    }
    return names;
}

void EasySharedNames::remove(const std::string& share_names_id){
    auto name_id = managed_shared_memory_name + "_" + share_names_id;
    boost::interprocess::shared_memory_object::remove(name_id.c_str());
}

void EasySharedNames::dumpAllNames(std::ostream& oss, const std::string& share_names_id){
    EasySharedNames sharedNames(share_names_id);
    auto names = sharedNames.getNames();
    oss << "TOTAL REF OBJECT COUNT: " << sharedNames.refCount() << std::endl;
    for (const auto& name : names) {
        oss << name << std::endl;
    }
}

const char* EasySharedNames::localNameRoot() const{

    return local_name_root.c_str();
}

const char* EasySharedNames::localNameObj() const{

    return local_name_obj.c_str();
}


EasySharedNames::SharedNames::SharedNames(const EasySharedNames::StringAllocator& a) :
        names(a){
    refCount = 0;
}
}
}
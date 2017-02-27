//
// Created by 潘绪洋 on 17-2-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_COMMON_EASY_CV_MAT_SHARED_NAMES_H
#define COBOTSYS_COMMON_EASY_CV_MAT_SHARED_NAMES_H

#include "shared_memory.h"
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/containers/vector.hpp>

#define EASY_SHARED_NAME_OBJ "shared_name_obj"

namespace cobotsys {
namespace common {

class EasySharedNames {
protected:
    typedef boost::interprocess::managed_shared_memory::segment_manager SegmentManager;
    typedef boost::interprocess::allocator<char, SegmentManager> CharAllocator;
    typedef boost::interprocess::basic_string<char, std::char_traits<char>, CharAllocator> ShmString;
    typedef boost::interprocess::allocator<ShmString, SegmentManager> StringAllocator;
    typedef boost::interprocess::vector<ShmString, StringAllocator> ShmStringVector;
public:
    EasySharedNames(const std::string &share_names_id = std::string());
    ~EasySharedNames();

    bool hasName(const std::string &sh_name);
    bool pushName(const std::string &sh_name);
    bool removeName(const std::string &sh_name);
    std::vector<std::string> getNames();

    uint32_t refCount() const{ return ptr_shared ? ptr_shared->refCount : 0; }

    static void remove(const std::string &share_names_id = std::string());
    static void dumpAllNames(std::ostream &oss, const std::string &share_names_id = std::string());
protected:
    bool tryOpenSharedNames();
    bool createSharedNames();
    void openOrCreateSharedNames();

    void addRef();
    void decRef();

    const char *localNameRoot() const;
    const char *localNameObj() const;

    ShmString toShared(const std::string &s);
    static const std::string managed_shared_memory_name;
protected:
    std::shared_ptr<boost::interprocess::managed_shared_memory> managed_memory;
    std::string local_name_id;
    std::string local_name_root;
    std::string local_name_obj;

    struct SharedNames {
        ShmStringVector names;
        uint32_t refCount;
        boost::interprocess::interprocess_mutex mutex;

        SharedNames(const StringAllocator &a);
    };
    SharedNames *ptr_shared;
};
}
}

#endif //COBOTSYS_COMMON_EASY_CV_MAT_SHARED_NAMES_H

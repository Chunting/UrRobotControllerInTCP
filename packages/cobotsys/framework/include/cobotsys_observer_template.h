//
// Created by 潘绪洋 on 17-3-15.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_OBSERVER_TEMPLATE_H
#define PROJECT_COBOTSYS_OBSERVER_TEMPLATE_H

#include <vector>
#include <memory>

namespace cobotsys {
//
//template<class T>
//class ObserverArray {
//public:
//    ObserverArray();
//    ~ObserverArray();
//
//    typedef std::shared_ptr<T> shared_type;
//
//    void attach(std::shared_ptr<T> observer){
//        for (auto& o : m_observerArray) {
//            if (o == observer)
//                return;
//        }
//
//        if (observer) {
//            m_observerArray.push_back(observer);
//        }
//    }
//
//    typename std::vector<shared_type>::iterator begin(){ return m_observerArray.begin(); }
//
//    typename std::vector<shared_type>::iterator end(){ return m_observerArray.end(); }
//
//    typename std::vector<shared_type>::const_iterator begin() const{ return m_observerArray.begin(); }
//
//    typename std::vector<shared_type>::const_iterator end() const{ return m_observerArray.end(); }
//
//protected:
//    std::vector<shared_type> m_observerArray;
//};
}


#endif //PROJECT_COBOTSYS_OBSERVER_TEMPLATE_H

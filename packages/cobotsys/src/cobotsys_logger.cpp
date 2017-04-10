//
// Created by 潘绪洋 on 17-1-19.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_logger.h"


namespace cobotsys {
Logger::Logger() {
    m_current_entry = "INFO";
    m_log_to_cout = false;
    m_cache_log_message = true;
    m_prefix_width = 12;
}

void Logger::println(const std::string& text) {
    if (m_append_filter)
        m_append_filter("", text);
}

void Logger::append(const std::string& entry, const std::string& message) {
    std::map<void*, std::function<void(const std::string&, const std::string&)> > obs;
    m_res_mutex.lock();
    obs = m_observers;

    if (m_cache_log_message)
        m_logs.push_back({entry, message});

    if (m_append_filter)
        m_append_filter(entry, message);

    if (m_log_to_cout) {
        std::cout << "["
                  << std::setw(m_prefix_width) << entry
                  << "] "
                  << message;
        if (message.size()) {
            if (message.back() != '\n')
                std::cout << std::endl;
        } else
            std::cout << std::endl;
    }
    m_res_mutex.unlock();

    for (auto& ob : obs) {
        ob.second(entry, message);
    }
}

void Logger::append(const std::string& message) {
    append(m_current_entry, message);
}

void Logger::setCurrentEntry(const std::string& entry) {
    m_current_entry = entry;
    for (auto& c : m_current_entry) c = toupper(c);
}

const std::string& Logger::currentEntry() const {
    return m_current_entry;
}

void Logger::setAppendFilter(std::function<void(const std::string& entry, const std::string& message)> filter) {
    if (filter) {
        m_append_filter = filter;
        for (auto& iter : m_logs) {
            m_append_filter(iter.entry, iter.message);
        }
    } else {
        m_append_filter = nullptr;
    }
}

Logger& Logger::instance() {
    static Logger logger;
    static bool first_init = true;
    if (first_init) {
        first_init = false;
        logger.logToCout(true);
        logger.m_prefix_width = 12;
        logger.m_cache_log_message = true;
    }
    return logger;
}

Logger::MessageWrapper Logger::message(const std::string& entry) {
    return MessageWrapper(entry, *this);
}

Logger::MessageWrapper Logger::message() {
    return message(m_current_entry);
}

Logger::MessageWrapper Logger::error() {
    return message("Error");
}

Logger::MessageWrapper Logger::warning() {
    return message("Warning");
}

Logger::MessageWrapper Logger::notice() {
    return message("Notice");
}

Logger::MessageWrapper Logger::info() {
    return message("Info");
}

void Logger::setCurrentInstanceName(const std::string& s) {
    m_current_instance_name = s;
}

void Logger::addFilter(void* obj, std::function<void(const std::string& entry, const std::string& message)> filter) {
    if (obj && filter) {
        m_res_mutex.lock();
        m_observers[obj] = filter;
        m_res_mutex.unlock();

        for (auto& iter : m_logs) {
            filter(iter.entry, iter.message);
        }
    }
}

void Logger::clrFilter(void* obj) {
    std::lock_guard<std::mutex> lockctx(m_res_mutex);
    m_observers.erase(obj);
}
}

cobotsys::Logger::MessageWrapper& endl(cobotsys::Logger::MessageWrapper& mwp) {
    mwp.endl();
    return mwp;
}

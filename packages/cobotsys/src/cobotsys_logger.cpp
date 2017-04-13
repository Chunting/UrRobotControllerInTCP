//
// Created by 潘绪洋 on 17-1-19.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_logger.h"


namespace cobotsys {

Logger::MessageWrapper::MessageWrapper(const MessageWrapper & r)
    : logger(r.logger) {
}

Logger::MessageWrapper::MessageWrapper(const std::string & e, Logger & r, LoggerLevel level)
    : logger(r) {
    oss << "[";
    oss << std::setw(5) << toString(level);
    oss << "]";
    if (e.size()) {
        oss << "[";
        oss << std::setw(logger.prefixWidth()) << e;
        oss << "]";
    }
    oss << " ";
}

Logger::MessageWrapper::~MessageWrapper() {
    auto strfinal = oss.str();
    if (strfinal.size()) {
        if (strfinal.at(strfinal.size() - 1) != '\n') {
            strfinal.push_back('\n');
        }
    }
    logger.append(strfinal);
}

void Logger::MessageWrapper::endl() {
    oss << std::endl;
}


std::string cobotsys::toString(LoggerLevel level) {
    switch (level) {
    case cobotsys::LoggerLevel::Debug:  return "DEBUG";  break;
    case cobotsys::LoggerLevel::Info:   return "INFO";   break;
    case cobotsys::LoggerLevel::Notice: return "NOTIC";  break;
    case cobotsys::LoggerLevel::Warning:return "WARN";   break;
    case cobotsys::LoggerLevel::Error:  return "ERROR";  break;
    case cobotsys::LoggerLevel::Fatal:  return "FATAL";  break;
    }
    return std::string();
}

Logger::Logger() {
    m_log_to_cout = false;
    m_cache_log_message = true;
    m_prefix_width = 12;
}

void Logger::append(const std::string& message) {
    std::map<void*, std::function<void(const std::string&)> > obs;
    m_res_mutex.lock();
    obs = m_observers;

    if (m_cache_log_message)
        m_logs.push_back(message);

    if (m_append_filter)
        m_append_filter(message);

    if (m_log_to_cout) {
        std::cout << message;
    }
    m_res_mutex.unlock();

    for (auto& ob : obs) {
        ob.second(message);
    }
}

void Logger::setAppendFilter(std::function<void(const std::string& message)> filter) {
    if (filter) {
        m_append_filter = filter;
        for (auto& iter : m_logs) {
            m_append_filter(iter);
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

Logger::MessageWrapper Logger::message(const std::string& entry, LoggerLevel level) {
    return MessageWrapper(entry, *this, level);
}

Logger::MessageWrapper Logger::message() {
    return message("", LoggerLevel::Debug);
}

Logger::MessageWrapper Logger::error() {
    return message("", LoggerLevel::Error);
}

Logger::MessageWrapper Logger::fatal() {
    return message("", LoggerLevel::Fatal);
}

Logger::MessageWrapper Logger::warning() {
    return message("", LoggerLevel::Warning);
}

Logger::MessageWrapper Logger::notice() {
    return message("", LoggerLevel::Notice);
}

Logger::MessageWrapper Logger::info() {
    return message("", LoggerLevel::Info);
}

void Logger::setCurrentInstanceName(const std::string& s) {
    m_current_instance_name = s;
}

void Logger::addFilter(void* obj, std::function<void(const std::string& message)> filter) {
    if (obj && filter) {
        m_res_mutex.lock();
        m_observers[obj] = filter;
        m_res_mutex.unlock();

        for (auto& iter : m_logs) {
            filter(iter);
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

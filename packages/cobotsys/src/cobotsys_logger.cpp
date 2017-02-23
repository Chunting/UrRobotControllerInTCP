//
// Created by 潘绪洋 on 17-1-19.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_logger.h"


namespace cobotsys {
Logger::Logger(){
    _current_entry = "INFO";
    _log_to_cout = false;
    _cache_log_message = false;
    _prefix_width = 12;
}

void Logger::println(const std::string &text){
    if (_append_filter)
        _append_filter("", text);
}

void Logger::append(const std::string &entry, const std::string &message){
    if (_cache_log_message)
        logs_.push_back({entry, message});

    if (_append_filter)
        _append_filter(entry, message);

    if (_log_to_cout) {
        std::cout << "["
                  << std::setw(_prefix_width) << entry
                  << "] "
                  << message;
        if (message.size()) {
            if (message.back() != '\n')
                std::cout << std::endl;
        } else
            std::cout << std::endl;
    }
}

void Logger::append(const std::string &message){
    append(_current_entry, message);
}

void Logger::setCurrentEntry(const std::string &entry){
    _current_entry = entry;
    for (auto &c : _current_entry) c = toupper(c);
}

const std::string &Logger::currentEntry() const{
    return _current_entry;
}

void Logger::setAppendFilter(std::function<void(const std::string &entry, const std::string &message)> filter){
    if (filter) {
        _append_filter = filter;
        for (auto &iter : logs_) {
            _append_filter(iter.entry, iter.message);
        }
    } else {
        _append_filter = nullptr;
    }
}

Logger &Logger::instance(){
    static Logger logger;
    static bool first_init = true;
    if (first_init) {
        first_init = false;
        logger.logToCout(true);
        logger._prefix_width = 12;
        logger._cache_log_message = false;
    }
    return logger;
}

Logger::MessageWrapper Logger::message(const std::string &entry){
    return MessageWrapper(entry, *this);
}

Logger::MessageWrapper Logger::message(){
    return message(_current_entry);
}

Logger::MessageWrapper Logger::error(){
    return message("Error");
}

Logger::MessageWrapper Logger::warning(){
    return message("Warning");
}

Logger::MessageWrapper Logger::notice(){
    return message("Notice");
}

Logger::MessageWrapper Logger::info(){
    return message("Info");
}

void Logger::setCurrentInstanceName(const std::string &s){
    _current_instance_name = s;
}
}

cobotsys::Logger::MessageWrapper &endl(cobotsys::Logger::MessageWrapper &mwp){
    mwp.endl();
    return mwp;
}

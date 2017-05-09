//
// Created by 潘绪洋 on 17-1-19.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_BASIC_LOGGER_H
#define PROJECT_COBOTSYS_BASIC_LOGGER_H

#include <iostream>
#include <sstream>
#include <iomanip>
#include <map>
#include <string>
#include <functional>
#include <deque>
#include <mutex>
#include <vector>
#include <chrono>
#include <ctime>
#include <boost/format.hpp>

std::ostream& operator<<(std::ostream& oss, const std::vector<double>& vals);


struct _Setfixedfloat {
    int width;
    int precision;
};

template<class T>
struct _Putfixedfloats {
    int width;
    int precision;
    double factor;
    const std::vector<T>& vals;
};

inline _Setfixedfloat setfixedfloat(int w, int p) { return {w, p}; }

template<class T>
_Putfixedfloats<T>
putfixedfloats(int w, int p, const std::vector<T>& v, double factor = 1) {
    return {w, p, factor, v};
}

template<typename _CharT, typename _Traits>
std::basic_ostream<_CharT, _Traits>& operator<<(std::basic_ostream<_CharT, _Traits>& __os, _Setfixedfloat __f) {
    __os.setf(std::basic_ostream<_CharT, _Traits>::fixed);
    __os.precision(__f.precision);
    __os << std::setw(__f.width);
    return __os;
}

template<typename _CharT, typename _Traits, class T>
std::basic_ostream<_CharT, _Traits>& operator<<(std::basic_ostream<_CharT, _Traits>& __os, _Putfixedfloats<T> __f) {
    auto old_precision = __os.precision();
    for (size_t i = 0; i < __f.vals.size(); i++) {
        __os << setfixedfloat(__f.width, __f.precision) << __f.vals[i] * __f.factor;
        if (i + 1 < __f.vals.size()) {
            __os << ", ";
        }
    }
    __os.precision(old_precision);
    return __os;
}

namespace cobotsys {

enum class LoggerLevel {
    Debug,
    Info,
    Notice,
    Warning,
    Error,
    Fatal,
};

std::string toString(LoggerLevel level);

class Logger {
public:
    class MessageWrapper {
    public:
        MessageWrapper(const std::string& e, Logger& r, LoggerLevel level);
        MessageWrapper(const MessageWrapper& r);
        ~MessageWrapper();

        template<class T>
        MessageWrapper& operator<<(const T& t) {
            oss << t;
            return *this;
        }

        MessageWrapper& operator<<(MessageWrapper& (* pf)(MessageWrapper&)) {
            return pf(*this);
        }

        void endl();

    private:
        std::stringstream oss;
        Logger& logger;
    };
public:
    Logger();

    void append(const std::string& message);

    MessageWrapper message(const std::string& module_ = "", LoggerLevel level = LoggerLevel::Debug);
    MessageWrapper error(const std::string& module_ = "");
    MessageWrapper warning(const std::string& module_ = "");
    MessageWrapper notice(const std::string& module_ = "");
    MessageWrapper info(const std::string& module_ = "");
    MessageWrapper fatal(const std::string& module_ = "");
    MessageWrapper debug(const std::string& module_ = "");

    void setAppendFilter(std::function<void(const std::string& message)> filter);
    void addFilter(void* obj, std::function<void(const std::string& message)> filter);
    void clrFilter(void* obj);

    static Logger& instance();

    int prefixWidth() const { return m_prefix_width; }

    bool logToCout() const { return m_log_to_cout; }

    bool logToCout(bool enabled) {
        m_log_to_cout = enabled;
        return m_log_to_cout;
    }

    void setCurrentInstanceName(const std::string& s);

protected:
    std::deque<std::string> m_logs;
    std::function<void(const std::string&)> m_append_filter;
    bool m_log_to_cout;
    bool m_cache_log_message;
    int m_prefix_width;
    std::string m_current_instance_name;
    std::mutex m_res_mutex;

    std::map<void*, std::function<void(const std::string&)> > m_observers;
};
}
#define COBOT_LOG    cobotsys::Logger::instance()
using std::setw;

cobotsys::Logger::MessageWrapper& endl(cobotsys::Logger::MessageWrapper& mwp);

#endif //PROJECT_COBOTSYS_BASIC_LOGGER_H

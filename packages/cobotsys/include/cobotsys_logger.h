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

namespace cobotsys {

class Logger {
public:
    class MessageWrapper {
    public:
        MessageWrapper(const MessageWrapper& r)
                : entry(r.entry), logger(r.logger), oss(r.oss.str()) {
        }

        ~MessageWrapper() {
            if (oss.str().size())
                endl();
        }

        template<class T>
        MessageWrapper& operator<<(const T& t) {
            oss << t;
            return *this;
        }

        MessageWrapper& operator<<(MessageWrapper& (* pf)(MessageWrapper&)) {
            return pf(*this);
        }

        void endl() {
            logger.append(entry, oss.str());
            oss.str("");
        }

    private:
        std::string entry;
        std::stringstream oss;
        Logger& logger;

        MessageWrapper(const std::string& e, Logger& r)
                : entry(e), logger(r) {
        }

        friend class Logger;
    };
public:
    Logger();

    void append(const std::string& entry, const std::string& message);
    void append(const std::string& message); // Will use current entry

    void println(const std::string& text);

    MessageWrapper message(const std::string& entry);
    MessageWrapper message();
    MessageWrapper error();
    MessageWrapper warning();
    MessageWrapper notice();
    MessageWrapper info();

    void setCurrentEntry(const std::string& entry); // Will change all to Upper.
    const std::string& currentEntry() const;

    void setAppendFilter(std::function<void(const std::string& entry, const std::string& message)> filter);
    void addFilter(void* obj, std::function<void(const std::string& entry, const std::string& message)> filter);
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
    struct LogMessage {
        std::string entry;
        std::string message;
    };
    std::deque<LogMessage> m_logs;
    std::string m_current_entry;
    std::function<void(const std::string&, const std::string&)> m_append_filter;
    bool m_log_to_cout;
    bool m_cache_log_message;
    int m_prefix_width;
    std::string m_current_instance_name;
    std::mutex m_res_mutex;

    std::map<void*, std::function<void(const std::string&, const std::string&)> > m_observers;
};
}
#define COBOT_LOG    cobotsys::Logger::instance()
using std::setw;

cobotsys::Logger::MessageWrapper& endl(cobotsys::Logger::MessageWrapper& mwp);

#endif //PROJECT_COBOTSYS_BASIC_LOGGER_H

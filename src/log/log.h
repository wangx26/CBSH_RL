#ifndef LOG_H
#define LOG_H

#include <iostream>
#include <sstream>
#include <future>

#include <log4cpp/Category.hh>
#include <log4cpp/OstreamAppender.hh>
#include <log4cpp/Priority.hh>
#include <log4cpp/PatternLayout.hh>

#define LOG_DEBUG(args) \
    {   \
        Log::GetInstance()->Debug(args); \
    }

#define LOG_INFO(args) \
    {   \
        Log::GetInstance()->Info(args); \
    }

#define LOG_WARN(args) \
    {   \
        Log::GetInstance()->Warn(args); \
    }

#define LOG_ERROR(args) \
    {   \
        Log::GetInstance()->Error(args); \
    }

#define LOG_FATAL(args) \
    {   \
        Log::GetInstance()->Fatal(args); \
    }

#define LOG_DEBUG_STREAM(args) \
    {   \
        std::stringstream log_ss; \
        log_ss<<args; \
        Log::GetInstance()->Debug(log_ss.str()); \
    }

#define LOG_INFO_STREAM(args) \
    {   \
        std::stringstream log_ss; \
        log_ss<<args; \
        Log::GetInstance()->Info(log_ss.str()); \
    }

#define LOG_WARN_STREAM(args) \
    {   \
        std::stringstream log_ss; \
        log_ss<<args; \
        Log::GetInstance()->Warn(log_ss.str()); \
    }

#define LOG_ERROR_STREAM(args) \
    {   \
        std::stringstream log_ss; \
        log_ss<<args; \
        Log::GetInstance()->Error(log_ss.str()); \
    }

#define LOG_FATAL_STREAM(args) \
    {   \
        std::stringstream log_ss; \
        log_ss<<args; \
        Log::GetInstance()->Fatal(log_ss.str()); \
    }


namespace mapf {

    class Log {
    public:
        typedef Log* Ptr;
        static Ptr GetInstance();
        static void DestroyInstance();

        void Fatal(const std::string &msg);
        void Error(const std::string &msg);
        void Warn(const std::string &msg);
        void Info(const std::string &msg);
        void Debug(const std::string &msg);

    private:
        Log();
        ~Log();

        static Ptr instance_;
        static std::mutex instance_mutex_;

        log4cpp::Category& root_;
    };

}

#endif //LOG_H

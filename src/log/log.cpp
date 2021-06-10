#include "log.h"

#include <log4cpp/PropertyConfigurator.hh>

namespace mapf {

    Log::Ptr Log::instance_ = nullptr;
    std::mutex Log::instance_mutex_;

    Log::Ptr Log::GetInstance() {
        std::lock_guard<std::mutex> lock(instance_mutex_);
        if (!instance_) {
            instance_ = new Log;
        }
        return instance_;
    }

    void Log::DestroyInstance() {
        std::lock_guard<std::mutex> lock(instance_mutex_);
        if (instance_) {
            delete instance_;
        }
    }

    Log::Log() :
    root_(log4cpp::Category::getRoot())
    {
        try
        {
            log4cpp::PropertyConfigurator::configure(std::string(std::getenv("HOME"))+"/mapf/data/config/log.conf");
        }
        catch (log4cpp::ConfigureFailure& f)
        {
            std::cerr << "Log configure problem: " << f.what() << std::endl;
        }
    }

    Log::~Log() {
        log4cpp::Category::shutdown();
    }

    void Log::Fatal(const std::string &msg) {
        root_.fatal(msg);
    }
    void Log::Error(const std::string &msg) {
        root_.error(msg);
    }
    void Log::Warn(const std::string &msg) {
        root_.warn(msg);
    }
    void Log::Info(const std::string &msg) {
        root_.info(msg);
    }
    void Log::Debug(const std::string &msg) {
        root_.debug(msg);
    }
}
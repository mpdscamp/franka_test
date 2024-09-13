#include "utils/logger.h"

Logger& Logger::getInstance() {
    static Logger instance;
    return instance;
}

Logger::Logger() : log_level_(Level::INFO), exit_flag_(false), log_thread_(&Logger::logWorker, this) {
    std::cout << "Logger initialized\n";
}

Logger::~Logger() {
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        exit_flag_ = true;
    }
    log_condition_.notify_all();
    if (log_thread_.joinable()) {
        log_thread_.join();
    }
    std::cout << "Logger destroyed\n";
}

void Logger::setLogLevel(Level level) {
    log_level_ = level;
}

void Logger::log(Level level, const std::string& message) {
    std::cout << "Logging: " << message << "\n";  // Debug print

    if (level >= log_level_) {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            std::ostringstream log_entry;
            log_entry << "[" << levelToString(level) << "] " << message;
            log_queue_.push(log_entry.str());
        }
        log_condition_.notify_one();
    }
}

void Logger::logWorker() {
    std::cout << "Logger worker thread started\n";  // Debug print

    while (true) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        log_condition_.wait(lock, [this] { return !log_queue_.empty() || exit_flag_; });

        while (!log_queue_.empty()) {
            const std::string& log_entry = log_queue_.front();
            std::cout << log_entry << std::endl;
            log_queue_.pop();
        }

        if (exit_flag_ && log_queue_.empty()) {
            break;
        }
    }
    std::cout << "Logger worker thread exiting\n";  // Debug print
}

std::string Logger::levelToString(Level level) const {
    switch (level) {
        case Level::INFO: return "INFO";
        case Level::DEBUG: return "DEBUG";
        case Level::WARNING: return "WARNING";
        case Level::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

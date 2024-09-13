#ifndef LOGGER_H_
#define LOGGER_H_

#include <iostream>
#include <sstream>
#include <mutex>
#include <memory>
#include <queue>
#include <thread>
#include <condition_variable>

class Logger {
public:
    enum class Level {
        INFO,
        DEBUG,
        WARNING,
        ERROR
    };

    static Logger& getInstance();

    void setLogLevel(Level level);
    void log(Level level, const std::string& message);

    ~Logger(); // Ensure proper cleanup

private:
    Logger();  // Constructor made private for Singleton pattern
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void logWorker();  // The worker thread function
    std::string levelToString(Level level) const;

    Level log_level_;
    std::mutex queue_mutex_;
    std::condition_variable log_condition_;
    std::queue<std::string> log_queue_;  // Queue for log messages
    bool exit_flag_;  // Flag to exit the logging thread
    std::thread log_thread_;  // Background thread for logging
};

#endif // LOGGER_H_

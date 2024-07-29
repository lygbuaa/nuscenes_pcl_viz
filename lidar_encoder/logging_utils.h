#ifndef __LOGGING_UTILS_H__
#define __LOGGING_UTILS_H__

/* set non-debug mode */
// #ifndef NDEBUG
//     #define NDEBUG 
// #endif

/** set debug mode */
#ifdef NDEBUG
    #undef NDEBUG 
#endif

#include <stdio.h>      /* printf */
#include <stdlib.h>     /* getenv */
#include <chrono>
#include <memory>
#include <string.h>

#define __LOG_USE_BASE_FILENAME__

#ifdef __LOG_USE_BASE_FILENAME__
    #ifdef __FILE_NAME__ /** since gcc-12 */
        #define __LOG_FILENAME__  __FILE_NAME__
    #else
        #define __LOG_FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
    #endif
#else
    #define __LOG_FILENAME__  __FILE__
#endif

//** log with fprintf *//
#define LOGPF(format, ...) fprintf(stderr ,"[%s:%d] " format "\n", __LOG_FILENAME__, __LINE__, ##__VA_ARGS__)

/**
 * 3: spdlog
 * others: fprintf
 */

#define __LOGGING_BACKEND__  0

/** ros2 does built-in spdlog, just find_package(spdlog REQUIRED) and link to spdlog */
#if __LOGGING_BACKEND__ == 3
    #include <spdlog/spdlog.h>
    #include <spdlog/fmt/bundled/printf.h>
    #include "spdlog/sinks/basic_file_sink.h"
    #include "spdlog/sinks/stdout_sinks.h"
    #include "spdlog/sinks/stdout_color_sinks.h"
    #include "spdlog/sinks/rotating_file_sink.h"

    template <class loggerPtr, class... Args>
    void loglineprintf(loggerPtr logger, spdlog::level::level_enum level, spdlog::source_loc loc, const char* fmt, const Args&... args) noexcept
    {
        if (logger && logger->should_log(level))
        {
            logger->log(loc, level, "{}", fmt::sprintf(fmt, args...));
        }
    }

    #define SPDLOG_LOGGER_PRINTF(logger, level, ...) \
        loglineprintf(logger, level, spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, __VA_ARGS__)

    //specific log implementation macros

    #define RLOGD(...) SPDLOG_LOGGER_PRINTF(spdlog::default_logger(),spdlog::level::debug,__VA_ARGS__)
    #define RLOGI(...) SPDLOG_LOGGER_PRINTF(spdlog::default_logger(),spdlog::level::info,__VA_ARGS__)
    #define RLOGW(...) SPDLOG_LOGGER_PRINTF(spdlog::default_logger(),spdlog::level::warn,__VA_ARGS__)
    #define RLOGE(...) SPDLOG_LOGGER_PRINTF(spdlog::default_logger(),spdlog::level::err,__VA_ARGS__)
    #define RLOGF(...) SPDLOG_LOGGER_PRINTF(spdlog::default_logger(),spdlog::level::critical,__VA_ARGS__)

    static inline void __setup_spdlog__(const char* log_file, spdlog::level::level_enum log_level)
    {
        std::vector<spdlog::sink_ptr> sinks;
        sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_mt>());
        /** auto-rotate log files, 20MB each, 5 in total, create new log file when process restart */
        sinks.push_back(std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_file, 1024*1024*20, 5, true));
        auto combined_logger = std::make_shared<spdlog::logger>("combined_logger", begin(sinks), end(sinks));

        spdlog::register_logger(combined_logger);
        spdlog::set_default_logger(combined_logger);
        spdlog::flush_every(std::chrono::seconds(3));
        spdlog::set_level(log_level);
        /** [%g:%#] for full file path */
        spdlog::set_pattern("[%L][%t][%Y-%m-%d %H:%M:%S.%f][%s:%#] %v");
    }

    /** call this before progress exit */
    static inline void __shutdown_spdlog__()
    {
        spdlog::shutdown();
    }

    static inline void __run_spdlog_logger_test__()
    {
        __setup_spdlog__("/tmp/atlantis/spdlog_test.log", spdlog::level::info);
        float magic = 12.3f;
        RLOGD("this is debug: %.2f", magic);
        RLOGI("this is info: %.2f", magic);
        RLOGW("this is warn: %.2f", magic);
        RLOGE("this is error: %.2f", magic);
        RLOGF("this is fatal: %.2f", magic);
    }

#else
    #define RLOGD(...) LOGPF(__VA_ARGS__)
    #define RLOGI(...) LOGPF(__VA_ARGS__)
    #define RLOGW(...) LOGPF(__VA_ARGS__)
    #define RLOGE(...) LOGPF(__VA_ARGS__)
    #define RLOGF(...) LOGPF(__VA_ARGS__)
#endif

/** profiling utility */
static inline __attribute__((always_inline)) uint64_t gfGetCurrentMicros()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::time_point_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now()).time_since_epoch()).count();
}

static inline std::shared_ptr<uint64_t> gfHangStopWatch(const char* func_name)
{
    uint64_t* pts_us = new uint64_t;
    *pts_us = gfGetCurrentMicros();
    RLOGD("stopwatch tick by %s", func_name);
    return std::shared_ptr<uint64_t>(pts_us, [func_name](uint64_t* ptr){
        uint64_t ts_us = gfGetCurrentMicros();
        RLOGI("stopwatch tock by %s, elapse: %ld us", func_name, (ts_us - *ptr));
        delete ptr;
    });
}

static inline std::string gShellCall(std::string cmd, const char* type)
{
    std::string result;
    FILE* fp = popen(cmd.c_str(), type);
    char buf[64] = {0};
    int32_t bytes_total = 0;
    int32_t bytes_read = 0;
    while( (bytes_read = fread(buf, 1, sizeof(buf), fp)) > 0 )
    {
        bytes_total += bytes_read;
        result.append(buf, bytes_read);
        memset(buf, 0, sizeof(buf));
    }

    pclose(fp);
    if(bytes_total <= 0){
        RLOGW("WTF! ShellCall (%s) result is empty!", cmd.c_str());
    } else {
        RLOGI("ShellCall (%s) result: %s", cmd.c_str(), result.c_str());
    }
    return result;
}

static inline void gMakeDir(std::string path)
{
    std::string cmd = "mkdir -p " + path;
    int32_t exit_code = system(cmd.c_str());
    RLOGI("ShellCall (%s) exit code: %d", cmd.c_str(), exit_code);
}


#define HANG_STOPWATCH() auto _ProfilingUtilsPtr_ = gfHangStopWatch(__FUNCTION__);

#endif //__LOGGING_UTILS_H__
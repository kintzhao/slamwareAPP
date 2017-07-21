/*
* log.h
* RPOS Log System
*
* Created By Tony Huang (tony@slamtec.com) at 2014-12-29
* Copyright 2014 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <rpos/rpos_config.h>
#include <rpos/core/rpos_core_config.h>
#include <rpos/system/target_info.h>
#include <stdarg.h>
#include <string>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <list>

#ifdef RPOS_TARGET_64BIT
#   define RPOS_LOG_POINTER_FMT "%016llx"
#else
#   define RPOS_LOG_POINTER_FMT "%08x"
#endif

namespace rpos { namespace system { namespace util {

    enum LogLevel {
        LogLevelDebug,
        LogLevelInfo,
        LogLevelWarn,
        LogLevelError,
        LogLevelFatal
    };

    struct LogAppenderConfig {
        LogLevel logLevel;
        std::vector<std::string> excludeLogSources;
    };

    struct FileLogAppenderConfig : public LogAppenderConfig {
        std::string filename;
    };

    struct LogConfig {
        LogAppenderConfig global;
        LogAppenderConfig console;
        std::vector<FileLogAppenderConfig> files;
    };

    RPOS_CORE_API void vlog(const char* source, LogLevel level, const char* msg, va_list args);
    RPOS_CORE_API void log(const char* source, LogLevel level, const char* msg, ...);

    RPOS_CORE_API void vdebug_out(const char* source, const char* msg, va_list args);
    RPOS_CORE_API void  debug_out(const char* source, const char* msg, ...);

    RPOS_CORE_API void vinfo_out(const char* source, const char* msg, va_list args);
    RPOS_CORE_API void  info_out(const char* source, const char* msg, ...);

    RPOS_CORE_API void vwarn_out(const char* source, const char* msg, va_list args);
    RPOS_CORE_API void  warn_out(const char* source, const char* msg, ...);

    RPOS_CORE_API void verror_out(const char* source, const char* msg, va_list args);
    RPOS_CORE_API void  error_out(const char* source, const char* msg, ...);

    RPOS_CORE_API void vfatal_out(const char* source, const char* msg, va_list args);
    RPOS_CORE_API void  fatal_out(const char* source, const char* msg, ...);

    class LogScope : private boost::noncopyable {
    public:
        RPOS_CORE_API LogScope(const std::string& source);
        RPOS_CORE_API ~LogScope();

    public:
        RPOS_CORE_API void vlog(LogLevel level, const char* msg, va_list args);
        RPOS_CORE_API void log(LogLevel level, const char* msg, ...);

        RPOS_CORE_API void vdebug_out(const char* msg, va_list args);
        RPOS_CORE_API void  debug_out(const char* msg, ...);

        RPOS_CORE_API void vinfo_out(const char* msg, va_list args);
        RPOS_CORE_API void  info_out(const char* msg, ...);

        RPOS_CORE_API void vwarn_out(const char* msg, va_list args);
        RPOS_CORE_API void  warn_out(const char* msg, ...);

        RPOS_CORE_API void verror_out(const char* msg, va_list args);
        RPOS_CORE_API void  error_out(const char* msg, ...);

        RPOS_CORE_API void vfatal_out(const char* msg, va_list args);
        RPOS_CORE_API void  fatal_out(const char* msg, ...);

    private:
        std::string source_;
    };

    class LogAppender : private boost::noncopyable {
    public:
        typedef boost::shared_ptr<LogAppender> Pointer;

    protected:
        RPOS_CORE_API LogAppender(LogLevel logLevel = LogLevelDebug);

    public:
        RPOS_CORE_API virtual ~LogAppender();

    public:
        RPOS_CORE_API LogLevel getLogLevel() const;
        RPOS_CORE_API void setLogLevel(LogLevel logLevel);

        RPOS_CORE_API void append(const std::string& logSource, LogLevel logLevel, const std::string& message);

        RPOS_CORE_API bool isExcluded(const std::string& logSource, LogLevel logLevel);

        RPOS_CORE_API std::vector<std::string>& excludeSources();

    protected:
        virtual void append_(const std::string& logSource, LogLevel logLevel, const std::string& message) = 0;

    private:
        LogLevel logLevel_;
        std::vector<std::string> excludeSources_;
    };

    class LogManager : private boost::noncopyable {
    public:
        typedef boost::shared_ptr<LogAppender> AppenderPointer;
        typedef boost::shared_ptr<LogManager> ManagerPointer;

    private:
        RPOS_CORE_API LogManager();

    public:
        RPOS_CORE_API ~LogManager();

        RPOS_CORE_API void append(const std::string& logSource, LogLevel logLevel, const std::string& message);

        RPOS_CORE_API bool isExcluded(const std::string& logSource, LogLevel logLevel);

        RPOS_CORE_API void addAppender(AppenderPointer appender);
        RPOS_CORE_API void removeAppender(AppenderPointer appender);
        RPOS_CORE_API void clearAppenders();

    public:
        // Helper functions
        RPOS_CORE_API void addConsoleAppender();
        RPOS_CORE_API void addFileAppender(const std::string& filename);

        RPOS_CORE_API void configWith(const LogConfig& config);

    public:
        RPOS_CORE_API static ManagerPointer defaultManager();

    private:
        static void createManager_();

    private:
        boost::mutex lock_;
        std::list<AppenderPointer> appenders_;
        LogAppenderConfig globalLogSettings_;
    };

    class ConsoleLogAppender : public LogAppender {
    public:
        RPOS_CORE_API ConsoleLogAppender(LogLevel logLevel = LogLevelDebug);
        RPOS_CORE_API ~ConsoleLogAppender();

    protected:
        RPOS_CORE_API virtual void append_(const std::string& logSource, LogLevel logLevel, const std::string& message);

    private:
        boost::mutex lock_;
    };

    class FileLogAppender : public LogAppender {
    public:
        RPOS_CORE_API FileLogAppender(const std::string& filename, LogLevel logLevel = LogLevelDebug);
        RPOS_CORE_API ~FileLogAppender();

    public:
        RPOS_CORE_API bool isAvailable() const;

    protected:
        RPOS_CORE_API virtual void append_(const std::string& logSource, LogLevel logLevel, const std::string& message);

    private:
        std::string filename_;
        FILE* file_;
        mutable boost::mutex lock_;
    };

} } }

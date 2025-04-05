#pragma once

#include <sstream>
#include <memory>
#include <cstdarg>

extern const char* __progname;

enum class CemLogLevel
{
    DEFAULT = -1,               /**< Default log level */
    OFF = 0x00,                 /**< Log level off */
    FATAL = 0x01,               /**< fatal system error */
    ERROR = 0x02,               /**< error with impact to correct functionality */
    WARN = 0x03,                /**< warning, correct behaviour could not be ensured */
    INFO = 0x04,                /**< informational */
    DEBUG = 0x05,               /**< debug  */
};

enum class CemTraceStatusType
{
    DEFAULT = -1,         /**< Default trace status */
    OFF = 0x00,           /**< Trace status: Off */
    ON = 0x01,            /**< Trace status: On */
};

namespace CemLog {
    template <int Value>
    struct require_at_compile_time
    {
        static constexpr const int value = Value;
    };

    constexpr int basename_index(const char * const path, const int index = 0, const int slash_index = -1)
    {
        return path[index] ?
            (path[index] == '/' ? basename_index(path, index + 1, index)
                : basename_index(path, index + 1, slash_index))
            : (slash_index + 1);
    }

    struct ContextStore;

    class CemLogPt
    {
    public:
        static const uint8_t DefaultContextId_{static_cast<uint8_t>(-1)};

    public:
        static CemLogPt& Instance();
        void Init(const std::string& appIdStr, const CemLogLevel defultLevel = CemLogLevel::WARN, const CemTraceStatusType defultTrace = CemTraceStatusType::OFF, 
            const size_t maxFmtSize = 256u, const std::string& description = __progname);
        void AddContext(const uint8_t contextId, const std::string& contextIdStr, const CemLogLevel level = CemLogLevel::WARN, 
            const CemTraceStatusType trace = CemTraceStatusType::OFF, const std::string& description = __progname);
        void Print(const uint8_t contextId, const CemLogLevel level, const char *format, ...);
        void Print(const uint8_t contextId, const CemLogLevel level, const std::stringstream& ss);

    private:
        CemLogPt();
        ~CemLogPt();
        CemLogPt(const CemLogPt&) = delete;
        CemLogPt(CemLogPt&&) = delete;
        CemLogPt& operator=(const CemLogPt&) = delete;
        CemLogPt& operator=(CemLogPt&&) = delete;

        CemLogLevel TryToGetLevelFromEnv(const std::string& contextIdStr, const CemLogLevel defaultLevel);

        bool isInited_{false};
        size_t maxFmtSize_{0u};
        std::unique_ptr<ContextStore> contextStore_;
    };

    class CemSsLog
    {
    public:
        CemSsLog(const CemLogLevel level, const uint8_t logContextId = CemLogPt::DefaultContextId_);
        ~CemSsLog();

        template <typename T>
        CemSsLog& operator << (const T& t)
        {
            bufStream_ << t;
            return *this;
        }

        template <typename T>
        CemSsLog& operator << (T&& t)
        {
            bufStream_ << t;
            return *this;
        }
            
    private:
        std::stringstream bufStream_;
        const uint8_t logContextId_{0u};
        const CemLogLevel level_{CemLogLevel::DEFAULT};
    };
}
#ifndef __SIGNAL_HANDLERS_H__
#define __SIGNAL_HANDLERS_H__

#include <unistd.h>
#include <signal.h>
#include <vector>
#include <exception>
#include <execinfo.h>
#include <cxxabi.h>
#include <dlfcn.h>
#include <iostream>
#include <sstream>
#include "logging_utils.h"

#define DUMP_EXCEPTION(exception_t, lines) do{\
    try{\
        lines\
    }catch(exception_t e){\
		RLOGE("catch %s execption: %s from the lines: ", #exception_t, e.what()); \
        RLOGE("call-stack dumped: \n%s", LogFatalException::BackTrace().c_str()); \
    }\
}while(0);\

class LogFatalException: public std::exception
{
private:
    std::string confess_;

public:
    LogFatalException(){
        confess_ = "@f@ LogFatal exception!";
    }

    LogFatalException(std::string& msg) : confess_(msg)
    {
        LOGPF("@f@ call-stack dumped:\n %s", BackTrace(8, 1).c_str());
    }

    ~LogFatalException(){}

    virtual const char* what() const noexcept {
        return confess_.c_str();
    }

    static std::string BackTrace(const int nMaxFrames = 32, const int skip = 1){
        void *callstack[nMaxFrames];
        char buf[1024*8];
        int nFrames = backtrace(callstack, nMaxFrames);
        char **symbols = backtrace_symbols(callstack, nFrames);

        std::ostringstream trace_buf;
        for (int i = skip; i < nFrames; i++) {
            Dl_info info;
            if (dladdr(callstack[i], &info)) {
                char *demangled = NULL;
                int status;
                demangled = abi::__cxa_demangle(info.dli_sname, NULL, 0, &status);
                snprintf(buf, sizeof(buf), "%-2d: %p\t%s\n",
                        i, callstack[i], status == 0 ? demangled : info.dli_sname);
                free(demangled);
            } else {
                snprintf(buf, sizeof(buf), "%-2d: %p\t%s\n", i, callstack[i], symbols[i]);
            }
            trace_buf << buf;
        }
        free(symbols);
        if (nFrames >= nMaxFrames)
            trace_buf << "[truncated]\n";
        return trace_buf.str();
	}
};


class SignalHandlers
{
public:
    SignalHandlers(){};
    ~SignalHandlers(){};

    static void BreakHandler(int sig)
    {
        RLOGW("@q@ pid %d catch break signal %d", getpid(), sig);
        break_flag_ = true;
    }

    static bool BreakByUser()
    {
        return break_flag_;
    }

    static void BackTraceHandler(int sig)
    {
        RLOGW("@q@ pid %d catch quit signal %d", getpid(), sig);
        RLOGW("@q@ call-stack dumped: %s", LogFatalException::BackTrace(8, 2).c_str());
#if __LOGGING_BACKEND__ == 3
        __shutdown_spdlog__();
#endif
        //use _exit(), exit() may cause re-enter problem
        _exit(sig);
    }

    static void SigSegvHandler(int sig, siginfo_t *si, void *arg)
    {
        RLOGE("@q@ pid %d catch segment fault @%p", getpid(), si->si_addr);
        RLOGE("@q@ call-stack dumped: %s", LogFatalException::BackTrace(16, 2).c_str());
#if __LOGGING_BACKEND__ == 3
        __shutdown_spdlog__();
#endif
        //use _exit(), exit() may cause re-enter problem
        _exit(sig);
    }

    static void CatchSignals()
    {
        signal(SIGINT, BackTraceHandler); //2
        signal(SIGABRT, BackTraceHandler);// 6
        signal(SIGTERM, BackTraceHandler);// 15

        /* catch segment fault */
        struct sigaction sa;
        sigemptyset(&sa.sa_mask);
        sa.sa_sigaction = SigSegvHandler;
        sa.sa_flags   = SA_SIGINFO;
        sigaction(SIGSEGV, &sa, NULL);
    }

    static void RegisterBackTraceSignals(const std::vector<int>& sigs)
    {
        for(int i=0; i<sigs.size(); i++)
        {
            signal(sigs[i], BackTraceHandler);
            RLOGW("register backtrace handler for signal %d", sigs[i]);
        }

        /* catch segment fault */
        struct sigaction sa;
        sigemptyset(&sa.sa_mask);
        sa.sa_sigaction = SigSegvHandler;
        sa.sa_flags   = SA_SIGINFO;
        sigaction(SIGSEGV, &sa, NULL);
    }

    static void RegisterBreakSignals(const int sig)
    {
        break_flag_ = false;
        signal(sig, BreakHandler);
        RLOGW("register break handler for signal %d", sig);
    }

private:
    static volatile bool break_flag_;

};

#endif //__SIGNAL_HANDLERS_H__
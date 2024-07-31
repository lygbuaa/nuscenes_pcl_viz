#include "logging_utils.h"
#include "signal_handlers.h"
#include "nuscenes_adapter.h"
#include "lsch128x1_encoder.h"

int main(int argc, char **argv)
{
    for(int i = 0; i < argc; i++){
        LOGPF("argv[%d] = %s\n", i, argv[i]);
    }

    /** argv[1]=config_file_path */
    if(argc < 2)
    {
        LOGPF("%s incomplete args!", argv[0]);
        return -1;
    }

#if __LOGGING_BACKEND__ == 3
    // __setup_spdlog__(argv[1], spdlog::level::info);
    __setup_spdlog__(argv[1], static_cast<spdlog::level::level_enum>(atoi(argv[4])));
#endif

    std::vector<int> sigs;
    sigs.push_back(SIGINT);
    sigs.push_back(SIGABRT);
    sigs.push_back(SIGTERM);
    SignalHandlers::RegisterBackTraceSignals(sigs);
    SignalHandlers::RegisterBreakSignals(SIGINT);

    auto ns_adapter = std::make_shared<NuscenesAdapter>();
    // ns_adapter -> LoadConfig(argv[1]);
    ns_adapter -> test();

    // uint32_t heartbeat = 0;
    // while(!SignalHandlers::BreakByUser())
    // {
    //     RLOGI("main heartbeat: %d", heartbeat++);
    //     sleep(2);
    // }

#if __LOGGING_BACKEND__ == 3
    __shutdown_spdlog__();
#endif
    return 0;
}
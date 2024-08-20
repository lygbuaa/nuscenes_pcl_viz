#include <rclcpp/rclcpp.hpp>
#include "logging_utils.h"
#include "signal_handlers.h"
#include "pcl_rviz_bridge.h"

int main(int argc, char **argv)
{
    for(int i = 0; i < argc; i++)
    {
        LOGPF("argv[%d] = %s\n", i, argv[i]);
    }
    /** argv[1]=log_file_path, argv[2]=json_file_path */
    if(argc < 2)
    {
        LOGPF("%s incomplete args!", argv[0]);
        return -1;
    }

    SignalHandlers::CatchSignals();
#if __LOGGING_BACKEND__ == 3
    __setup_spdlog__(argv[1], spdlog::level::info);
#endif
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto ppl_rviz = std::make_shared<PclRvizBridgeNode>();
    executor.add_node(ppl_rviz);

    ppl_rviz -> init(argv[2]);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
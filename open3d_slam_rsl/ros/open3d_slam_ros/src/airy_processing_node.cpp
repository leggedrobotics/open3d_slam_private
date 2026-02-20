#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/callback_queue.h>
#include <thread>
#include "open3d_slam_ros/AiryProcessorRos.hpp"

int main(int argc, char** argv) {
  using namespace airy_processor;

  // apt-get install -y libgoogle-glog-dev
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "airy_processing_node");
  ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

  // This is where the initial class is constructed and passed on.
  std::shared_ptr<AiryProcessorRos> airyProcessor = std::make_shared<AiryProcessorRos>(nh);

  airyProcessor->initCommonRosStuff();
  airyProcessor->subscribeCloud();

  const int N_THREADS = std::max(2u, std::thread::hardware_concurrency());
  ros::AsyncSpinner spinner(N_THREADS);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}

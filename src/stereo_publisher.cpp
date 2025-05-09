#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>
#include <sys/syscall.h> // for SYS_gettid
#include <unistd.h>

#include "camera_info_manager/camera_info_manager.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "BridgePublisherPi.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"

std::tuple<dai::Pipeline, int, int> createPipeline(
                bool withDepth, bool lrcheck, bool extended, bool subpixel,
                int confidence, int LRchecktresh, std::string resolution)
{
    std::cout << "IP: createPipeline()" << std::endl;

    std::cout << "FYI: withDepth: " << withDepth << "  lrcheck: " << lrcheck << "  extended: " << extended << "  subpixel: " << subpixel << std::endl;
    std::cout << "FYI: confidence: " << confidence << "  LRchecktresh: " << LRchecktresh << "  resolution: " << resolution << std::endl;

    dai::Pipeline pipeline;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

    // XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");

    if(withDepth) {
        xoutDepth->setStreamName("depth");
    } else {
        xoutDepth->setStreamName("disparity");
    }

    int width, height;
    if(resolution == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
        width = 1280;
        height = 720;
    } else if(resolution == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
        width = 640;
        height = 400;
    } else if(resolution == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
        width = 1280;
        height = 800;
    } else if(resolution == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
        width = 640;
        height = 480;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> monoResolution: %s", resolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    // MonoCamera
    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);

    // StereoDepth
    stereo->initialConfig.setConfidenceThreshold(confidence);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    stereo->rectifiedLeft.link(xoutLeft->input);
    stereo->rectifiedRight.link(xoutRight->input);

    if(withDepth) {
        stereo->depth.link(xoutDepth->input);
    } else {
        stereo->disparity.link(xoutDepth->input);
    }

    return std::make_tuple(pipeline, width, height);
}

static rclcpp::Node::SharedPtr node;

static dai::rosBridge::BridgePublisherPi<sensor_msgs::msg::Image, dai::ImgFrame>* leftPublish_ = nullptr;
static dai::rosBridge::BridgePublisherPi<sensor_msgs::msg::Image, dai::ImgFrame>* rightPublish_ = nullptr;
static dai::rosBridge::BridgePublisherPi<sensor_msgs::msg::Image, dai::ImgFrame>* depthPublish_ = nullptr;

static std::string tfPrefix, monoResolution_;
static bool lrcheck_, extended_, subpixel_;
static int confidence_, LRchecktresh_;
static int monoWidth, monoHeight;
static int fpsDivider_ = 1;

static int cnt_crashes = 0;

void runPublishers()
{
    dai::Pipeline pipeline;

    std::cout << "IP: runPublishers()" << std::endl;

    std::cout << "FYI: tfPrefix: " << tfPrefix << "  monoResolution_: " << monoResolution_ << std::endl;

    std::cout << "FYI: main() depth enabled" << std::endl;
    bool enableDepth = true;

    std::tie(pipeline, monoWidth, monoHeight) = createPipeline(enableDepth, lrcheck_, extended_, subpixel_, confidence_, LRchecktresh_, monoResolution_);
    dai::Device device(pipeline);
    auto leftQueue = device.getOutputQueue("left", 30, false);
    auto rightQueue = device.getOutputQueue("right", 30, false);
    std::shared_ptr<dai::DataOutputQueue> stereoQueue;
    stereoQueue = device.getOutputQueue("depth", 30, false);

    auto calibrationHandler = device.readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    std::cout << "FYI: boardName: " << boardName << std::endl;
    if(monoHeight > 480 && boardName == "OAK-D-LITE") {
        monoWidth = 640;
        monoHeight = 480;
    }

    dai::rosBridge::ImageConverter converter(tfPrefix + "_left_camera_optical_frame", true);
    auto leftCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, monoWidth, monoHeight);
    dai::rosBridge::BridgePublisherPi<sensor_msgs::msg::Image, dai::ImgFrame> leftPublish(
        leftQueue,
        node,
        std::string("left/image_rect"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &converter, std::placeholders::_1, std::placeholders::_2),
        30,
        leftCameraInfo,
        "left");

    //leftPublish.addPublisherCallback();
    leftPublish_ = &leftPublish;
    leftPublish.setFpsDivider(fpsDivider_);   
    leftPublish.startPublisherThread();

    dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
    auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);
    dai::rosBridge::BridgePublisherPi<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(
        rightQueue,
        node,
        std::string("right/image_rect"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rightconverter, std::placeholders::_1, std::placeholders::_2),
        30,
        rightCameraInfo,
        "right");

    //rightPublish.addPublisherCallback();
    rightPublish_ = &rightPublish;
    rightPublish.setFpsDivider(fpsDivider_);   
    rightPublish.startPublisherThread();

    dai::rosBridge::ImageConverter rightconverter_(tfPrefix + "_right_camera_optical_frame", true);
    auto rightCameraInfo_ = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);
    dai::rosBridge::BridgePublisherPi<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(
        stereoQueue,
        node,
        std::string("stereo/depth"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                    &rightconverter_,  // since the converter has the same frame name
                                    // and image type is also same we can reuse it
                    std::placeholders::_1,
                    std::placeholders::_2),
        30,
        rightCameraInfo_,
        "stereo");
    //depthPublish.addPublisherCallback();
    depthPublish_ = &depthPublish;
    depthPublish.setFpsDivider(fpsDivider_);   
    depthPublish.startPublisherThread();
}

void workerTask()
{
    pid_t id = syscall(SYS_gettid);
    std::cout << "IP: workerTask() - in thread ID: " << id << std::endl;
    std::cout << "IP: Initializing new publishers..." << std::endl;

    try {
        runPublishers(); // will block until it crashes
    } catch(...)
    {
        std::cerr << "workerTask: Caught an exception in runPublishers()" << std::endl;
    }

    cnt_crashes++;
    std::cout << "Error: Publishers crashed, recovering..." << std::endl;
    std::cout << "FYI:  cnt_crashes: " << cnt_crashes << std::endl;
}

void monitoringTask()
{
    pid_t id = syscall(SYS_gettid);
    std::cout << "IP: monitoringTask() - in thread ID: " << id << std::endl;

    while(rclcpp::ok()) {

        unsigned int last_received = 0;

        std::cout << "IP: Outer loop of monitoring thread " << id << std::endl;
        std::cout << "IP: starting worker thread..." << std::endl;

        std::thread workerThread;

        try {
            workerThread = std::thread(workerTask);

            sleep(3);

            while (rclcpp::ok()) {

                sleep(1);

                unsigned int received = depthPublish_->getReceivedCount();

                // \x1b[1;A  // (CSI 1 A) move cursor up a line.
                // \r     // (CR) move cursor to beginning of line.
                // \x1b[0;K  // (CSI 0 K) clear line from cursor to EOL.

                std::cout << "\x1b[1;A\rIP: Inner loop of monitoring thread, received: " << received << std::endl;

                if (received == last_received) {
                    std::cout << "Error: frozen pipeline " << std::endl;
                    break;
                }

                last_received = received;
            }
        } catch (...) {
            std::cerr << "monitoringTask: Caught an exception while starting or running the worker thread" << std::endl;
        }

        if (workerThread.joinable()) {
            workerThread.join();
        }
    }
}

int main(int argc, char** argv)
{

    std::cout << "IP: main()" << std::endl;

    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("stereo_node");

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("lrcheck", true);
    node->declare_parameter("extended", false);
    node->declare_parameter("subpixel", true);
    node->declare_parameter("confidence", 200);
    node->declare_parameter("LRchecktresh", 5);
    node->declare_parameter("monoResolution", "720p");
    node->declare_parameter("fpsDivider", 10);

    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("lrcheck", lrcheck_);
    node->get_parameter("extended", extended_);
    node->get_parameter("subpixel", subpixel_);
    node->get_parameter("confidence", confidence_);
    node->get_parameter("LRchecktresh", LRchecktresh_);
    node->get_parameter("monoResolution", monoResolution_);
    node->get_parameter("fpsDivider", fpsDivider_);

    std::cout << "IP: starting monitoring thread..." << std::endl;

    std::thread monitoringThread = std::thread(monitoringTask);

    std::cout << "IP: spinning node..." << std::endl;

    rclcpp::spin(node);

    std::cout << "OK: main() finished" << std::endl;

    if (monitoringThread.joinable())
        monitoringThread.join();

    return 0;
}


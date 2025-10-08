/*#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <MvCameraControl.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <std_msgs/msg/float32.hpp>

class HikvisionCameraNode : public rclcpp::Node
{
public:
    HikvisionCameraNode() : Node("hikvision_camera_node")
    {
        setupRealCamera();
    }

    ~HikvisionCameraNode()
    {
        // 清理工作
        if (camera_handle_) {
            MV_CC_StopGrabbing(camera_handle_);
            MV_CC_CloseDevice(camera_handle_);
            MV_CC_DestroyHandle(camera_handle_);
            camera_handle_ = nullptr;
            MV_CC_Finalize();
            RCLCPP_INFO(this->get_logger(), "MVS SDK Destroyed");
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr frame_rate_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_check_;
    rclcpp::TimerBase::SharedPtr frame_rate_timer_;
    void* camera_handle_ = nullptr;

    // 函数声明
    void setupRealCamera();
    void checkAndUpdateParams();
    void updateCameraParams(int exposure_time, int gain, int frame_rate, const std::string &image_format, int image_width, int image_height);
    void readAndPublishFrameRate();
    void readAndLogActualParameters();
    std::string getPixelFormatString(unsigned int pixel_format);
    void captureAndPublishImage();
    void processImageAndPublish(MV_FRAME_OUT &stImageInfo);
    void connectFirstAvailableCamera();
    void connectBySerialNumber(const std::string &serial_number);
    void attemptReconnect(const std::string& serial_number);
    void displayCameraInfo(MV_CC_DEVICE_INFO* pDeviceInfo);
    void queryCameraIntrinsics();
    void estimateIntrinsics();
    void setPixelFormat(const std::string &image_format);
    
    // 新增函数
    void debugEnumDevices();
    void printDeviceDetails(MV_CC_DEVICE_INFO* pDeviceInfo, int index);
};

// 新增调试函数
void HikvisionCameraNode::debugEnumDevices()
{
    RCLCPP_INFO(this->get_logger(), "=== 开始详细设备枚举调试 ===");
    
    // 测试不同的枚举模式
    const char* enum_modes[] = {
        "MV_GIGE_DEVICE | MV_USB_DEVICE",
        "MV_USB_DEVICE", 
        "MV_GIGE_DEVICE",
        "MV_ALL"
    };
    
    unsigned int mode_values[] = {
        MV_GIGE_DEVICE | MV_USB_DEVICE,
        MV_USB_DEVICE,
        MV_GIGE_DEVICE,
        (unsigned int)-1  // MV_ALL
    };
    
    for (int mode_idx = 0; mode_idx < 4; mode_idx++) {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        
        int nRet = MV_CC_EnumDevices(mode_values[mode_idx], &stDeviceList);
        
        RCLCPP_INFO(this->get_logger(), "模式: %s -> 返回码: 0x%x, 找到设备数: %d", 
                   enum_modes[mode_idx], nRet, stDeviceList.nDeviceNum);
        
        if (nRet == MV_OK && stDeviceList.nDeviceNum > 0) {
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
                printDeviceDetails(stDeviceList.pDeviceInfo[i], i);
            }
        }
        
        // 短暂延迟
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void HikvisionCameraNode::printDeviceDetails(MV_CC_DEVICE_INFO* pDeviceInfo, int index)
{
    if (!pDeviceInfo) return;
    
    std::string interface_type = "Unknown";
    std::string device_model = "Unknown";
    std::string device_serial = "Unknown";
    std::string vendor_name = "Unknown";
    
    if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
        interface_type = "GigE";
        device_model = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stGigEInfo.chModelName);
        device_serial = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber);
        vendor_name = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stGigEInfo.chManufacturerName);
    } else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
        interface_type = "USB3.0";
        device_model = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName);
        device_serial = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        vendor_name = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chManufacturerName);
        
        // USB设备额外信息
        RCLCPP_INFO(this->get_logger(), "  USB设备信息:");
        RCLCPP_INFO(this->get_logger(), "    - 设备GUID: %s", pDeviceInfo->SpecialInfo.stUsb3VInfo.chDeviceGUID);
        RCLCPP_INFO(this->get_logger(), "    - 设备版本: %s", pDeviceInfo->SpecialInfo.stUsb3VInfo.chDeviceVersion);
        // 移除不存在的 chInstanceId 字段
    }
    
    RCLCPP_INFO(this->get_logger(), "设备 %d:", index);
    RCLCPP_INFO(this->get_logger(), "  - 接口类型: %s", interface_type.c_str());
    RCLCPP_INFO(this->get_logger(), "  - 厂商: %s", vendor_name.c_str());
    RCLCPP_INFO(this->get_logger(), "  - 型号: %s", device_model.c_str());
    RCLCPP_INFO(this->get_logger(), "  - 序列号: %s", device_serial.c_str());
    
    // 显示设备能力标志（使用存在的字段）
    RCLCPP_INFO(this->get_logger(), "  - 传输层类型: 0x%x", pDeviceInfo->nTLayerType);
}

// 函数定义
void HikvisionCameraNode::setupRealCamera()
{
    RCLCPP_INFO(this->get_logger(), "=== REAL CAMERA MODE ===");
    
    // 初始化 SDK
    if (MV_CC_Initialize() != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera SDK");
        rclcpp::shutdown();
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "MVS SDK Initialized Successfully");
    }

    // 首先运行详细的设备枚举调试
    debugEnumDevices();

    // 创建图像发布器
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

    // 创建帧率发布器
    frame_rate_pub_ = this->create_publisher<std_msgs::msg::Float32>("camera/frame_rate", 10);

    // 参数声明 - 修改默认格式为BayerRG8
    this->declare_parameter<int>("exposure_time", 10000);
    this->declare_parameter<int>("gain", 10);
    this->declare_parameter<int>("frame_rate", 30);
    this->declare_parameter<std::string>("image_format", "BayerRG8");
    this->declare_parameter<std::string>("serial_number", "");
    // 添加分辨率参数
    this->declare_parameter<int>("image_width", 1920);
    this->declare_parameter<int>("image_height", 1080);

    // 获取序列号参数
    std::string serial_number;
    this->get_parameter("serial_number", serial_number);

    // 连接相机
    if (!serial_number.empty()) {
        connectBySerialNumber(serial_number);
    } else {
        connectFirstAvailableCamera();
    }

    // 如果相机连接成功，设置分辨率并启动定时器
    if (camera_handle_) {
        // 获取分辨率参数
        int image_width, image_height;
        this->get_parameter("image_width", image_width);
        this->get_parameter("image_height", image_height);

        RCLCPP_INFO(this->get_logger(), "Attempting to set resolution to %dx%d", image_width, image_height);

        // 先停止采集
        MV_CC_StopGrabbing(camera_handle_);
        
        // 1. 设置分辨率
        int ret1 = MV_CC_SetIntValue(camera_handle_, "Width", image_width);
        int ret2 = MV_CC_SetIntValue(camera_handle_, "Height", image_height);
        
        if (ret1 == MV_OK && ret2 == MV_OK) {
            RCLCPP_INFO(this->get_logger(), "✓ Resolution set to %dx%d", image_width, image_height);
        } else {
            RCLCPP_ERROR(this->get_logger(), "✗ Failed to set resolution: Width=0x%x, Height=0x%x", ret1, ret2);
        }
        
        // 2. 设置图像格式
        std::string image_format;
        this->get_parameter("image_format", image_format);
        setPixelFormat(image_format);
        
        // 3. 设置帧率
        int frame_rate;
        this->get_parameter("frame_rate", frame_rate);
        MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", (float)frame_rate);
        
        // 4. 设置曝光和增益
        int exposure_time, gain;
        this->get_parameter("exposure_time", exposure_time);
        this->get_parameter("gain", gain);
        MV_CC_SetFloatValue(camera_handle_, "ExposureTime", (float)exposure_time);
        MV_CC_SetFloatValue(camera_handle_, "Gain", (float)gain);
        
        // 5. 禁用所有图像处理以减少延迟
        MV_CC_SetBoolValue(camera_handle_, "GammaEnable", false);
        MV_CC_SetBoolValue(camera_handle_, "SharpnessEnable", false); 
        MV_CC_SetBoolValue(camera_handle_, "NoiseReductionEnable", false);
        MV_CC_SetBoolValue(camera_handle_, "ColorCorrectionEnable", false);
        
        // 6. 优化采集模式
        MV_CC_SetEnumValue(camera_handle_, "AcquisitionMode", 2); // Continuous
        MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0);     // Off
        
        // 7. 设置流控参数
        MV_CC_SetEnumValue(camera_handle_, "StreamBufferHandlingMode", 2); // NewestOnly
        MV_CC_SetIntValue(camera_handle_, "StreamBufferCount", 16); // 增加缓冲区
        
        // 重新开始采集
        MV_CC_StartGrabbing(camera_handle_);

        // 添加内参查询
        queryCameraIntrinsics();

        // 启动定时器进行图像采集与发布
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&HikvisionCameraNode::captureAndPublishImage, this)
        );

        // 定时检查参数变化 - 修改为只检查一次，避免重复设置
        timer_check_ = this->create_wall_timer(
            std::chrono::seconds(2), // 缩短检查间隔以便更快响应参数变化
            std::bind(&HikvisionCameraNode::checkAndUpdateParams, this)
        );

        // 帧率读取定时器
        frame_rate_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&HikvisionCameraNode::readAndPublishFrameRate, this)
        );
        
        // 初始读取并记录实际参数
        readAndLogActualParameters();
    } else {
        RCLCPP_ERROR(this->get_logger(), "No camera connected, cannot start image capture");
    }
}

void HikvisionCameraNode::checkAndUpdateParams()
{
    // 真实相机模式下的参数检查
    if (!camera_handle_) {
        return;
    }

    static int last_exposure_time = 0;
    static int last_gain = 0;
    static int last_frame_rate = 0;
    static std::string last_image_format = "";
    static int last_image_width = 0;
    static int last_image_height = 0;

    int exposure_time, gain, frame_rate, image_width, image_height;
    std::string image_format;

    this->get_parameter("exposure_time", exposure_time);
    this->get_parameter("gain", gain);
    this->get_parameter("frame_rate", frame_rate);
    this->get_parameter("image_format", image_format);
    this->get_parameter("image_width", image_width);
    this->get_parameter("image_height", image_height);

    // 只有当参数真正改变时才更新
    bool needs_update = false;
    if (exposure_time != last_exposure_time || 
        gain != last_gain || 
        frame_rate != last_frame_rate || 
        image_format != last_image_format ||
        image_width != last_image_width ||
        image_height != last_image_height) {
        
        needs_update = true;
        last_exposure_time = exposure_time;
        last_gain = gain;
        last_frame_rate = frame_rate;
        last_image_format = image_format;
        last_image_width = image_width;
        last_image_height = image_height;
        
        RCLCPP_INFO(this->get_logger(), "Parameters changed, updating camera...");
        RCLCPP_INFO(this->get_logger(), "  Exposure: %d, Gain: %d, FrameRate: %d, Format: %s, Resolution: %dx%d", 
                   exposure_time, gain, frame_rate, image_format.c_str(), image_width, image_height);
    }

    if (needs_update) {
        updateCameraParams(exposure_time, gain, frame_rate, image_format, image_width, image_height);
    }
}

void HikvisionCameraNode::updateCameraParams(int exposure_time, int gain, int frame_rate, 
                                           const std::string &image_format, int image_width, int image_height)
{
    if (!camera_handle_) {
        RCLCPP_ERROR(this->get_logger(), "Camera handle is null");
        return;
    }

    bool all_success = true;

    // 首先检查分辨率是否需要更改
    MVCC_INTVALUE current_width, current_height;
    bool resolution_changed = false;
    
    if (MV_CC_GetIntValue(camera_handle_, "Width", &current_width) == MV_OK && 
        current_width.nCurValue != image_width) {
        resolution_changed = true;
    }
    
    if (MV_CC_GetIntValue(camera_handle_, "Height", &current_height) == MV_OK && 
        current_height.nCurValue != image_height) {
        resolution_changed = true;
    }

    // 如果分辨率需要更改，先停止采集
    if (resolution_changed) {
        RCLCPP_INFO(this->get_logger(), "Resolution changed, stopping capture to apply new settings");
        MV_CC_StopGrabbing(camera_handle_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 设置新的分辨率
        int ret1 = MV_CC_SetIntValue(camera_handle_, "Width", image_width);
        int ret2 = MV_CC_SetIntValue(camera_handle_, "Height", image_height);
        
        if (ret1 == MV_OK && ret2 == MV_OK) {
            RCLCPP_INFO(this->get_logger(), "✓ Resolution updated to %dx%d", image_width, image_height);
        } else {
            RCLCPP_ERROR(this->get_logger(), "✗ Failed to set resolution: Width=0x%x, Height=0x%x", ret1, ret2);
            all_success = false;
        }
    }

    // 设置曝光时间
    if (MV_CC_SetFloatValue(camera_handle_, "ExposureTime", (float)exposure_time) != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set exposure time to %d", exposure_time);
        all_success = false;
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Exposure time set to %d μs", exposure_time);
    }

    // 设置增益
    if (MV_CC_SetFloatValue(camera_handle_, "Gain", (float)gain) != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set gain to %d", gain);
        all_success = false;
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Gain set to %d", gain);
    }

    // 设置帧率
    if (MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", (float)frame_rate) != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set frame rate to %d", frame_rate);
        all_success = false;
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Frame rate set to %d fps", frame_rate);
    }

    // 设置像素格式
    setPixelFormat(image_format);

    // 如果分辨率改变了，重新开始采集
    if (resolution_changed) {
        int start_ret = MV_CC_StartGrabbing(camera_handle_);
        if (start_ret == MV_OK) {
            RCLCPP_INFO(this->get_logger(), "Capture resumed after resolution change");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to resume capture: 0x%x", start_ret);
            all_success = false;
        }
    }

    if (all_success) {
        RCLCPP_INFO(this->get_logger(), "Camera parameters updated successfully");
        readAndLogActualParameters();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Some camera parameters failed to update");
    }
}

void HikvisionCameraNode::setPixelFormat(const std::string &image_format)
{
    if (!camera_handle_) return;
    
    // 先获取当前格式
    MVCC_ENUMVALUE current_format;
    if (MV_CC_GetEnumValue(camera_handle_, "PixelFormat", &current_format) == MV_OK) {
        std::string current_format_str = getPixelFormatString(current_format.nCurValue);
        RCLCPP_INFO(this->get_logger(), "Current pixel format before change: %s (0x%x)", 
                   current_format_str.c_str(), current_format.nCurValue);
    }
    
    unsigned int nPixelFormat = 0x01080009; // 默认BayerRG8
    std::string format_name = "BayerRG8";
    
    if (image_format == "Mono8") {
        nPixelFormat = PixelType_Gvsp_Mono8;
        format_name = "Mono8";
    } else if (image_format == "Mono16") {
        nPixelFormat = PixelType_Gvsp_Mono16;
        format_name = "Mono16";
    } else if (image_format == "RGB8") {
        nPixelFormat = PixelType_Gvsp_RGB8_Packed;
        format_name = "RGB8";
    } else if (image_format == "BGR8") {
        nPixelFormat = PixelType_Gvsp_BGR8_Packed;
        format_name = "BGR8";
    } else if (image_format == "BayerRG8") {
        nPixelFormat = 0x01080009;
        format_name = "BayerRG8";
    } else if (image_format == "BayerGR8") {
        nPixelFormat = 0x0108000a;
        format_name = "BayerGR8";
    } else if (image_format == "BayerGB8") {
        nPixelFormat = 0x0108000b;
        format_name = "BayerGB8";
    } else if (image_format == "BayerBG8") {
        nPixelFormat = 0x0108000c;
        format_name = "BayerBG8";
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported image format: %s, using default BayerRG8", image_format.c_str());
    }

    // 检查是否已经是目标格式
    MVCC_ENUMVALUE current_pixel_format;
    if (MV_CC_GetEnumValue(camera_handle_, "PixelFormat", &current_pixel_format) == MV_OK) {
        if (current_pixel_format.nCurValue == nPixelFormat) {
            RCLCPP_INFO(this->get_logger(), "Pixel format already set to %s, skipping", format_name.c_str());
            return;
        }
    }

    // 直接停止采集以更改像素格式（简化方法）
    MV_CC_StopGrabbing(camera_handle_);
    RCLCPP_INFO(this->get_logger(), "Stopped grabbing to change pixel format");
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 短暂延迟确保停止完成

    // 尝试设置像素格式
    int ret = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", nPixelFormat);
    if (ret != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set pixel format to %s (error: 0x%x)", 
                    format_name.c_str(), ret);
        
        // 显示支持的格式
        MVCC_ENUMVALUE pixel_format_info;
        if (MV_CC_GetEnumValue(camera_handle_, "PixelFormat", &pixel_format_info) == MV_OK) {
            RCLCPP_INFO(this->get_logger(), "Supported pixel formats (%d):", pixel_format_info.nSupportedNum);
            for (unsigned int i = 0; i < pixel_format_info.nSupportedNum; i++) {
                std::string supported_format_name = getPixelFormatString(pixel_format_info.nSupportValue[i]);
                RCLCPP_INFO(this->get_logger(), "  - 0x%x: %s", pixel_format_info.nSupportValue[i], supported_format_name.c_str());
            }
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "✓ Successfully set pixel format to %s", format_name.c_str());
        
        // 验证设置结果
        MVCC_ENUMVALUE verified_format;
        if (MV_CC_GetEnumValue(camera_handle_, "PixelFormat", &verified_format) == MV_OK) {
            std::string verified_format_str = getPixelFormatString(verified_format.nCurValue);
            RCLCPP_INFO(this->get_logger(), "Verified pixel format: %s (0x%x)", 
                       verified_format_str.c_str(), verified_format.nCurValue);
        }
    }

    // 重新开始采集
    int start_ret = MV_CC_StartGrabbing(camera_handle_);
    if (start_ret == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Resumed grabbing after pixel format change");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to resume grabbing: 0x%x", start_ret);
    }
}

void HikvisionCameraNode::queryCameraIntrinsics()
{
    RCLCPP_INFO(this->get_logger(), "=== QUERYING CAMERA INTRINSICS ===");
    
    if (!camera_handle_) {
        RCLCPP_ERROR(this->get_logger(), "Camera handle is null, cannot query intrinsics");
        return;
    }
    
    // 尝试查询相机标定参数
    MVCC_FLOATVALUE focal_length_x, focal_length_y;
    MVCC_FLOATVALUE principal_point_x, principal_point_y;
    
    // 焦距 fx, fy
    bool has_intrinsics = false;
    
    if (MV_CC_GetFloatValue(camera_handle_, "FocalLengthX", &focal_length_x) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Focal Length X: %.3f", focal_length_x.fCurValue);
        has_intrinsics = true;
    } else {
        RCLCPP_INFO(this->get_logger(), "Focal Length X: Not available in camera memory");
    }
    
    if (MV_CC_GetFloatValue(camera_handle_, "FocalLengthY", &focal_length_y) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Focal Length Y: %.3f", focal_length_y.fCurValue);
        has_intrinsics = true;
    } else {
        RCLCPP_INFO(this->get_logger(), "Focal Length Y: Not available in camera memory");
    }
    
    // 主点 cx, cy
    if (MV_CC_GetFloatValue(camera_handle_, "PrincipalPointX", &principal_point_x) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Principal Point X: %.3f", principal_point_x.fCurValue);
        has_intrinsics = true;
    } else {
        RCLCPP_INFO(this->get_logger(), "Principal Point X: Not available in camera memory");
    }
    
    if (MV_CC_GetFloatValue(camera_handle_, "PrincipalPointY", &principal_point_y) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Principal Point Y: %.3f", principal_point_y.fCurValue);
        has_intrinsics = true;
    } else {
        RCLCPP_INFO(this->get_logger(), "Principal Point Y: Not available in camera memory");
    }
    
    // 畸变参数
    const char* distortion_names[] = {
        "DistortionK1", "DistortionK2", "DistortionP1", 
        "DistortionP2", "DistortionK3"
    };
    
    for (int i = 0; i < 5; i++) {
        MVCC_FLOATVALUE distortion_param;
        if (MV_CC_GetFloatValue(camera_handle_, distortion_names[i], &distortion_param) == MV_OK) {
            RCLCPP_INFO(this->get_logger(), "%s: %.6f", distortion_names[i], distortion_param.fCurValue);
            has_intrinsics = true;
    } else {
            RCLCPP_INFO(this->get_logger(), "%s: Not available in camera memory", distortion_names[i]);
        }
    }
    
    // 如果相机没有存储内参，提供估计值
    if (!has_intrinsics) {
        RCLCPP_INFO(this->get_logger(), "No intrinsics stored in camera. Providing estimated values:");
        estimateIntrinsics();
    } else {
        RCLCPP_INFO(this->get_logger(), "Intrinsics retrieved from camera memory");
    }
}

void HikvisionCameraNode::estimateIntrinsics()
{
    if (!camera_handle_) return;
    
    // 获取当前分辨率
    MVCC_INTVALUE width_value, height_value;
    if (MV_CC_GetIntValue(camera_handle_, "Width", &width_value) == MV_OK &&
        MV_CC_GetIntValue(camera_handle_, "Height", &height_value) == MV_OK) {
        
        int width = width_value.nCurValue;
        int height = height_value.nCurValue;
        
        // 近似计算内参（基于常见传感器尺寸）
        double sensor_width_mm = 6.4;  // 典型1/1.8"传感器宽度
        double focal_length_mm = 8.0;  // 典型镜头焦距
        
        double fx = (focal_length_mm * width) / sensor_width_mm;
        double fy = (focal_length_mm * height) / sensor_width_mm;
        double cx = width / 2.0;
        double cy = height / 2.0;
        
        RCLCPP_INFO(this->get_logger(), "=== ESTIMATED INTRINSICS ===");
        RCLCPP_INFO(this->get_logger(), "Camera Matrix:");
        RCLCPP_INFO(this->get_logger(), "[%.1f, 0.0, %.1f]", fx, cx);
        RCLCPP_INFO(this->get_logger(), "[0.0, %.1f, %.1f]", fy, cy);
        RCLCPP_INFO(this->get_logger(), "[0.0, 0.0, 1.0]");
        RCLCPP_INFO(this->get_logger(), "Distortion (typical): [0.0, 0.0, 0.0, 0.0, 0.0]");
        RCLCPP_INFO(this->get_logger(), "Note: These are estimated values. For accurate calibration, use camera_calibration package.");
    }
}

void HikvisionCameraNode::readAndPublishFrameRate()
{
    if (!camera_handle_) {
        return;
    }

    MVCC_FLOATVALUE frame_rate_value;
    memset(&frame_rate_value, 0, sizeof(MVCC_FLOATVALUE));
    
    int nRet = MV_CC_GetFloatValue(camera_handle_, "ResultingFrameRate", &frame_rate_value);
    
    if (nRet == MV_OK) {
        float actual_frame_rate = frame_rate_value.fCurValue;
        
        auto frame_rate_msg = std_msgs::msg::Float32();
        frame_rate_msg.data = actual_frame_rate;
        frame_rate_pub_->publish(frame_rate_msg);
        
        int target_frame_rate;
        this->get_parameter("frame_rate", target_frame_rate);
        
        RCLCPP_INFO(this->get_logger(), 
                   "Frame Rate - Target: %d fps, Actual: %.2f fps", 
                   target_frame_rate, actual_frame_rate);
        
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to read actual frame rate: 0x%x", nRet);
    }
}

void HikvisionCameraNode::readAndLogActualParameters()
{
    if (!camera_handle_) return;

    // 添加分辨率显示
    MVCC_INTVALUE width_value;
    memset(&width_value, 0, sizeof(MVCC_INTVALUE));
    if (MV_CC_GetIntValue(camera_handle_, "Width", &width_value) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Image Width: %d", width_value.nCurValue);
    }

    MVCC_INTVALUE height_value;
    memset(&height_value, 0, sizeof(MVCC_INTVALUE));
    if (MV_CC_GetIntValue(camera_handle_, "Height", &height_value) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Image Height: %d", height_value.nCurValue);
    }

    MVCC_FLOATVALUE exposure_value;
    memset(&exposure_value, 0, sizeof(MVCC_FLOATVALUE));
    if (MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &exposure_value) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Actual Exposure Time: %.1f μs", exposure_value.fCurValue);
    }

    MVCC_FLOATVALUE gain_value;
    memset(&gain_value, 0, sizeof(MVCC_FLOATVALUE));
    if (MV_CC_GetFloatValue(camera_handle_, "Gain", &gain_value) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Actual Gain: %.1f", gain_value.fCurValue);
    }

    MVCC_FLOATVALUE frame_rate_value;
    memset(&frame_rate_value, 0, sizeof(MVCC_FLOATVALUE));
    if (MV_CC_GetFloatValue(camera_handle_, "ResultingFrameRate", &frame_rate_value) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Actual Frame Rate: %.2f fps", frame_rate_value.fCurValue);
    }

    MVCC_ENUMVALUE pixel_format_value;
    memset(&pixel_format_value, 0, sizeof(MVCC_ENUMVALUE));
    if (MV_CC_GetEnumValue(camera_handle_, "PixelFormat", &pixel_format_value) == MV_OK) {
        std::string pixel_format_str = getPixelFormatString(pixel_format_value.nCurValue);
        RCLCPP_INFO(this->get_logger(), "Actual Pixel Format: %s (0x%x)", 
                    pixel_format_str.c_str(), pixel_format_value.nCurValue);
    }
}

std::string HikvisionCameraNode::getPixelFormatString(unsigned int pixel_format)
{
    switch (pixel_format) {
        case PixelType_Gvsp_Mono8: return "Mono8";
        case PixelType_Gvsp_Mono16: return "Mono16";
        case PixelType_Gvsp_RGB8_Packed: return "RGB8";
        case PixelType_Gvsp_BGR8_Packed: return "BGR8";
        case 0x01080009: return "BayerRG8";
        case 0x0108000a: return "BayerGR8";
        case 0x0108000b: return "BayerGB8";
        case 0x0108000c: return "BayerBG8";
        default: 
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "Unknown(0x%x)", pixel_format);
            return buffer;
    }
}

void HikvisionCameraNode::captureAndPublishImage() 
{
    if (!camera_handle_) {
        RCLCPP_WARN(this->get_logger(), "Camera not connected");
        std::string serial_number;
        this->get_parameter("serial_number", serial_number);
        attemptReconnect(serial_number);
        return;
    }
    
    MV_FRAME_OUT stImageInfo;
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT));
    
    int nRet = MV_CC_GetImageBuffer(camera_handle_, &stImageInfo, 1000);
    if (MV_OK != nRet) {
        RCLCPP_WARN(this->get_logger(), "Failed to get frame: 0x%x", nRet);
        return;
    }

    processImageAndPublish(stImageInfo);
    MV_CC_FreeImageBuffer(camera_handle_, &stImageInfo);
}

void HikvisionCameraNode::processImageAndPublish(MV_FRAME_OUT &stImageInfo)
{
    cv::Mat image;
    std::string encoding;
    
    RCLCPP_DEBUG(this->get_logger(), "Processing image with pixel format: 0x%lx", 
                (unsigned long)stImageInfo.stFrameInfo.enPixelType);
    
    // 使用 unsigned int 进行比较，避免枚举警告
    unsigned int pixel_format = stImageInfo.stFrameInfo.enPixelType;
    
    switch (pixel_format) {
        case PixelType_Gvsp_BGR8_Packed:
            image = cv::Mat(stImageInfo.stFrameInfo.nHeight, 
                           stImageInfo.stFrameInfo.nWidth, CV_8UC3, stImageInfo.pBufAddr);
            encoding = "bgr8";
            break;
        case PixelType_Gvsp_Mono8:
            image = cv::Mat(stImageInfo.stFrameInfo.nHeight,
                           stImageInfo.stFrameInfo.nWidth, CV_8UC1, stImageInfo.pBufAddr);
            encoding = "mono8";
            break;
        case PixelType_Gvsp_Mono16:
            image = cv::Mat(stImageInfo.stFrameInfo.nHeight,
                           stImageInfo.stFrameInfo.nWidth, CV_16UC1, stImageInfo.pBufAddr);
            encoding = "mono16";
            break;
        case PixelType_Gvsp_RGB8_Packed:
            image = cv::Mat(stImageInfo.stFrameInfo.nHeight,
                           stImageInfo.stFrameInfo.nWidth, CV_8UC3, stImageInfo.pBufAddr);
            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
            encoding = "bgr8";
            break;
        case 0x01080009:  // BayerRG8
            RCLCPP_DEBUG(this->get_logger(), "Converting BayerRG8 to BGR8");
            image = cv::Mat(stImageInfo.stFrameInfo.nHeight,
                           stImageInfo.stFrameInfo.nWidth, CV_8UC1, stImageInfo.pBufAddr);
            cv::cvtColor(image, image, cv::COLOR_BayerRG2BGR);
            encoding = "bgr8";
            break;
        case 0x0108000a:  // BayerGR8
            RCLCPP_DEBUG(this->get_logger(), "Converting BayerGR8 to BGR8");
            image = cv::Mat(stImageInfo.stFrameInfo.nHeight,
                           stImageInfo.stFrameInfo.nWidth, CV_8UC1, stImageInfo.pBufAddr);
            cv::cvtColor(image, image, cv::COLOR_BayerGR2BGR);
            encoding = "bgr8";
            break;
        case 0x0108000b:  // BayerGB8
            RCLCPP_DEBUG(this->get_logger(), "Converting BayerGB8 to BGR8");
            image = cv::Mat(stImageInfo.stFrameInfo.nHeight,
                           stImageInfo.stFrameInfo.nWidth, CV_8UC1, stImageInfo.pBufAddr);
            cv::cvtColor(image, image, cv::COLOR_BayerGB2BGR);
            encoding = "bgr8";
            break;
        case 0x0108000c:  // BayerBG8
            RCLCPP_DEBUG(this->get_logger(), "Converting BayerBG8 to BGR8");
            image = cv::Mat(stImageInfo.stFrameInfo.nHeight,
                           stImageInfo.stFrameInfo.nWidth, CV_8UC1, stImageInfo.pBufAddr);
            cv::cvtColor(image, image, cv::COLOR_BayerBG2BGR);
            encoding = "bgr8";
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unsupported pixel format: 0x%lx", 
                       (unsigned long)stImageInfo.stFrameInfo.enPixelType);
            RCLCPP_INFO(this->get_logger(), "Image info - Width: %d, Height: %d", 
                       stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nHeight);
            return;
    }

    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera_frame";

    sensor_msgs::msg::Image::SharedPtr msg;
    try {
        msg = cv_bridge::CvImage(header, encoding, image).toImageMsg();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error converting image: %s", e.what());
        return;
    }

    image_pub_->publish(*msg);
    RCLCPP_DEBUG(this->get_logger(), "Image published successfully");
}

void HikvisionCameraNode::connectFirstAvailableCamera()
{
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    
    // 使用所有设备类型进行枚举
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        RCLCPP_ERROR(this->get_logger(), "Enum devices failed: 0x%x", nRet);
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Found %d devices", stDeviceList.nDeviceNum);
    
    if (stDeviceList.nDeviceNum == 0) {
        RCLCPP_ERROR(this->get_logger(), "No cameras detected");
        return;
    }

    // 显示所有找到的设备详细信息
    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
        printDeviceDetails(stDeviceList.pDeviceInfo[i], i);
    }

    // 尝试连接第一个设备
    MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[0];
    nRet = MV_CC_CreateHandle(&camera_handle_, pDeviceInfo);
    if (MV_OK == nRet) {
        nRet = MV_CC_OpenDevice(camera_handle_);
        if (MV_OK == nRet) {
            nRet = MV_CC_StartGrabbing(camera_handle_);
            if (MV_OK == nRet) {
                RCLCPP_INFO(this->get_logger(), "Connected to first available camera");
                displayCameraInfo(pDeviceInfo);
                return;
            }
            MV_CC_CloseDevice(camera_handle_);
        }
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
    }
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to camera");
}

void HikvisionCameraNode::connectBySerialNumber(const std::string &serial_number)
{
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enumerate devices");
        return;
    }

    bool found = false;
    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
        MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
        std::string device_serial;
        
        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            device_serial = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber);
        } else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
            device_serial = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        }

        if (device_serial == serial_number) {
            found = true;
            nRet = MV_CC_CreateHandle(&camera_handle_, pDeviceInfo);
            if (MV_OK == nRet) {
                nRet = MV_CC_OpenDevice(camera_handle_);
                if (MV_OK == nRet) {
                    nRet = MV_CC_StartGrabbing(camera_handle_);
                    if (MV_OK == nRet) {
                        RCLCPP_INFO(this->get_logger(), "Connected to camera: %s", serial_number.c_str());
                        displayCameraInfo(pDeviceInfo);
                        return;
                    }
                    MV_CC_CloseDevice(camera_handle_);
                }
                MV_CC_DestroyHandle(camera_handle_);
                camera_handle_ = nullptr;
            }
            break;
        }
    }
    
    if (!found) {
        RCLCPP_ERROR(this->get_logger(), "Camera with serial %s not found", serial_number.c_str());
    }
}

void HikvisionCameraNode::attemptReconnect(const std::string& serial_number)
{
    for (int i = 0; i < 3; i++) {
        RCLCPP_INFO(this->get_logger(), "Reconnection attempt %d/3", i + 1);
        
        if (camera_handle_) {
            MV_CC_StopGrabbing(camera_handle_);
            MV_CC_CloseDevice(camera_handle_);
            MV_CC_DestroyHandle(camera_handle_);
            camera_handle_ = nullptr;
        }
        
        if (!serial_number.empty()) {
            connectBySerialNumber(serial_number);
        } else {
            connectFirstAvailableCamera();
        }
        
        if (camera_handle_) {
            RCLCPP_INFO(this->get_logger(), "Reconnected successfully");
            return;
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    RCLCPP_ERROR(this->get_logger(), "Reconnection failed");
}

void HikvisionCameraNode::displayCameraInfo(MV_CC_DEVICE_INFO* pDeviceInfo)
{
    if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
        RCLCPP_INFO(this->get_logger(), "Model: %s", pDeviceInfo->SpecialInfo.stGigEInfo.chModelName);
        RCLCPP_INFO(this->get_logger(), "Vendor: %s", pDeviceInfo->SpecialInfo.stGigEInfo.chManufacturerName);
    } else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
        RCLCPP_INFO(this->get_logger(), "Model: %s", pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName);
        RCLCPP_INFO(this->get_logger(), "Vendor: %s", pDeviceInfo->SpecialInfo.stUsb3VInfo.chManufacturerName);
    }
    
    // 添加像素格式信息显示
    if (camera_handle_) {
        MVCC_ENUMVALUE pixel_format_value;
        memset(&pixel_format_value, 0, sizeof(MVCC_ENUMVALUE));
        if (MV_CC_GetEnumValue(camera_handle_, "PixelFormat", &pixel_format_value) == MV_OK) {
            RCLCPP_INFO(this->get_logger(), "Current Pixel Format: 0x%x", pixel_format_value.nCurValue);
            RCLCPP_INFO(this->get_logger(), "Supported Pixel Formats: %d entries", pixel_format_value.nSupportedNum);
        }
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HikvisionCameraNode>());
    rclcpp::shutdown();
    return 0;
}*/
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <MvCameraControl.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <std_msgs/msg/float32.hpp>

class HikvisionCameraNode : public rclcpp::Node
{
public:
    HikvisionCameraNode() : Node("hikvision_camera_node")
    {
        setupRealCamera();
    }

    ~HikvisionCameraNode()
    {
        // 清理工作
        if (camera_handle_) {
            MV_CC_StopGrabbing(camera_handle_);
            MV_CC_CloseDevice(camera_handle_);
            MV_CC_DestroyHandle(camera_handle_);
            camera_handle_ = nullptr;
            MV_CC_Finalize();
            RCLCPP_INFO(this->get_logger(), "MVS SDK Destroyed");
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr frame_rate_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_check_;
    rclcpp::TimerBase::SharedPtr frame_rate_timer_;
    void* camera_handle_ = nullptr;

    // 函数声明
    void setupRealCamera();
    void checkAndUpdateParams();
    void updateCameraParams(int exposure_time, int gain, int frame_rate, const std::string &image_format, int image_width, int image_height);
    void readAndPublishFrameRate();
    void readAndLogActualParameters();
    std::string getPixelFormatString(unsigned int pixel_format);
    void captureAndPublishImage();
    void processImageAndPublish(MV_FRAME_OUT &stImageInfo);
    bool connectFirstAvailableCamera();
    bool connectBySerialNumber(const std::string &serial_number);
    void attemptReconnect(const std::string& serial_number);
    void displayCameraInfo(MV_CC_DEVICE_INFO* pDeviceInfo);
    void queryCameraIntrinsics();
    void estimateIntrinsics();
    void setPixelFormat(const std::string &image_format);
    
    // 新增函数
    void debugEnumDevices();
    void printDeviceDetails(MV_CC_DEVICE_INFO* pDeviceInfo, int index);
    bool checkCameraConnection();
    void diagnoseFrameRateLimit();
    void optimizeForHighFrameRate();
    void applyParametersAfterReconnect();
    void restartCamera();
};

// 新增调试函数
void HikvisionCameraNode::debugEnumDevices()
{
    RCLCPP_INFO(this->get_logger(), "=== 开始详细设备枚举调试 ===");
    
    // 测试不同的枚举模式
    const char* enum_modes[] = {
        "MV_GIGE_DEVICE | MV_USB_DEVICE",
        "MV_USB_DEVICE", 
        "MV_GIGE_DEVICE",
        "MV_ALL"
    };
    
    unsigned int mode_values[] = {
        MV_GIGE_DEVICE | MV_USB_DEVICE,
        MV_USB_DEVICE,
        MV_GIGE_DEVICE,
        (unsigned int)-1  // MV_ALL
    };
    
    for (int mode_idx = 0; mode_idx < 4; mode_idx++) {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        
        int nRet = MV_CC_EnumDevices(mode_values[mode_idx], &stDeviceList);
        
        RCLCPP_INFO(this->get_logger(), "模式: %s -> 返回码: 0x%x, 找到设备数: %d", 
                   enum_modes[mode_idx], nRet, stDeviceList.nDeviceNum);
        
        if (nRet == MV_OK && stDeviceList.nDeviceNum > 0) {
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
                printDeviceDetails(stDeviceList.pDeviceInfo[i], i);
            }
        }
        
        // 短暂延迟
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void HikvisionCameraNode::printDeviceDetails(MV_CC_DEVICE_INFO* pDeviceInfo, int index)
{
    if (!pDeviceInfo) return;
    
    std::string interface_type = "Unknown";
    std::string device_model = "Unknown";
    std::string device_serial = "Unknown";
    std::string vendor_name = "Unknown";
    
    if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
        interface_type = "GigE";
        device_model = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stGigEInfo.chModelName);
        device_serial = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber);
        vendor_name = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stGigEInfo.chManufacturerName);
    } else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
        interface_type = "USB3.0";
        device_model = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName);
        device_serial = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        vendor_name = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chManufacturerName);
        
        // USB设备额外信息
        RCLCPP_INFO(this->get_logger(), "  USB设备信息:");
        RCLCPP_INFO(this->get_logger(), "    - 设备GUID: %s", pDeviceInfo->SpecialInfo.stUsb3VInfo.chDeviceGUID);
        RCLCPP_INFO(this->get_logger(), "    - 设备版本: %s", pDeviceInfo->SpecialInfo.stUsb3VInfo.chDeviceVersion);
    }
    
    RCLCPP_INFO(this->get_logger(), "设备 %d:", index);
    RCLCPP_INFO(this->get_logger(), "  - 接口类型: %s", interface_type.c_str());
    RCLCPP_INFO(this->get_logger(), "  - 厂商: %s", vendor_name.c_str());
    RCLCPP_INFO(this->get_logger(), "  - 型号: %s", device_model.c_str());
    RCLCPP_INFO(this->get_logger(), "  - 序列号: %s", device_serial.c_str());
    RCLCPP_INFO(this->get_logger(), "  - 传输层类型: 0x%x", pDeviceInfo->nTLayerType);
}

// 检查相机连接状态
bool HikvisionCameraNode::checkCameraConnection()
{
    if (!camera_handle_) {
        return false;
    }
    
    // 尝试读取一个简单的参数来检查连接状态
    MVCC_INTVALUE width_value;
    int ret = MV_CC_GetIntValue(camera_handle_, "Width", &width_value);
    
    if (ret != MV_OK) {
        RCLCPP_WARN(this->get_logger(), "Camera connection check failed: 0x%x", ret);
        return false;
    }
    
    return true;
}

// 诊断帧率限制
void HikvisionCameraNode::diagnoseFrameRateLimit()
{
    if (!camera_handle_) return;
    
    RCLCPP_INFO(this->get_logger(), "=== FRAME RATE LIMIT DIAGNOSIS ===");
    
    // 检查当前分辨率
    MVCC_INTVALUE width, height;
    MV_CC_GetIntValue(camera_handle_, "Width", &width);
    MV_CC_GetIntValue(camera_handle_, "Height", &height);
    
    // 检查帧率控制模式
    MVCC_ENUMVALUE frame_rate_mode;
    if (MV_CC_GetEnumValue(camera_handle_, "AcquisitionFrameRateMode", &frame_rate_mode) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Frame Rate Mode: %d (0=Off, 1=On)", frame_rate_mode.nCurValue);
    }
    
    // 检查设置的帧率
    MVCC_FLOATVALUE set_frame_rate;
    if (MV_CC_GetFloatValue(camera_handle_, "AcquisitionFrameRate", &set_frame_rate) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Set Frame Rate: %.2f fps", set_frame_rate.fCurValue);
    }
    
    // 检查实际帧率
    MVCC_FLOATVALUE actual_frame_rate;
    if (MV_CC_GetFloatValue(camera_handle_, "ResultingFrameRate", &actual_frame_rate) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Actual Frame Rate: %.2f fps", actual_frame_rate.fCurValue);
    }
    
    // 检查曝光模式
    MVCC_ENUMVALUE exposure_mode;
    if (MV_CC_GetEnumValue(camera_handle_, "ExposureMode", &exposure_mode) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Exposure Mode: %d", exposure_mode.nCurValue);
    }
    
    // 检查曝光时间
    MVCC_FLOATVALUE exposure_time;
    if (MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &exposure_time) == MV_OK) {
        double min_frame_time_ms = exposure_time.fCurValue / 1000.0;
        double max_possible_fps = 1000.0 / min_frame_time_ms;
        RCLCPP_INFO(this->get_logger(), "Exposure Time: %.1f μs", exposure_time.fCurValue);
        RCLCPP_INFO(this->get_logger(), "Theoretical max FPS from exposure: %.2f", max_possible_fps);
    }
    
    RCLCPP_INFO(this->get_logger(), "Current resolution: %dx%d", width.nCurValue, height.nCurValue);
}

// 优化高帧率设置
void HikvisionCameraNode::optimizeForHighFrameRate()
{
    if (!camera_handle_) return;
    
    RCLCPP_INFO(this->get_logger(), "=== OPTIMIZING FOR HIGH FRAME RATE ===");
    
    // 1. 设置流控制为最新帧模式（丢弃旧帧）
    MV_CC_SetEnumValue(camera_handle_, "StreamBufferHandlingMode", 2); // NewestOnly
    
    // 2. 减少缓冲区数量以减少延迟
    MV_CC_SetIntValue(camera_handle_, "StreamBufferCount", 4);
    
    // 3. 禁用所有图像处理以减少数据量
    MV_CC_SetBoolValue(camera_handle_, "GammaEnable", false);
    MV_CC_SetBoolValue(camera_handle_, "SharpnessEnable", false);
    MV_CC_SetBoolValue(camera_handle_, "NoiseReductionEnable", false);
    MV_CC_SetBoolValue(camera_handle_, "ColorCorrectionEnable", false);
    
    // 4. 设置触发模式为连续采集
    MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0); // Off
    
    // 5. 尝试设置设备带宽限制（如果支持）
    MV_CC_SetIntValue(camera_handle_, "DeviceLinkThroughputLimit", 300000000); // 300 Mbps
    
    RCLCPP_INFO(this->get_logger(), "High frame rate optimization applied");
}

// 重连后应用参数
void HikvisionCameraNode::applyParametersAfterReconnect()
{
    if (!camera_handle_) return;
    
    RCLCPP_INFO(this->get_logger(), "Applying parameters after reconnection");
    
    // 获取当前参数
    int exposure_time, gain, frame_rate, image_width, image_height;
    std::string image_format;
    
    this->get_parameter("exposure_time", exposure_time);
    this->get_parameter("gain", gain);
    this->get_parameter("frame_rate", frame_rate);
    this->get_parameter("image_format", image_format);
    this->get_parameter("image_width", image_width);
    this->get_parameter("image_height", image_height);
    
    // 应用参数
    updateCameraParams(exposure_time, gain, frame_rate, image_format, image_width, image_height);
}

// 重启相机
void HikvisionCameraNode::restartCamera()
{
    RCLCPP_INFO(this->get_logger(), "=== ATTEMPTING CAMERA RESTART ===");
    
    if (camera_handle_) {
        MV_CC_StopGrabbing(camera_handle_);
        MV_CC_CloseDevice(camera_handle_);
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
    }
    
    // 短暂延迟
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 重新连接
    std::string serial_number;
    this->get_parameter("serial_number", serial_number);
    
    if (!serial_number.empty()) {
        connectBySerialNumber(serial_number);
    } else {
        connectFirstAvailableCamera();
    }
    
    if (camera_handle_) {
        // 应用参数
        applyParametersAfterReconnect();
        RCLCPP_INFO(this->get_logger(), "Camera restart successful");
    }
}

// 函数定义
void HikvisionCameraNode::setupRealCamera()
{
    RCLCPP_INFO(this->get_logger(), "=== REAL CAMERA MODE ===");
    
    // 初始化 SDK
    if (MV_CC_Initialize() != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera SDK");
        rclcpp::shutdown();
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "MVS SDK Initialized Successfully");
    }

    // 首先运行详细的设备枚举调试
    debugEnumDevices();

    // 创建图像发布器
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

    // 创建帧率发布器
    frame_rate_pub_ = this->create_publisher<std_msgs::msg::Float32>("camera/frame_rate", 10);

    // 参数声明
    this->declare_parameter<int>("exposure_time", 10000);
    this->declare_parameter<int>("gain", 10);
    this->declare_parameter<int>("frame_rate", 30);
    this->declare_parameter<std::string>("image_format", "BayerRG8");
    this->declare_parameter<std::string>("serial_number", "");
    this->declare_parameter<int>("image_width", 1920);
    this->declare_parameter<int>("image_height", 1080);

    // 获取序列号参数
    std::string serial_number;
    this->get_parameter("serial_number", serial_number);

    // 连接相机
    bool connected = false;
    if (!serial_number.empty()) {
        connected = connectBySerialNumber(serial_number);
    } else {
        connected = connectFirstAvailableCamera();
    }

    // 如果相机连接成功，设置分辨率并启动定时器
    if (connected && camera_handle_) {
        // 获取分辨率参数
        int image_width, image_height;
        this->get_parameter("image_width", image_width);
        this->get_parameter("image_height", image_height);

        RCLCPP_INFO(this->get_logger(), "Attempting to set resolution to %dx%d", image_width, image_height);

        // 先停止采集
        MV_CC_StopGrabbing(camera_handle_);
        
        // 1. 设置分辨率
        int ret1 = MV_CC_SetIntValue(camera_handle_, "Width", image_width);
        int ret2 = MV_CC_SetIntValue(camera_handle_, "Height", image_height);
        
        if (ret1 == MV_OK && ret2 == MV_OK) {
            RCLCPP_INFO(this->get_logger(), "✓ Resolution set to %dx%d", image_width, image_height);
        } else {
            RCLCPP_ERROR(this->get_logger(), "✗ Failed to set resolution: Width=0x%x, Height=0x%x", ret1, ret2);
        }
        
        // 2. 设置图像格式
        std::string image_format;
        this->get_parameter("image_format", image_format);
        setPixelFormat(image_format);
        
        // 3. 设置帧率
        int frame_rate;
        this->get_parameter("frame_rate", frame_rate);
        MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", (float)frame_rate);
        
        // 4. 设置曝光和增益
        int exposure_time, gain;
        this->get_parameter("exposure_time", exposure_time);
        this->get_parameter("gain", gain);
        MV_CC_SetFloatValue(camera_handle_, "ExposureTime", (float)exposure_time);
        MV_CC_SetFloatValue(camera_handle_, "Gain", (float)gain);
        
        // 5. 优化高帧率设置
        optimizeForHighFrameRate();
        
        // 6. 重新开始采集
        MV_CC_StartGrabbing(camera_handle_);

        // 添加内参查询
        queryCameraIntrinsics();

        // 启动定时器进行图像采集与发布
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&HikvisionCameraNode::captureAndPublishImage, this)
        );

        // 定时检查参数变化和连接状态
        timer_check_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&HikvisionCameraNode::checkAndUpdateParams, this)
        );

        // 帧率读取定时器
        frame_rate_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&HikvisionCameraNode::readAndPublishFrameRate, this)
        );
        
        // 初始读取并记录实际参数
        readAndLogActualParameters();
        diagnoseFrameRateLimit();
    } else {
        RCLCPP_ERROR(this->get_logger(), "No camera connected, cannot start image capture");
    }
}

void HikvisionCameraNode::checkAndUpdateParams()
{
    // 首先检查相机连接状态
    if (!checkCameraConnection()) {
        RCLCPP_WARN(this->get_logger(), "Camera connection lost, attempting to reconnect...");
        std::string serial_number;
        this->get_parameter("serial_number", serial_number);
        attemptReconnect(serial_number);
        return;
    }

    // 真实相机模式下的参数检查
    static int last_exposure_time = 0;
    static int last_gain = 0;
    static int last_frame_rate = 0;
    static std::string last_image_format = "";
    static int last_image_width = 0;
    static int last_image_height = 0;

    int exposure_time, gain, frame_rate, image_width, image_height;
    std::string image_format;

    this->get_parameter("exposure_time", exposure_time);
    this->get_parameter("gain", gain);
    this->get_parameter("frame_rate", frame_rate);
    this->get_parameter("image_format", image_format);
    this->get_parameter("image_width", image_width);
    this->get_parameter("image_height", image_height);

    // 只有当参数真正改变时才更新
    bool needs_update = false;
    if (exposure_time != last_exposure_time || 
        gain != last_gain || 
        frame_rate != last_frame_rate || 
        image_format != last_image_format ||
        image_width != last_image_width ||
        image_height != last_image_height) {
        
        needs_update = true;
        last_exposure_time = exposure_time;
        last_gain = gain;
        last_frame_rate = frame_rate;
        last_image_format = image_format;
        last_image_width = image_width;
        last_image_height = image_height;
        
        RCLCPP_INFO(this->get_logger(), "Parameters changed, updating camera...");
        RCLCPP_INFO(this->get_logger(), "  Exposure: %d, Gain: %d, FrameRate: %d, Format: %s, Resolution: %dx%d", 
                   exposure_time, gain, frame_rate, image_format.c_str(), image_width, image_height);
    }

    if (needs_update) {
        updateCameraParams(exposure_time, gain, frame_rate, image_format, image_width, image_height);
    }
}

void HikvisionCameraNode::updateCameraParams(int exposure_time, int gain, int frame_rate, 
                                           const std::string &image_format, int image_width, int image_height)
{
    if (!camera_handle_) {
        RCLCPP_ERROR(this->get_logger(), "Camera handle is null");
        return;
    }

    bool all_success = true;

    // 首先检查分辨率是否需要更改
    MVCC_INTVALUE current_width, current_height;
    bool resolution_changed = false;
    
    if (MV_CC_GetIntValue(camera_handle_, "Width", &current_width) == MV_OK && 
        static_cast<int>(current_width.nCurValue) != image_width) {  // 修复符号比较
        resolution_changed = true;
    }
    
    if (MV_CC_GetIntValue(camera_handle_, "Height", &current_height) == MV_OK && 
        static_cast<int>(current_height.nCurValue) != image_height) {  // 修复符号比较
        resolution_changed = true;
    }

    // 如果分辨率需要更改，先停止采集
    if (resolution_changed) {
        RCLCPP_INFO(this->get_logger(), "Resolution changed, stopping capture to apply new settings");
        MV_CC_StopGrabbing(camera_handle_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 设置新的分辨率
        int ret1 = MV_CC_SetIntValue(camera_handle_, "Width", image_width);
        int ret2 = MV_CC_SetIntValue(camera_handle_, "Height", image_height);
        
        if (ret1 == MV_OK && ret2 == MV_OK) {
            RCLCPP_INFO(this->get_logger(), "✓ Resolution updated to %dx%d", image_width, image_height);
        } else {
            RCLCPP_ERROR(this->get_logger(), "✗ Failed to set resolution: Width=0x%x, Height=0x%x", ret1, ret2);
            all_success = false;
        }
    }

    // 关键修改：强制启用帧率控制
    RCLCPP_INFO(this->get_logger(), "=== CONFIGURING FRAME RATE ===");
    
    // 1. 尝试设置帧率模式为On
    int frame_mode_ret = MV_CC_SetEnumValue(camera_handle_, "AcquisitionFrameRateMode", 1);
    if (frame_mode_ret != MV_OK) {
        RCLCPP_WARN(this->get_logger(), "AcquisitionFrameRateMode not available, error: 0x%x", frame_mode_ret);
    } else {
        RCLCPP_INFO(this->get_logger(), "AcquisitionFrameRateMode enabled successfully");
    }

    // 2. 设置目标帧率
    RCLCPP_INFO(this->get_logger(), "Setting target frame rate to %d fps", frame_rate);
    int frame_rate_ret = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", (float)frame_rate);
    if (frame_rate_ret != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set frame rate to %d: error 0x%x", frame_rate, frame_rate_ret);
        all_success = false;
    } else {
        RCLCPP_INFO(this->get_logger(), "Frame rate set command successful");
    }

    // 3. 禁用自动曝光
    MV_CC_SetEnumValue(camera_handle_, "ExposureAuto", 0); // 0=Off

    // 4. 设置曝光时间
    RCLCPP_INFO(this->get_logger(), "Setting exposure time to %d μs", exposure_time);
    if (MV_CC_SetFloatValue(camera_handle_, "ExposureTime", (float)exposure_time) != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set exposure time to %d", exposure_time);
        all_success = false;
    } else {
        RCLCPP_INFO(this->get_logger(), "Exposure time set to %d μs", exposure_time);
    }

    // 5. 设置增益
    RCLCPP_INFO(this->get_logger(), "Setting gain to %d", gain);
    if (MV_CC_SetFloatValue(camera_handle_, "Gain", (float)gain) != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set gain to %d", gain);
        all_success = false;
    } else {
        RCLCPP_INFO(this->get_logger(), "Gain set to %d", gain);
    }

    // 6. 设置像素格式
    setPixelFormat(image_format);

    // 如果分辨率改变了，重新开始采集
    if (resolution_changed) {
        int start_ret = MV_CC_StartGrabbing(camera_handle_);
        if (start_ret == MV_OK) {
            RCLCPP_INFO(this->get_logger(), "Capture resumed after resolution change");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to resume capture: 0x%x", start_ret);
            all_success = false;
        }
    }

    if (all_success) {
        RCLCPP_INFO(this->get_logger(), "Camera parameters updated successfully");
        readAndLogActualParameters();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Some camera parameters failed to update");
    }
}

void HikvisionCameraNode::setPixelFormat(const std::string &image_format)
{
    if (!camera_handle_) return;
    
    // 先获取当前格式
    MVCC_ENUMVALUE current_format;
    if (MV_CC_GetEnumValue(camera_handle_, "PixelFormat", &current_format) == MV_OK) {
        std::string current_format_str = getPixelFormatString(current_format.nCurValue);
        RCLCPP_INFO(this->get_logger(), "Current pixel format before change: %s (0x%x)", 
                   current_format_str.c_str(), current_format.nCurValue);
    }
    
    unsigned int nPixelFormat = 0x01080009; // 默认BayerRG8
    std::string format_name = "BayerRG8";
    
    if (image_format == "Mono8") {
        nPixelFormat = PixelType_Gvsp_Mono8;
        format_name = "Mono8";
    } else if (image_format == "Mono16") {
        nPixelFormat = PixelType_Gvsp_Mono16;
        format_name = "Mono16";
    } else if (image_format == "RGB8") {
        nPixelFormat = PixelType_Gvsp_RGB8_Packed;
        format_name = "RGB8";
    } else if (image_format == "BGR8") {
        nPixelFormat = PixelType_Gvsp_BGR8_Packed;
        format_name = "BGR8";
    } else if (image_format == "BayerRG8") {
        nPixelFormat = 0x01080009;
        format_name = "BayerRG8";
    } else if (image_format == "BayerGR8") {
        nPixelFormat = 0x0108000a;
        format_name = "BayerGR8";
    } else if (image_format == "BayerGB8") {
        nPixelFormat = 0x0108000b;
        format_name = "BayerGB8";
    } else if (image_format == "BayerBG8") {
        nPixelFormat = 0x0108000c;
        format_name = "BayerBG8";
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported image format: %s, using default BayerRG8", image_format.c_str());
    }

    // 检查是否已经是目标格式
    MVCC_ENUMVALUE current_pixel_format;
    if (MV_CC_GetEnumValue(camera_handle_, "PixelFormat", &current_pixel_format) == MV_OK) {
        if (current_pixel_format.nCurValue == nPixelFormat) {
            RCLCPP_INFO(this->get_logger(), "Pixel format already set to %s, skipping", format_name.c_str());
            return;
        }
    }

    // 直接停止采集以更改像素格式
    MV_CC_StopGrabbing(camera_handle_);
    RCLCPP_INFO(this->get_logger(), "Stopped grabbing to change pixel format");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 尝试设置像素格式
    int ret = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", nPixelFormat);
    if (ret != MV_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set pixel format to %s (error: 0x%x)", 
                    format_name.c_str(), ret);
    } else {
        RCLCPP_INFO(this->get_logger(), "✓ Successfully set pixel format to %s", format_name.c_str());
    }

    // 重新开始采集
    int start_ret = MV_CC_StartGrabbing(camera_handle_);
    if (start_ret == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Resumed grabbing after pixel format change");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to resume grabbing: 0x%x", start_ret);
    }
}

void HikvisionCameraNode::queryCameraIntrinsics()
{
    RCLCPP_INFO(this->get_logger(), "=== QUERYING CAMERA INTRINSICS ===");
    
    if (!camera_handle_) {
        RCLCPP_ERROR(this->get_logger(), "Camera handle is null, cannot query intrinsics");
        return;
    }
    
    // 尝试查询相机标定参数
    MVCC_FLOATVALUE focal_length_x, focal_length_y;
    MVCC_FLOATVALUE principal_point_x, principal_point_y;
    
    bool has_intrinsics = false;
    
    if (MV_CC_GetFloatValue(camera_handle_, "FocalLengthX", &focal_length_x) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Focal Length X: %.3f", focal_length_x.fCurValue);
        has_intrinsics = true;
    }
    
    if (MV_CC_GetFloatValue(camera_handle_, "FocalLengthY", &focal_length_y) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Focal Length Y: %.3f", focal_length_y.fCurValue);
        has_intrinsics = true;
    }
    
    if (MV_CC_GetFloatValue(camera_handle_, "PrincipalPointX", &principal_point_x) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Principal Point X: %.3f", principal_point_x.fCurValue);
        has_intrinsics = true;
    }
    
    if (MV_CC_GetFloatValue(camera_handle_, "PrincipalPointY", &principal_point_y) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Principal Point Y: %.3f", principal_point_y.fCurValue);
        has_intrinsics = true;
    }
    
    // 如果相机没有存储内参，提供估计值
    if (!has_intrinsics) {
        RCLCPP_INFO(this->get_logger(), "No intrinsics stored in camera. Providing estimated values:");
        estimateIntrinsics();
    } else {
        RCLCPP_INFO(this->get_logger(), "Intrinsics retrieved from camera memory");
    }
}

void HikvisionCameraNode::estimateIntrinsics()
{
    if (!camera_handle_) return;
    
    MVCC_INTVALUE width_value, height_value;
    if (MV_CC_GetIntValue(camera_handle_, "Width", &width_value) == MV_OK &&
        MV_CC_GetIntValue(camera_handle_, "Height", &height_value) == MV_OK) {
        
        int width = width_value.nCurValue;
        int height = height_value.nCurValue;
        
        double sensor_width_mm = 6.4;
        double focal_length_mm = 8.0;
        
        double fx = (focal_length_mm * width) / sensor_width_mm;
        double fy = (focal_length_mm * height) / sensor_width_mm;
        double cx = width / 2.0;
        double cy = height / 2.0;
        
        RCLCPP_INFO(this->get_logger(), "=== ESTIMATED INTRINSICS ===");
        RCLCPP_INFO(this->get_logger(), "Camera Matrix:");
        RCLCPP_INFO(this->get_logger(), "[%.1f, 0.0, %.1f]", fx, cx);
        RCLCPP_INFO(this->get_logger(), "[0.0, %.1f, %.1f]", fy, cy);
        RCLCPP_INFO(this->get_logger(), "[0.0, 0.0, 1.0]");
        RCLCPP_INFO(this->get_logger(), "Distortion (typical): [0.0, 0.0, 0.0, 0.0, 0.0]");
    }
}

void HikvisionCameraNode::readAndPublishFrameRate()
{
    if (!camera_handle_) {
        return;
    }

    MVCC_FLOATVALUE frame_rate_value;
    memset(&frame_rate_value, 0, sizeof(MVCC_FLOATVALUE));
    
    int nRet = MV_CC_GetFloatValue(camera_handle_, "ResultingFrameRate", &frame_rate_value);
    
    if (nRet == MV_OK) {
        float actual_frame_rate = frame_rate_value.fCurValue;
        
        auto frame_rate_msg = std_msgs::msg::Float32();
        frame_rate_msg.data = actual_frame_rate;
        frame_rate_pub_->publish(frame_rate_msg);
        
        int target_frame_rate;
        this->get_parameter("frame_rate", target_frame_rate);
        
        RCLCPP_INFO(this->get_logger(), 
                   "Frame Rate - Target: %d fps, Actual: %.2f fps", 
                   target_frame_rate, actual_frame_rate);
        
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to read actual frame rate: 0x%x", nRet);
    }
}

void HikvisionCameraNode::readAndLogActualParameters()
{
    if (!camera_handle_) return;

    MVCC_INTVALUE width_value;
    memset(&width_value, 0, sizeof(MVCC_INTVALUE));
    if (MV_CC_GetIntValue(camera_handle_, "Width", &width_value) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Image Width: %d", width_value.nCurValue);
    }

    MVCC_INTVALUE height_value;
    memset(&height_value, 0, sizeof(MVCC_INTVALUE));
    if (MV_CC_GetIntValue(camera_handle_, "Height", &height_value) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Image Height: %d", height_value.nCurValue);
    }

    MVCC_FLOATVALUE exposure_value;
    memset(&exposure_value, 0, sizeof(MVCC_FLOATVALUE));
    if (MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &exposure_value) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Actual Exposure Time: %.1f μs", exposure_value.fCurValue);
    }

    MVCC_FLOATVALUE gain_value;
    memset(&gain_value, 0, sizeof(MVCC_FLOATVALUE));
    if (MV_CC_GetFloatValue(camera_handle_, "Gain", &gain_value) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Actual Gain: %.1f", gain_value.fCurValue);
    }

    MVCC_FLOATVALUE frame_rate_value;
    memset(&frame_rate_value, 0, sizeof(MVCC_FLOATVALUE));
    if (MV_CC_GetFloatValue(camera_handle_, "ResultingFrameRate", &frame_rate_value) == MV_OK) {
        RCLCPP_INFO(this->get_logger(), "Actual Frame Rate: %.2f fps", frame_rate_value.fCurValue);
    }

    MVCC_ENUMVALUE pixel_format_value;
    memset(&pixel_format_value, 0, sizeof(MVCC_ENUMVALUE));
    if (MV_CC_GetEnumValue(camera_handle_, "PixelFormat", &pixel_format_value) == MV_OK) {
        std::string pixel_format_str = getPixelFormatString(pixel_format_value.nCurValue);
        RCLCPP_INFO(this->get_logger(), "Actual Pixel Format: %s (0x%x)", 
                    pixel_format_str.c_str(), pixel_format_value.nCurValue);
    }
}

std::string HikvisionCameraNode::getPixelFormatString(unsigned int pixel_format)
{
    switch (pixel_format) {
        case PixelType_Gvsp_Mono8: return "Mono8";
        case PixelType_Gvsp_Mono16: return "Mono16";
        case PixelType_Gvsp_RGB8_Packed: return "RGB8";
        case PixelType_Gvsp_BGR8_Packed: return "BGR8";
        case 0x01080009: return "BayerRG8";
        case 0x0108000a: return "BayerGR8";
        case 0x0108000b: return "BayerGB8";
        case 0x0108000c: return "BayerBG8";
        default: 
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "Unknown(0x%x)", pixel_format);
            return buffer;
    }
}

void HikvisionCameraNode::captureAndPublishImage() 
{
    if (!camera_handle_) {
        RCLCPP_WARN(this->get_logger(), "Camera not connected");
        std::string serial_number;
        this->get_parameter("serial_number", serial_number);
        attemptReconnect(serial_number);
        return;
    }
    
    MV_FRAME_OUT stImageInfo;
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT));
    
    int nRet = MV_CC_GetImageBuffer(camera_handle_, &stImageInfo, 500);
    if (MV_OK != nRet) {
        // 修复错误代码处理
        const char* error_msg = "Unknown error";
        if (nRet == static_cast<int>(0x80000007)) {
            error_msg = "MV_E_GC_UNKNOW - Unknown error or device communication issue";
        } else if (nRet == static_cast<int>(0x80000301)) {
            error_msg = "MV_E_GC_ERR_USER_DEFINED - Parameter conflict or user defined error";
        } else if (nRet == MV_E_CALLORDER) {
            error_msg = "Call order error";
        } else if (nRet == MV_E_RESOURCE) {
            error_msg = "Resource error";
        }
        
        RCLCPP_WARN(this->get_logger(), "Failed to get frame: 0x%x - %s", nRet, error_msg);
        
        // 如果是连续错误，尝试恢复
        static int error_count = 0;
        error_count++;
        if (error_count > 10) {
            RCLCPP_ERROR(this->get_logger(), "Too many consecutive errors, attempting to restart camera...");
            error_count = 0;
            std::string serial_number;
            this->get_parameter("serial_number", serial_number);
            attemptReconnect(serial_number);
        }
        return;
    }

    // 重置错误计数
    static int error_count = 0;
    error_count = 0;

    processImageAndPublish(stImageInfo);
    MV_CC_FreeImageBuffer(camera_handle_, &stImageInfo);
}

void HikvisionCameraNode::processImageAndPublish(MV_FRAME_OUT &stImageInfo)
{
    cv::Mat image;
    std::string encoding;
    
    unsigned int pixel_format = stImageInfo.stFrameInfo.enPixelType;
    
    switch (pixel_format) {
        case PixelType_Gvsp_BGR8_Packed:
            image = cv::Mat(stImageInfo.stFrameInfo.nHeight, 
                           stImageInfo.stFrameInfo.nWidth, CV_8UC3, stImageInfo.pBufAddr);
            encoding = "bgr8";
            break;
        case PixelType_Gvsp_Mono8:
            image = cv::Mat(stImageInfo.stFrameInfo.nHeight,
                           stImageInfo.stFrameInfo.nWidth, CV_8UC1, stImageInfo.pBufAddr);
            encoding = "mono8";
            break;
        case PixelType_Gvsp_Mono16:
            image = cv::Mat(stImageInfo.stFrameInfo.nHeight,
                           stImageInfo.stFrameInfo.nWidth, CV_16UC1, stImageInfo.pBufAddr);
            encoding = "mono16";
            break;
        case PixelType_Gvsp_RGB8_Packed:
            image = cv::Mat(stImageInfo.stFrameInfo.nHeight,
                           stImageInfo.stFrameInfo.nWidth, CV_8UC3, stImageInfo.pBufAddr);
            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
            encoding = "bgr8";
            break;
        case 0x01080009:  // BayerRG8
            image = cv::Mat(stImageInfo.stFrameInfo.nHeight,
                           stImageInfo.stFrameInfo.nWidth, CV_8UC1, stImageInfo.pBufAddr);
            cv::cvtColor(image, image, cv::COLOR_BayerRG2BGR);
            encoding = "bgr8";
            break;
        case 0x0108000a:  // BayerGR8
            image = cv::Mat(stImageInfo.stFrameInfo.nHeight,
                           stImageInfo.stFrameInfo.nWidth, CV_8UC1, stImageInfo.pBufAddr);
            cv::cvtColor(image, image, cv::COLOR_BayerGR2BGR);
            encoding = "bgr8";
            break;
        case 0x0108000b:  // BayerGB8
            image = cv::Mat(stImageInfo.stFrameInfo.nHeight,
                           stImageInfo.stFrameInfo.nWidth, CV_8UC1, stImageInfo.pBufAddr);
            cv::cvtColor(image, image, cv::COLOR_BayerGB2BGR);
            encoding = "bgr8";
            break;
        case 0x0108000c:  // BayerBG8
            image = cv::Mat(stImageInfo.stFrameInfo.nHeight,
                           stImageInfo.stFrameInfo.nWidth, CV_8UC1, stImageInfo.pBufAddr);
            cv::cvtColor(image, image, cv::COLOR_BayerBG2BGR);
            encoding = "bgr8";
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unsupported pixel format: 0x%lx", 
                       (unsigned long)stImageInfo.stFrameInfo.enPixelType);
            return;
    }

    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera_frame";

    sensor_msgs::msg::Image::SharedPtr msg;
    try {
        msg = cv_bridge::CvImage(header, encoding, image).toImageMsg();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error converting image: %s", e.what());
        return;
    }

    image_pub_->publish(*msg);
}

bool HikvisionCameraNode::connectFirstAvailableCamera()
{
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        RCLCPP_ERROR(this->get_logger(), "Enum devices failed: 0x%x", nRet);
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Found %d devices", stDeviceList.nDeviceNum);
    
    if (stDeviceList.nDeviceNum == 0) {
        RCLCPP_ERROR(this->get_logger(), "No cameras detected");
        return false;
    }

    // 显示所有找到的设备详细信息
    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
        printDeviceDetails(stDeviceList.pDeviceInfo[i], i);
    }

    // 尝试连接第一个设备
    MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[0];
    nRet = MV_CC_CreateHandle(&camera_handle_, pDeviceInfo);
    if (MV_OK == nRet) {
        nRet = MV_CC_OpenDevice(camera_handle_);
        if (MV_OK == nRet) {
            nRet = MV_CC_StartGrabbing(camera_handle_);
            if (MV_OK == nRet) {
                RCLCPP_INFO(this->get_logger(), "Connected to first available camera");
                displayCameraInfo(pDeviceInfo);
                return true;
            }
            MV_CC_CloseDevice(camera_handle_);
        }
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = nullptr;
    }
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to camera");
    return false;
}

bool HikvisionCameraNode::connectBySerialNumber(const std::string &serial_number)
{
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        RCLCPP_ERROR(this->get_logger(), "Failed to enumerate devices");
        return false;
    }

    bool found = false;
    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
        MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
        std::string device_serial;
        
        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            device_serial = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber);
        } else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
            device_serial = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        }

        if (device_serial == serial_number) {
            found = true;
            nRet = MV_CC_CreateHandle(&camera_handle_, pDeviceInfo);
            if (MV_OK == nRet) {
                nRet = MV_CC_OpenDevice(camera_handle_);
                if (MV_OK == nRet) {
                    nRet = MV_CC_StartGrabbing(camera_handle_);
                    if (MV_OK == nRet) {
                        RCLCPP_INFO(this->get_logger(), "Connected to camera: %s", serial_number.c_str());
                        displayCameraInfo(pDeviceInfo);
                        return true;
                    }
                    MV_CC_CloseDevice(camera_handle_);
                }
                MV_CC_DestroyHandle(camera_handle_);
                camera_handle_ = nullptr;
            }
            break;
        }
    }
    
    if (!found) {
        RCLCPP_ERROR(this->get_logger(), "Camera with serial %s not found", serial_number.c_str());
    }
    return false;
}

void HikvisionCameraNode::attemptReconnect(const std::string& serial_number)
{
    RCLCPP_INFO(this->get_logger(), "=== STARTING RECONNECTION PROCEDURE ===");
    
    for (int i = 0; i < 3; i++) {
        RCLCPP_INFO(this->get_logger(), "Reconnection attempt %d/3", i + 1);
        
        // 彻底清理
        if (camera_handle_) {
            RCLCPP_INFO(this->get_logger(), "Cleaning up existing camera handle");
            MV_CC_StopGrabbing(camera_handle_);
            MV_CC_CloseDevice(camera_handle_);
            MV_CC_DestroyHandle(camera_handle_);
            camera_handle_ = nullptr;
        }
        
        // 短暂延迟让设备重置
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 重新连接
        bool success = false;
        if (!serial_number.empty()) {
            success = connectBySerialNumber(serial_number);
        } else {
            success = connectFirstAvailableCamera();
        }
        
        if (success) {
            RCLCPP_INFO(this->get_logger(), "✓ Reconnected successfully on attempt %d", i + 1);
            
            // 重新应用参数
            applyParametersAfterReconnect();
            return;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    
    RCLCPP_ERROR(this->get_logger(), "✗ All reconnection attempts failed");
}

void HikvisionCameraNode::displayCameraInfo(MV_CC_DEVICE_INFO* pDeviceInfo)
{
    if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
        RCLCPP_INFO(this->get_logger(), "Model: %s", pDeviceInfo->SpecialInfo.stGigEInfo.chModelName);
        RCLCPP_INFO(this->get_logger(), "Vendor: %s", pDeviceInfo->SpecialInfo.stGigEInfo.chManufacturerName);
    } else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
        RCLCPP_INFO(this->get_logger(), "Model: %s", pDeviceInfo->SpecialInfo.stUsb3VInfo.chModelName);
        RCLCPP_INFO(this->get_logger(), "Vendor: %s", pDeviceInfo->SpecialInfo.stUsb3VInfo.chManufacturerName);
    }
    
    if (camera_handle_) {
        MVCC_ENUMVALUE pixel_format_value;
        memset(&pixel_format_value, 0, sizeof(MVCC_ENUMVALUE));
        if (MV_CC_GetEnumValue(camera_handle_, "PixelFormat", &pixel_format_value) == MV_OK) {
            RCLCPP_INFO(this->get_logger(), "Current Pixel Format: 0x%x", pixel_format_value.nCurValue);
        }
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HikvisionCameraNode>());
    rclcpp::shutdown();
    return 0;
}
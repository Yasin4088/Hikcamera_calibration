#include "Acquisition/CameraInput.h"
#include "Acquisition/AcquireFrame.h"

#include <MvCameraControl.h>
#include <MvErrorDefine.h>
#include <arpa/inet.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
void checkSdk(int result, const char* operation)
{
    if (result == MV_OK) return;
    std::ostringstream message;
    message << operation << " failed (MVS 0x" << std::hex
            << static_cast<unsigned int>(result) << ')';
    throw std::runtime_error(message.str());
}

// 清理流程继续释放剩余资源，错误写入日志，避免析构抛异常。
void logSdk(int result, const char* operation) noexcept
{
    if (result != MV_OK)
        std::fprintf(stderr, "[CameraInput] %s failed (MVS 0x%x)\n",
                     operation, static_cast<unsigned int>(result));
}

// 两台设备共享 SDK；第一台连接前初始化，最后一个句柄销毁后反初始化。
std::mutex sdkMutex;
std::size_t sdkUsers = 0;

void acquireSdk()
{
    std::lock_guard<std::mutex> lock(sdkMutex);
    if (sdkUsers == 0) checkSdk(MV_CC_Initialize(), "MV_CC_Initialize");
    ++sdkUsers;
}

void releaseSdk() noexcept
{
    std::lock_guard<std::mutex> lock(sdkMutex);
    if (--sdkUsers == 0) logSdk(MV_CC_Finalize(), "MV_CC_Finalize");
}

std::uint32_t parseIp(const std::string& text)
{
    in_addr address{};
    if (inet_pton(AF_INET, text.c_str(), &address) != 1 || address.s_addr == 0)
        throw std::invalid_argument("Camera requires a valid nonzero IPv4 address: " + text);
    // MVS 指定 IP 连接示例使用高字节在前的数值表示。
    return ntohl(address.s_addr);
}

std::optional<std::uint32_t> parseOptionalIp(const std::string& text)
{
    if (text.empty()) return std::nullopt;
    return parseIp(text);
}

std::string formatIp(std::uint32_t hostOrderIp)
{
    in_addr address{};
    address.s_addr = htonl(hostOrderIp);
    char text[INET_ADDRSTRLEN]{};
    if (inet_ntop(AF_INET, &address, text, sizeof(text)) == nullptr)
        return "invalid-ip";
    return text;
}

std::string fixedBufferToString(const unsigned char* buffer, std::size_t length)
{
    if (buffer == nullptr || length == 0) return {};
    const auto* begin = reinterpret_cast<const char*>(buffer);
    std::size_t used = 0;
    while (used < length && begin[used] != '\0') ++used;
    return std::string(begin, used);
}

std::string describeGigE(const MV_CC_DEVICE_INFO& device)
{
    if (device.nTLayerType != MV_GIGE_DEVICE) return "non-gige-device";
    const auto& gige = device.SpecialInfo.stGigEInfo;
    std::ostringstream stream;
    stream << "camera=" << formatIp(gige.nCurrentIp)
           << ", host_if=" << formatIp(gige.nNetExport)
           << ", model=" << fixedBufferToString(gige.chModelName, sizeof(gige.chModelName))
           << ", serial=" << fixedBufferToString(gige.chSerialNumber, sizeof(gige.chSerialNumber));
    return stream.str();
}

std::string describeUsb(const MV_CC_DEVICE_INFO& device)
{
    if (device.nTLayerType != MV_USB_DEVICE) return "non-usb-device";
    const auto& usb = device.SpecialInfo.stUsb3VInfo;
    std::ostringstream stream;
    stream << "usb model=" << fixedBufferToString(usb.chModelName, sizeof(usb.chModelName))
           << ", serial=" << fixedBufferToString(usb.chSerialNumber, sizeof(usb.chSerialNumber));
    return stream.str();
}

std::string describeDevice(const MV_CC_DEVICE_INFO& device)
{
    if (device.nTLayerType == MV_GIGE_DEVICE) return describeGigE(device);
    if (device.nTLayerType == MV_USB_DEVICE) return describeUsb(device);
    std::ostringstream stream;
    stream << "unsupported transport layer=" << device.nTLayerType;
    return stream.str();
}

const MV_CC_DEVICE_INFO* selectDevice(const std::optional<std::uint32_t>& requestedDeviceIp)
{
    MV_CC_DEVICE_INFO_LIST devices{};
    checkSdk(MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &devices), "MV_CC_EnumDevices");
    if (devices.nDeviceNum == 0)
        throw std::runtime_error("No camera discovered by MVS; check power/USB/network");

    const MV_CC_DEVICE_INFO* selected = nullptr;
    const MV_CC_DEVICE_INFO* fallbackUsb = nullptr;
    std::ostringstream discovered;
    for (unsigned int index = 0; index < devices.nDeviceNum; ++index) {
        const auto* info = devices.pDeviceInfo[index];
        if (info == nullptr) continue;
        if (discovered.tellp() > 0) discovered << " | ";
        discovered << describeDevice(*info);
        if (info->nTLayerType == MV_GIGE_DEVICE) {
            const auto& gige = info->SpecialInfo.stGigEInfo;
            if (requestedDeviceIp && gige.nCurrentIp != *requestedDeviceIp) continue;
            if (selected == nullptr) selected = info;
            continue;
        }
        if (!requestedDeviceIp && info->nTLayerType == MV_USB_DEVICE && fallbackUsb == nullptr)
            fallbackUsb = info;
    }
    if (selected == nullptr && !requestedDeviceIp) {
        selected = fallbackUsb;
    }

    if (selected == nullptr) {
        std::ostringstream message;
        if (requestedDeviceIp)
            message << "Requested camera " << formatIp(*requestedDeviceIp) << " is not in MVS enumeration.";
        else
            message << "MVS enumerated devices but none are usable.";
        message << " discovered: " << discovered.str();
        throw std::runtime_error(message.str());
    }
    return selected;
}

void setFloat(void* handle, const char* name, float value)
{
    MVCC_FLOATVALUE range{};
    checkSdk(MV_CC_GetFloatValue(handle, name, &range), name);
    if (value < range.fMin || value > range.fMax)
        throw std::invalid_argument(std::string(name) + " is outside the device range [" +
                                    std::to_string(range.fMin) + ", " +
                                    std::to_string(range.fMax) + "]");
    checkSdk(MV_CC_SetFloatValue(handle, name, value), name);
}

std::int64_t getInt(void* handle, const char* name)
{
    MVCC_INTVALUE_EX value{};
    checkSdk(MV_CC_GetIntValueEx(handle, name, &value), name);
    return value.nCurValue;
}

// 成功取得缓存后立即建立守卫；尺寸校验、内存分配或转换失败也会归还 SDK 缓存。
struct ImageBufferGuard {
    void* handle;
    MV_FRAME_OUT& frame;
    ~ImageBufferGuard() noexcept
    {
        logSdk(MV_CC_FreeImageBuffer(handle, &frame), "MV_CC_FreeImageBuffer");
    }
};
} // namespace

CameraDevice::CameraDevice(const CameraParams& params, CameraSide side)
    : params_(params), side_(side)
{
    const auto deviceIp = parseOptionalIp(params.deviceIp);
    const auto netIp = parseOptionalIp(params.netIp);
    if (netIp && !deviceIp)
        throw std::invalid_argument("netIp requires a non-empty deviceIp");
    if (deviceIp && netIp && *deviceIp == *netIp)
        throw std::invalid_argument("Camera device IP and host interface IP must differ");
    if (!std::isfinite(params.exposure) || params.exposure <= 0 || !std::isfinite(params.gain))
        throw std::invalid_argument("Camera exposure must be positive and gain must be finite");
    if (params.width <= 0 || params.height <= 0)
        throw std::invalid_argument("Camera calibration dimensions must be positive");
}

CameraDevice::~CameraDevice() noexcept { StopGrabbing(); }

void CameraDevice::FindCamera()
{
    acquireSdk();
    sdkInitialized_ = true;

    // net_ip 为空时自动从 MVS 枚举结果选择连接网口，避免单目标定手工区分左右网口。
    const auto requestedDeviceIp = parseOptionalIp(params_.deviceIp);
    const auto requestedNetIp = parseOptionalIp(params_.netIp);
    if (requestedNetIp) {
        MV_CC_DEVICE_INFO info{};
        info.nTLayerType = MV_GIGE_DEVICE;
        info.SpecialInfo.stGigEInfo.nCurrentIp = *requestedDeviceIp;
        info.SpecialInfo.stGigEInfo.nNetExport = *requestedNetIp;
        transportLayer_ = MV_GIGE_DEVICE;
        checkSdk(MV_CC_CreateHandle(&handle_, &info), "MV_CC_CreateHandle");
    } else {
        const auto* selected = selectDevice(requestedDeviceIp);
        transportLayer_ = selected->nTLayerType;
        if (transportLayer_ == MV_GIGE_DEVICE) {
            const auto& gige = selected->SpecialInfo.stGigEInfo;
            params_.deviceIp = formatIp(gige.nCurrentIp);
            params_.netIp = formatIp(gige.nNetExport);
        } else {
            params_.deviceIp.clear();
            params_.netIp.clear();
        }
        std::fprintf(stderr, "[CameraInput] auto-selected %s\n", describeDevice(*selected).c_str());
        checkSdk(MV_CC_CreateHandle(&handle_, selected), "MV_CC_CreateHandle");
    }
    checkSdk(MV_CC_OpenDevice(handle_), "MV_CC_OpenDevice");
    opened_ = true;
}

void CameraDevice::SetCamera()
{
    // 1. 探测网口包长；探测失败时沿用设备包长，并输出诊断信息。
    if (transportLayer_ == MV_GIGE_DEVICE) {
        const int packetSize = MV_CC_GetOptimalPacketSize(handle_);
        if (packetSize > 0)
            checkSdk(MV_CC_SetIntValueEx(handle_, "GevSCPSPacketSize", packetSize), "GevSCPSPacketSize");
        else
            std::fprintf(stderr, "[CameraInput] packet size probe failed for %s (0x%x); keeping device value\n",
                         params_.deviceIp.c_str(), static_cast<unsigned int>(packetSize));
    }

    // 2. 使用连续采集和手动曝光、增益，避免相机自动调整覆盖 YAML 参数。
    checkSdk(MV_CC_SetEnumValueByString(handle_, "AcquisitionMode", "Continuous"), "AcquisitionMode");
    checkSdk(MV_CC_SetEnumValueByString(handle_, "TriggerMode", "Off"), "TriggerMode");
    checkSdk(MV_CC_SetEnumValueByString(handle_, "ExposureAuto", "Off"), "ExposureAuto");
    checkSdk(MV_CC_SetEnumValueByString(handle_, "GainAuto", "Off"), "GainAuto");
    setFloat(handle_, "ExposureTime", params_.exposure);
    setFloat(handle_, "Gain", params_.gain);

    // 3. 保持标定时的图像尺寸和原点；首版不自动改变硬件 ROI 或标定内参。
    const auto width = getInt(handle_, "Width");
    const auto height = getInt(handle_, "Height");
    const auto offsetX = getInt(handle_, "OffsetX");
    const auto offsetY = getInt(handle_, "OffsetY");
    if (width != params_.width || height != params_.height || offsetX != 0 || offsetY != 0) {
        std::ostringstream message;
        message << "Camera dimensions/offset do not match full-image calibration"
                << " (camera: " << width << "x" << height << ", offset " << offsetX << "," << offsetY
                << "; config: " << params_.width << "x" << params_.height << ", offset 0,0)";
        throw std::runtime_error(message.str());
    }

    // 4. 限制 SDK 缓存，并在支持时优先取最新帧以减小延迟。
    checkSdk(MV_CC_SetImageNodeNum(handle_, 2), "MV_CC_SetImageNodeNum");
    const int strategyResult = MV_CC_SetGrabStrategy(handle_, MV_GrabStrategy_LatestImagesOnly);
    // Linux 普通 GigE 设备可能不支持切换策略；此时保留默认的 OneByOne 顺序取帧。
    // 只容忍明确的“不支持”，避免掩盖无效句柄、调用顺序等其他错误。
    if (static_cast<unsigned int>(strategyResult) == MV_E_SUPPORT)
        std::fprintf(stderr, "[CameraInput] latest-frame strategy unsupported for %s; keeping default OneByOne\n",
                     params_.deviceIp.c_str());
    else
        checkSdk(strategyResult, "MV_CC_SetGrabStrategy");
    // extraInfoDelay 尚无对应的图像时钟补偿约定，采集端不使用它修改时间戳。
}

void CameraDevice::StartGrabbing()
{
    if (grabbing_) throw std::logic_error("CameraDevice is already grabbing");
    try {
        FindCamera();
        SetCamera();
        checkSdk(MV_CC_StartGrabbing(handle_), "MV_CC_StartGrabbing");
        grabbing_ = true;
    } catch (...) {
        StopGrabbing();
        throw;
    }
}

std::optional<CameraFrame> CameraDevice::GrabImage(std::uint32_t timeoutMs)
{
    if (!grabbing_) throw std::logic_error("CameraDevice is not grabbing");
    if (timeoutMs == std::numeric_limits<std::uint32_t>::max())
        throw std::invalid_argument("Infinite camera timeout is not supported");

    // 1. 从 SDK 获取一帧；无数据超时可重试，断连等错误交给采集循环处理。
    MV_FRAME_OUT raw{};
    const int result = MV_CC_GetImageBuffer(handle_, &raw, timeoutMs);
    if (static_cast<unsigned int>(result) == MV_E_NODATA) {
        if (!MV_CC_IsDeviceConnected(handle_))
            throw std::runtime_error("Camera disconnected: " + params_.deviceIp);
        return std::nullopt;
    }
    checkSdk(result, "MV_CC_GetImageBuffer");
    ImageBufferGuard release{handle_, raw};
    const auto receivedAt = std::chrono::steady_clock::now();
    const auto& info = raw.stFrameInfo;
    const unsigned int width = info.nExtendWidth != 0 ? info.nExtendWidth : info.nWidth;
    const unsigned int height = info.nExtendHeight != 0 ? info.nExtendHeight : info.nHeight;
    const std::uint64_t sourceBytes = info.nFrameLenEx != 0 ? info.nFrameLenEx : info.nFrameLen;
    if (raw.pBufAddr == nullptr || sourceBytes == 0 || info.nLostPacket != 0 ||
        width != static_cast<unsigned int>(params_.width) ||
        height != static_cast<unsigned int>(params_.height) || info.nOffsetX != 0 || info.nOffsetY != 0)
        throw std::runtime_error("Invalid/incomplete frame or unexpected geometry: " + params_.deviceIp);

    // 2. 输出内存由 cv::Mat 独立持有，转换完成后即可归还原始 SDK 缓存。
    const auto outputBytes = static_cast<std::uint64_t>(width) * height * 3;
    if (sourceBytes > std::numeric_limits<unsigned int>::max() ||
        outputBytes > std::numeric_limits<unsigned int>::max())
        throw std::runtime_error("Camera frame exceeds MVS pixel conversion size limit");
    CameraFrame frame;
    frame.image.create(params_.height, params_.width, CV_8UC3);
    if (info.enPixelType == PixelType_Gvsp_BGR8_Packed) {
        if (sourceBytes < outputBytes) throw std::runtime_error("Truncated BGR camera frame");
        std::memcpy(frame.image.data, raw.pBufAddr, static_cast<std::size_t>(outputBytes));
    } else {
        // Bayer、RGB、Mono 等格式统一由 SDK 转成后续算法约定的 BGR。
        MV_CC_PIXEL_CONVERT_PARAM_EX convert{};
        convert.nWidth = width;
        convert.nHeight = height;
        convert.enSrcPixelType = info.enPixelType;
        convert.pSrcData = raw.pBufAddr;
        convert.nSrcDataLen = static_cast<unsigned int>(sourceBytes);
        convert.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
        convert.pDstBuffer = frame.image.data;
        convert.nDstBufferSize = static_cast<unsigned int>(outputBytes);
        checkSdk(MV_CC_ConvertPixelTypeEx(handle_, &convert), "MV_CC_ConvertPixelTypeEx");
        if (convert.nDstLen != outputBytes)
            throw std::runtime_error("MVS conversion returned an unexpected BGR size");
    }

    // TODO 后续看看能否接收曝光时间，使用曝光时间作为时间戳
    // 3. 两侧使用同一主机单调时钟；这是收帧时间，设备曝光同步需另行接入。
    frame.frame_id = info.nFrameNum;
    frame.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        receivedAt.time_since_epoch()).count();
    frame.camera_side = side_;
    return frame;
}

void CameraDevice::StopGrabbing() noexcept
{
    if (grabbing_) {
        logSdk(MV_CC_StopGrabbing(handle_), "MV_CC_StopGrabbing");
        grabbing_ = false;
    }
    CloseCamera();
}

void CameraDevice::CloseCamera() noexcept
{
    if (opened_) {
        logSdk(MV_CC_CloseDevice(handle_), "MV_CC_CloseDevice");
        opened_ = false;
    }
    if (handle_ != nullptr) {
        logSdk(MV_CC_DestroyHandle(handle_), "MV_CC_DestroyHandle");
        handle_ = nullptr;
    }
    if (sdkInitialized_) {
        releaseSdk();
        sdkInitialized_ = false;
    }
}

CameraInput::CameraInput(const CameraParams& leftParams, const CameraParams& rightParams,
                         std::size_t queueCapacity, std::uint32_t grabTimeoutMs)
    : leftCamera_(leftParams, CameraSide::Left), rightCamera_(rightParams, CameraSide::Right),
      queue_(std::make_unique<StereoFrameQueue>(queueCapacity)), grabTimeoutMs_(grabTimeoutMs)
{
    if (grabTimeoutMs == 0 || grabTimeoutMs == std::numeric_limits<std::uint32_t>::max())
        throw std::invalid_argument("Camera grab timeout must be positive and finite");
    if (parseIp(leftParams.deviceIp) == parseIp(rightParams.deviceIp) &&
        parseIp(leftParams.netIp) == parseIp(rightParams.netIp))
        throw std::invalid_argument("Left and right cameras must refer to different devices");
}

CameraInput::~CameraInput() noexcept { StopCamera(); }

void CameraInput::StartCamera()
{
    if (leftThread_.joinable() || rightThread_.joinable())
        throw std::logic_error("Call StopCamera before starting camera input again");
    // 原队列对象保持有效；重新启动前丢弃上一次采集留下的所有帧。
    queue_->clear();
    try {
        leftCamera_.StartGrabbing();
        rightCamera_.StartGrabbing();
        stopRequested_.store(false);
        leftThread_ = std::thread(&CameraInput::CaptureLoop, this, std::ref(leftCamera_));
        rightThread_ = std::thread(&CameraInput::CaptureLoop, this, std::ref(rightCamera_));
    } catch (...) {
        StopCamera();
        throw;
    }
}

void CameraInput::StopCamera() noexcept
{
    // 先让取图和入队结束，再停止 SDK，避免另一线程仍访问已销毁的句柄。
    stopRequested_.store(true);
    if (leftThread_.joinable()) leftThread_.join();
    if (rightThread_.joinable()) rightThread_.join();
    leftCamera_.StopGrabbing();
    rightCamera_.StopGrabbing();
    queue_->clear();
}

StereoFrameQueue& CameraInput::frameQueue() noexcept { return *queue_; }

void CameraInput::CaptureLoop(CameraDevice& device) noexcept
{
    try {
        while (!stopRequested_.load()) {
            auto frame = device.GrabImage(grabTimeoutMs_);
            if (!frame || stopRequested_.load()) continue;
            if (frame->camera_side == CameraSide::Left) queue_->pushLeft(*frame);
            else queue_->pushRight(*frame);
        }
    } catch (const std::exception& error) {
        stopRequested_.store(true);
        std::fprintf(stderr, "[CameraInput] capture stopped: %s\n", error.what());
    } catch (...) {
        stopRequested_.store(true);
        std::fprintf(stderr, "[CameraInput] capture stopped: unknown error\n");
    }
}

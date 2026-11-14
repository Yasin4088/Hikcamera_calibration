#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "MvCameraControl.h"

namespace {

struct CliArgs {
  std::string camera_params_path = "config/camera_params.yaml";
  std::string world_points_path = "config/world_points.yaml";
  std::string output_path = "config/out_matrix.yaml";
  int display_width = 1536;
  int display_height = 1125;
  int timeout_ms = 1000;
  bool check_only = false;
};

struct RuntimeCameraConfig {
  int camera_type = 1;  // 0=gige, 1=usb
  int device_index = 0;
  int gige_index = 0;
  std::string camera_ip;
  std::string pixel_format = "BayerBG8";
  double acquisition_frame_rate = 120.0;
  double exposure_time = 10000.0;
  double gain = 10.0;
  int trigger_mode = 0;
  int trigger_source = 2;
  int trigger_activation = 0;
};

struct IntrinsicImageSize {
  int width = 0;
  int height = 0;

  bool IsValid() const { return width > 0 && height > 0; }
};

struct WorldPoints {
  std::vector<std::string> labels;
  std::vector<cv::Point3f> points;
};

void PrintUsage() {
  std::cout << "Usage: ./extrinsic_calibrator [options]\n"
            << "Options:\n"
            << "  --camera-params <path>   camera config yaml path\n"
            << "  --world-points <path>    world points yaml path\n"
            << "  --output <path>          output yaml path\n"
            << "  --display-width <int>    display width\n"
            << "  --display-height <int>   display height\n"
            << "  --timeout-ms <int>       camera grab timeout(ms)\n"
            << "  --check-only             only validate config files\n";
}

CliArgs ParseArgs(int argc, char **argv) {
  CliArgs args;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    auto need_value = [&](const char *flag) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(std::string("Missing value for ") + flag);
      }
      return argv[++i];
    };

    if (key == "--camera-params") {
      args.camera_params_path = need_value("--camera-params");
    } else if (key == "--world-points") {
      args.world_points_path = need_value("--world-points");
    } else if (key == "--output") {
      args.output_path = need_value("--output");
    } else if (key == "--display-width") {
      args.display_width = std::stoi(need_value("--display-width"));
    } else if (key == "--display-height") {
      args.display_height = std::stoi(need_value("--display-height"));
    } else if (key == "--timeout-ms") {
      args.timeout_ms = std::stoi(need_value("--timeout-ms"));
    } else if (key == "--check-only") {
      args.check_only = true;
    } else if (key == "--help" || key == "-h") {
      PrintUsage();
      std::exit(0);
    } else {
      throw std::runtime_error("Unknown argument: " + key);
    }
  }
  return args;
}

template <typename T>
void ReadOrDefault(const cv::FileNode &node, const std::string &key, T *value) {
  if (!node.empty() && !node[key].empty()) {
    node[key] >> *value;
  }
}

uint32_t ParseIpToUint(const std::string &ip) {
  int a = 0;
  int b = 0;
  int c = 0;
  int d = 0;
  if (std::sscanf(ip.c_str(), "%d.%d.%d.%d", &a, &b, &c, &d) != 4) {
    throw std::runtime_error("Invalid IP format: " + ip);
  }
  auto in_range = [](int v) { return v >= 0 && v <= 255; };
  if (!in_range(a) || !in_range(b) || !in_range(c) || !in_range(d)) {
    throw std::runtime_error("IP octet out of range: " + ip);
  }
  return (static_cast<uint32_t>(a) << 24U) | (static_cast<uint32_t>(b) << 16U) |
         (static_cast<uint32_t>(c) << 8U) | static_cast<uint32_t>(d);
}

void LoadCameraConfig(const std::string &path, cv::Mat *camera_matrix, cv::Mat *dist_coeffs,
                      RuntimeCameraConfig *runtime_cfg, IntrinsicImageSize *intrinsic_size) {
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    throw std::runtime_error("Cannot open camera params file: " + path);
  }

  fs["camera_matrix"] >> *camera_matrix;
  fs["dist_coeffs"] >> *dist_coeffs;
  if (camera_matrix->empty() || dist_coeffs->empty()) {
    throw std::runtime_error("camera_matrix or dist_coeffs missing in: " + path);
  }
  if (intrinsic_size != nullptr) {
    cv::FileNode width_node = fs["image_width"];
    cv::FileNode height_node = fs["image_height"];
    if (!width_node.empty() && !height_node.empty()) {
      width_node >> intrinsic_size->width;
      height_node >> intrinsic_size->height;
    }
  }

  cv::FileNode hik_node = fs["hik_camera"];
  ReadOrDefault(hik_node, "camera_type", &runtime_cfg->camera_type);
  ReadOrDefault(hik_node, "device_index", &runtime_cfg->device_index);
  ReadOrDefault(hik_node, "gige_index", &runtime_cfg->gige_index);
  ReadOrDefault(hik_node, "camera_ip", &runtime_cfg->camera_ip);
  ReadOrDefault(hik_node, "pixel_format", &runtime_cfg->pixel_format);
  ReadOrDefault(hik_node, "acquisition_frame_rate", &runtime_cfg->acquisition_frame_rate);
  ReadOrDefault(hik_node, "exposure_time", &runtime_cfg->exposure_time);
  ReadOrDefault(hik_node, "gain", &runtime_cfg->gain);
  ReadOrDefault(hik_node, "trigger_mode", &runtime_cfg->trigger_mode);
  ReadOrDefault(hik_node, "trigger_source", &runtime_cfg->trigger_source);
  ReadOrDefault(hik_node, "trigger_activation", &runtime_cfg->trigger_activation);
}

std::string Trim(const std::string &s) {
  const auto first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
}

std::string ParseValue(const std::string &line) {
  const auto pos = line.find(':');
  if (pos == std::string::npos) {
    return "";
  }
  std::string value = Trim(line.substr(pos + 1));
  if (!value.empty() && (value.front() == '"' || value.front() == '\'')) {
    value.erase(value.begin());
  }
  if (!value.empty() && (value.back() == '"' || value.back() == '\'')) {
    value.pop_back();
  }
  return value;
}

std::string StripCommentSuffix(const std::string &text) {
  const auto pos = text.find('#');
  if (pos == std::string::npos) {
    return Trim(text);
  }
  return Trim(text.substr(0, pos));
}

WorldPoints LoadWorldPoints(const std::string &path) {
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    throw std::runtime_error("Cannot open world points file: " + path);
  }

  struct TempPoint {
    std::string name;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    bool has_x = false;
    bool has_y = false;
    bool has_z = false;
  };

  WorldPoints result;
  TempPoint current;
  bool in_point = false;
  int point_idx = 0;

  auto flush_current = [&]() {
    if (!in_point) {
      return;
    }
    if (!current.has_x || !current.has_y || !current.has_z) {
      throw std::runtime_error("world point missing x/y/z fields.");
    }
    std::string clean_name = StripCommentSuffix(current.name);
    if (clean_name.empty()) {
      clean_name = "point_" + std::to_string(point_idx);
    }
    result.labels.push_back(clean_name);
    result.points.emplace_back(current.x, current.y, current.z);
    ++point_idx;
    current = TempPoint{};
  };

  std::string raw_line;
  while (std::getline(ifs, raw_line)) {
    const std::string line = Trim(raw_line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    if (line == "points:") {
      continue;
    }
    if (line.rfind("- ", 0) == 0) {
      flush_current();
      in_point = true;
      const std::string body = Trim(line.substr(2));
      if (body.rfind("name:", 0) == 0) {
        current.name = ParseValue(body);
      }
      continue;
    }
    if (!in_point) {
      continue;
    }

    if (line.rfind("name:", 0) == 0) {
      current.name = ParseValue(line);
    } else if (line.rfind("x:", 0) == 0) {
      current.x = std::stof(ParseValue(line));
      current.has_x = true;
    } else if (line.rfind("y:", 0) == 0) {
      current.y = std::stof(ParseValue(line));
      current.has_y = true;
    } else if (line.rfind("z:", 0) == 0) {
      current.z = std::stof(ParseValue(line));
      current.has_z = true;
    }
  }
  flush_current();

  if (result.points.size() < 4) {
    throw std::runtime_error("At least 4 world points are required.");
  }
  return result;
}

int PromptPointCount(std::size_t max_points) {
  if (max_points < 4) {
    throw std::runtime_error("At least 4 world points are required.");
  }

  while (true) {
    std::cout << "Input calibration point count n (4-" << max_points << "): " << std::flush;
    std::string line;
    if (!std::getline(std::cin, line)) {
      throw std::runtime_error("Failed to read n from stdin.");
    }
    line = Trim(line);
    if (line.empty()) {
      std::cout << "n cannot be empty.\n";
      continue;
    }

    try {
      const int n = std::stoi(line);
      if (n < 4 || n > static_cast<int>(max_points)) {
        std::cout << "n must be in [4, " << max_points << "].\n";
        continue;
      }
      return n;
    } catch (const std::exception &) {
      std::cout << "Invalid integer input. Try again.\n";
    }
  }
}

WorldPoints TakeFirstNPoints(const WorldPoints &source, int n) {
  WorldPoints sliced;
  sliced.labels.assign(source.labels.begin(), source.labels.begin() + n);
  sliced.points.assign(source.points.begin(), source.points.begin() + n);
  return sliced;
}

class HikCameraCapture {
 public:
  HikCameraCapture() = default;
  ~HikCameraCapture() { Shutdown(); }

  void Initialize(const RuntimeCameraConfig &cfg) {
    int ret = MV_CC_Initialize();
    if (ret != MV_OK) {
      throw std::runtime_error("MV_CC_Initialize failed, code=" + std::to_string(ret));
    }
    initialized_ = true;

    MV_CC_DEVICE_INFO_LIST device_list;
    std::memset(&device_list, 0, sizeof(device_list));
    ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
    if (ret != MV_OK || device_list.nDeviceNum == 0) {
      throw std::runtime_error("No Hik camera found or enum failed, code=" + std::to_string(ret));
    }

    int selected = SelectDeviceIndex(cfg, device_list);
    if (selected < 0) {
      throw std::runtime_error("No suitable camera found for current config.");
    }

    const auto *dev_info = device_list.pDeviceInfo[selected];
    ret = MV_CC_CreateHandle(&handle_, dev_info);
    if (ret != MV_OK || handle_ == nullptr) {
      throw std::runtime_error("MV_CC_CreateHandle failed, code=" + std::to_string(ret));
    }

    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) {
      throw std::runtime_error("MV_CC_OpenDevice failed, code=" + std::to_string(ret));
    }

    if (dev_info->nTLayerType == MV_GIGE_DEVICE) {
      const int packet_size = MV_CC_GetOptimalPacketSize(handle_);
      if (packet_size > 0) {
        MV_CC_SetIntValue(handle_, "GevSCPSPacketSize", packet_size);
      }
    }

    ApplyConfig(cfg);
    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) {
      throw std::runtime_error("MV_CC_StartGrabbing failed, code=" + std::to_string(ret));
    }
    grabbing_ = true;
  }

  bool GrabFrameBgr(cv::Mat *frame, int timeout_ms) {
    MV_FRAME_OUT out;
    std::memset(&out, 0, sizeof(out));
    const int ret = MV_CC_GetImageBuffer(handle_, &out, timeout_ms);
    if (ret != MV_OK) {
      return false;
    }

    const unsigned int width = out.stFrameInfo.nWidth;
    const unsigned int height = out.stFrameInfo.nHeight;
    const std::size_t dst_size = static_cast<std::size_t>(width) * height * 3;
    if (convert_buffer_.size() < dst_size) {
      convert_buffer_.resize(dst_size);
    }

    MV_CC_PIXEL_CONVERT_PARAM convert_param;
    std::memset(&convert_param, 0, sizeof(convert_param));
    convert_param.nWidth = width;
    convert_param.nHeight = height;
    convert_param.pSrcData = out.pBufAddr;
    convert_param.nSrcDataLen = out.stFrameInfo.nFrameLen;
    convert_param.enSrcPixelType = out.stFrameInfo.enPixelType;
    convert_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    convert_param.pDstBuffer = convert_buffer_.data();
    convert_param.nDstBufferSize = static_cast<unsigned int>(convert_buffer_.size());

    const int convert_ret = MV_CC_ConvertPixelType(handle_, &convert_param);
    MV_CC_FreeImageBuffer(handle_, &out);
    if (convert_ret != MV_OK) {
      return false;
    }

    cv::Mat view(static_cast<int>(height), static_cast<int>(width), CV_8UC3, convert_buffer_.data());
    *frame = view.clone();
    return true;
  }

 private:
  int SelectDeviceIndex(const RuntimeCameraConfig &cfg, const MV_CC_DEVICE_INFO_LIST &device_list) {
    std::vector<int> usb_ids;
    std::vector<int> gige_ids;
    for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
      const auto *dev = device_list.pDeviceInfo[i];
      if (dev == nullptr) {
        continue;
      }
      if (dev->nTLayerType == MV_USB_DEVICE) {
        usb_ids.push_back(static_cast<int>(i));
      } else if (dev->nTLayerType == MV_GIGE_DEVICE) {
        gige_ids.push_back(static_cast<int>(i));
      }
    }

    if (cfg.camera_type == 1) {
      if (cfg.device_index < 0 || cfg.device_index >= static_cast<int>(usb_ids.size())) {
        throw std::runtime_error("USB camera index out of range.");
      }
      return usb_ids[cfg.device_index];
    }

    if (!cfg.camera_ip.empty()) {
      const uint32_t target_ip = ParseIpToUint(cfg.camera_ip);
      for (int idx : gige_ids) {
        const auto *dev = device_list.pDeviceInfo[idx];
        if (dev->SpecialInfo.stGigEInfo.nCurrentIp == target_ip) {
          return idx;
        }
      }
      throw std::runtime_error("Cannot find GigE camera with ip: " + cfg.camera_ip);
    }
    if (cfg.gige_index < 0 || cfg.gige_index >= static_cast<int>(gige_ids.size())) {
      throw std::runtime_error("GigE camera index out of range.");
    }
    return gige_ids[cfg.gige_index];
  }

  void ApplyConfig(const RuntimeCameraConfig &cfg) {
    MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
    MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", static_cast<float>(cfg.acquisition_frame_rate));
    MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(cfg.exposure_time));
    MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(cfg.gain));
    if (!cfg.pixel_format.empty()) {
      MV_CC_SetEnumValueByString(handle_, "PixelFormat", cfg.pixel_format.c_str());
    }

    MV_CC_SetEnumValue(handle_, "TriggerMode", static_cast<unsigned int>(cfg.trigger_mode));
    if (cfg.trigger_mode != 0) {
      MV_CC_SetEnumValue(handle_, "TriggerSource", static_cast<unsigned int>(cfg.trigger_source));
      MV_CC_SetEnumValue(handle_, "TriggerActivation",
                         static_cast<unsigned int>(cfg.trigger_activation));
    }
  }

  void Shutdown() {
    if (handle_ != nullptr) {
      if (grabbing_) {
        MV_CC_StopGrabbing(handle_);
      }
      MV_CC_CloseDevice(handle_);
      MV_CC_DestroyHandle(&handle_);
      handle_ = nullptr;
    }
    if (initialized_) {
      MV_CC_Finalize();
      initialized_ = false;
    }
    grabbing_ = false;
  }

  void *handle_ = nullptr;
  bool initialized_ = false;
  bool grabbing_ = false;
  std::vector<unsigned char> convert_buffer_;
};

struct ReprojectionStats {
  double mean = 0.0;
  double rms = 0.0;
  double max = 0.0;
};

ReprojectionStats ComputeReprojectionStats(const std::vector<cv::Point3f> &object_points,
                                             const std::vector<cv::Point2f> &image_points,
                                             const cv::Mat &rvec, const cv::Mat &tvec,
                                             const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs) {
  if (object_points.size() != image_points.size() || object_points.empty()) {
    throw std::runtime_error("Reprojection error requires matched non-empty point sets.");
  }
  std::vector<cv::Point2f> projected;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, projected);

  ReprojectionStats stats;
  double sum = 0.0;
  double sum_sq = 0.0;
  for (std::size_t i = 0; i < image_points.size(); ++i) {
    const cv::Point2f diff = image_points[i] - projected[i];
    const double err = std::sqrt(diff.x * diff.x + diff.y * diff.y);
    sum += err;
    sum_sq += err * err;
    stats.max = std::max(stats.max, err);
  }
  const double n = static_cast<double>(image_points.size());
  stats.mean = sum / n;
  stats.rms = std::sqrt(sum_sq / n);
  return stats;
}

class CalibrationUi {
 public:
  CalibrationUi(const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs, WorldPoints world_points,
                std::string output_path, int display_width, int display_height)
      : camera_matrix_(camera_matrix.clone()),
        dist_coeffs_(dist_coeffs.clone()),
        world_points_(std::move(world_points)),
        output_path_(std::move(output_path)),
        display_width_(display_width),
        display_height_(display_height) {
    cv::namedWindow("calibrate", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("ROI", cv::WINDOW_AUTOSIZE);
    cv::resizeWindow("ROI", 400, 400);
    cv::setMouseCallback("calibrate", &CalibrationUi::MouseThunk, this);
  }

  bool HandleFrame(const cv::Mat &frame) {
    original_ = frame.clone();
    const float sx = static_cast<float>(display_width_) / static_cast<float>(original_.cols);
    const float sy = static_cast<float>(display_height_) / static_cast<float>(original_.rows);
    display_scale_ = std::min({sx, sy, 1.0f});
    const int show_w = std::max(1, static_cast<int>(std::round(original_.cols * display_scale_)));
    const int show_h = std::max(1, static_cast<int>(std::round(original_.rows * display_scale_)));
    cv::resize(original_, display_, cv::Size(show_w, show_h));
    cursor_x_ = std::clamp(cursor_x_, 0, std::max(0, display_.cols - 1));
    cursor_y_ = std::clamp(cursor_y_, 0, std::max(0, display_.rows - 1));

    if (is_calibrating_) {
      const std::size_t picked = image_points_.size();
      const std::size_t total = world_points_.points.size();
      const std::string next_name =
          picked < world_points_.labels.size() ? world_points_.labels[picked] : "none";
      DrawText(display_, "picked: " + std::to_string(picked) + "/" + std::to_string(total), 0);
      DrawText(display_, "next point: " + next_name, 1);
      DrawText(display_, "click point, WASD fine tune, n confirm", 2);
      DrawText(display_, "r reset, Enter start, q quit", 3);
    } else {
      DrawText(display_, "press Enter to start calibration", 0);
      DrawText(display_, "q quit", 1);
    }

    DrawPickedPoints();

    if (adjust_mode_) {
      if (!HandleAdjustMode()) {
        return false;
      }
    }

    cv::imshow("calibrate", display_);
    const int key = cv::waitKey(10) & 0xFF;
    if (key == 'q' || key == 27) {
      return false;
    }
    if (key == 13) {
      is_calibrating_ = true;
    } else if (key == 'r') {
      image_points_.clear();
      is_calibrating_ = false;
      std::cout << "Calibration reset.\n";
    }

    if (image_points_.size() == world_points_.points.size()) {
      SolveAndSave();
      image_points_.clear();
      is_calibrating_ = false;
    }
    return true;
  }

 private:
  static void MouseThunk(int event, int x, int y, int flags, void *userdata) {
    auto *self = static_cast<CalibrationUi *>(userdata);
    if (self != nullptr) {
      self->OnMouse(event, x, y, flags);
    }
  }

  void OnMouse(int event, int x, int y, int /*flags*/) {
    if (!PointInsideDisplay(x, y)) {
      return;
    }
    cursor_x_ = x;
    cursor_y_ = y;
    ShowRoi(x, y);
    if (event == cv::EVENT_LBUTTONDOWN && is_calibrating_) {
      adjust_mode_ = true;
    }
  }

  void DrawText(cv::Mat &canvas, const std::string &text, int idx) {
    cv::putText(canvas, text, cv::Point(30, 50 + idx * 45), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
  }

  void ShowRoi(int x, int y) {
    constexpr int radius = 50;
    if (display_.empty()) {
      return;
    }
    if (x < radius || y < radius || x >= display_.cols - radius || y >= display_.rows - radius) {
      return;
    }

    cv::Mat roi = display_(cv::Rect(x - radius, y - radius, 2 * radius, 2 * radius)).clone();
    cv::Mat zoom;
    cv::resize(roi, zoom, cv::Size(400, 400));
    cv::line(zoom, cv::Point(200, 100), cv::Point(200, 300), cv::Scalar(0, 0, 255), 1);
    cv::line(zoom, cv::Point(100, 200), cv::Point(300, 200), cv::Scalar(0, 0, 255), 1);
    cv::imshow("ROI", zoom);
  }

  bool HandleAdjustMode() {
    while (adjust_mode_) {
      cv::Mat preview = display_.clone();
      cv::drawMarker(preview, cv::Point(cursor_x_, cursor_y_), cv::Scalar(0, 255, 255),
                     cv::MARKER_CROSS, 20, 2);
      cv::imshow("calibrate", preview);
      ShowRoi(cursor_x_, cursor_y_);

      const int key = cv::waitKey(20) & 0xFF;
      if (key == 'w') {
        cursor_y_ = std::max(50, cursor_y_ - 1);
      } else if (key == 's') {
        cursor_y_ = std::min(display_.rows - 51, cursor_y_ + 1);
      } else if (key == 'a') {
        cursor_x_ = std::max(50, cursor_x_ - 1);
      } else if (key == 'd') {
        cursor_x_ = std::min(display_.cols - 51, cursor_x_ + 1);
      } else if (key == 'n') {
        const float x0 = static_cast<float>(cursor_x_) / display_scale_;
        const float y0 = static_cast<float>(cursor_y_) / display_scale_;
        image_points_.emplace_back(x0, y0);
        std::cout << "saved point " << image_points_.size() << ": display=(" << cursor_x_ << ","
                  << cursor_y_ << ") original=(" << x0 << "," << y0 << ")\n";
        adjust_mode_ = false;
      } else if (key == 'q' || key == 27) {
        adjust_mode_ = false;
        return false;
      }
    }
    return true;
  }

  void DrawPickedPoints() {
    for (std::size_t i = 0; i < image_points_.size(); ++i) {
      const auto &pt = image_points_[i];
      const int px = static_cast<int>(std::round(pt.x * display_scale_));
      const int py = static_cast<int>(std::round(pt.y * display_scale_));
      cv::circle(display_, cv::Point(px, py), 6, cv::Scalar(0, 255, 0), -1);
      cv::putText(display_, std::to_string(i + 1), cv::Point(px + 10, py - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
  }

  void SolveAndSave() {
    cv::Mat rvec;
    cv::Mat tvec;
    const bool ok_init = cv::solvePnP(world_points_.points, image_points_, camera_matrix_, dist_coeffs_,
                                      rvec, tvec, false, cv::SOLVEPNP_EPNP);
    if (!ok_init || rvec.empty() || tvec.empty()) {
      throw std::runtime_error("solvePnP(EPNP) failed.");
    }

    const bool ok_refine = cv::solvePnP(world_points_.points, image_points_, camera_matrix_, dist_coeffs_,
                                        rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);
    if (!ok_refine || rvec.empty() || tvec.empty()) {
      throw std::runtime_error("solvePnP refinement (ITERATIVE) failed.");
    }

    const ReprojectionStats reproj = ComputeReprojectionStats(
        world_points_.points, image_points_, rvec, tvec, camera_matrix_, dist_coeffs_);

    cv::FileStorage fs(output_path_, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
      throw std::runtime_error("Cannot write output file: " + output_path_);
    }
    fs << "world_rvec" << rvec;
    fs << "world_tvec" << tvec;
    fs << "mean_error" << reproj.mean;
    fs << "rms_error" << reproj.rms;
    fs << "max_error" << reproj.max;
    fs.release();

    std::cout << "Calibration complete.\n";
    std::cout << "Reprojection error: mean=" << reproj.mean << " px, rms=" << reproj.rms
              << " px, max=" << reproj.max << " px\n";
    std::cout << "rvec:\n" << rvec << "\n";
    std::cout << "tvec:\n" << tvec << "\n";
    std::cout << "Saved to: " << output_path_ << "\n";
  }

  bool PointInsideDisplay(int x, int y) const {
    return x >= 0 && y >= 0 && x < display_.cols && y < display_.rows;
  }

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  WorldPoints world_points_;
  std::string output_path_;
  int display_width_ = 0;
  int display_height_ = 0;
  float display_scale_ = 1.0f;

  bool is_calibrating_ = false;
  bool adjust_mode_ = false;
  int cursor_x_ = 0;
  int cursor_y_ = 0;

  cv::Mat original_;
  cv::Mat display_;
  std::vector<cv::Point2f> image_points_;
};

}  // namespace

int main(int argc, char **argv) {
  try {
    const CliArgs args = ParseArgs(argc, argv);

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    RuntimeCameraConfig runtime_cfg;
    IntrinsicImageSize intrinsic_size;
    LoadCameraConfig(args.camera_params_path, &camera_matrix, &dist_coeffs, &runtime_cfg,
                     &intrinsic_size);
    const WorldPoints all_world_points = LoadWorldPoints(args.world_points_path);

    std::cout << "Loaded camera params from: " << args.camera_params_path << "\n";
    std::cout << "Loaded " << all_world_points.points.size() << " world points from: "
              << args.world_points_path << "\n";
    for (std::size_t i = 0; i < all_world_points.points.size(); ++i) {
      const auto &p = all_world_points.points[i];
      std::cout << "  " << (i + 1) << ". " << all_world_points.labels[i] << ": [" << p.x << ", " << p.y
                << ", " << p.z << "]\n";
    }

    if (args.check_only) {
      std::cout << "Config check passed.\n";
      return 0;
    }

    const int n = PromptPointCount(all_world_points.points.size());
    const WorldPoints world_points = TakeFirstNPoints(all_world_points, n);
    std::cout << "Using first " << n << " points for calibration.\n";

    HikCameraCapture camera;
    camera.Initialize(runtime_cfg);

    CalibrationUi ui(camera_matrix, dist_coeffs, world_points, args.output_path, args.display_width,
                     args.display_height);

    std::cout
        << "Controls: click window, Enter=start, WASD=fine tune, n=confirm, r=reset, q/ESC=quit.\n";

    while (true) {
      cv::Mat frame;
      if (!camera.GrabFrameBgr(&frame, args.timeout_ms)) {
        const int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) {
          break;
        }
        continue;
      }
      static bool frame_size_checked = false;
      if (!frame_size_checked) {
        frame_size_checked = true;
        if (intrinsic_size.IsValid()) {
          if (frame.cols != intrinsic_size.width || frame.rows != intrinsic_size.height) {
            throw std::runtime_error("Frame size (" + std::to_string(frame.cols) + "x" +
                                     std::to_string(frame.rows) +
                                     ") does not match intrinsic size (" +
                                     std::to_string(intrinsic_size.width) + "x" +
                                     std::to_string(intrinsic_size.height) + ").");
          }
          std::cout << "Frame size check passed: " << frame.cols << "x" << frame.rows << "\n";
        } else {
          std::cout << "[WARN] image_width/image_height missing in camera params; skipped size check.\n";
        }
      }
      if (!ui.HandleFrame(frame)) {
        break;
      }
    }

    cv::destroyAllWindows();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "[ERROR] " << e.what() << "\n";
    return 1;
  }
}

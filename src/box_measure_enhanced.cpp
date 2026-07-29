#include <libobsensor/ObSensor.hpp>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>

struct Options {
    std::string modelPath;
    std::string outputDir = "outputs_cpp";
    int colorWidth = 640;
    int colorHeight = 480;
    int depthWidth = 640;
    int depthHeight = 480;
    int fps = OB_FPS_ANY;
    cv::Rect roi;
    bool hasRoi = false;
    float groundDepthM = 0.0f;
    bool hasGroundDepth = false;
    float minDepthM = 0.15f;
    float maxDepthM = 1.2f;
    float minBoxHeightM = 0.04f;
    float maxBoxHeightM = 0.18f;
    int minAreaPx = 1200;
    float minFillRatio = 0.25f;
    int morphKernel = 9;
};

struct Intrinsics {
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    int width = 0;
    int height = 0;
};

struct Measurement {
    bool ok = false;
    std::string reason = "not measured";
    float lengthM = 0.0f;
    float widthM = 0.0f;
    float heightM = 0.0f;
    float volumeM3 = 0.0f;
    float angleDeg = 0.0f;
    float groundDepthM = 0.0f;
    float topDepthM = 0.0f;
    int areaPx = 0;
    float fillRatio = 0.0f;
    cv::Point2f centerPx;
    std::vector<cv::Point2f> boxPoints;
    cv::Mat mask;
    cv::Mat filteredDepthM;
    int validDepthPx = 0;
    int foregroundPx = 0;
    int contourCount = 0;
    double largestContourAreaPx = 0.0;
    float validMinM = 0.0f;
    float validMedianM = 0.0f;
    float validMaxM = 0.0f;
    float validP90M = 0.0f;
};

static void printUsage(const char *program) {
    std::cout
        << "Usage: " << program << " [options]\n"
        << "  --model <path>                 EnhancedDepthFilter model path\n"
        << "  --output <dir>                 Output directory when pressing s\n"
        << "  --roi x,y,w,h                  Detection ROI in aligned image pixels\n"
        << "  --ground-depth <meters>        Fixed ground/table depth\n"
        << "  --min-depth <meters>           Minimum valid depth\n"
        << "  --max-depth <meters>           Maximum valid depth\n"
        << "  --min-height <meters>          Minimum box height above ground\n"
        << "  --max-box-height <meters>      Maximum box height above ground\n"
        << "  --min-area <pixels>            Minimum contour area\n"
        << "  --color-width/--color-height   Color stream size, default 640x480\n"
        << "  --depth-width/--depth-height   Depth stream size, default 640x480\n"
        << "  --help                         Show this help\n";
}

static bool parseRect(const std::string &text, cv::Rect &rect) {
    std::stringstream ss(text);
    std::string item;
    std::vector<int> values;
    while(std::getline(ss, item, ',')) {
        try {
            values.push_back(std::stoi(item));
        }
        catch(...) {
            return false;
        }
    }
    if(values.size() != 4 || values[2] <= 0 || values[3] <= 0) {
        return false;
    }
    rect = cv::Rect(values[0], values[1], values[2], values[3]);
    return true;
}

static bool parseArgs(int argc, char **argv, Options &opts) {
    for(int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto needValue = [&](const std::string &name) -> std::string {
            if(i + 1 >= argc) {
                throw std::runtime_error(name + " requires a value");
            }
            return argv[++i];
        };

        try {
            if(arg == "--model" || arg == "-m") {
                opts.modelPath = needValue(arg);
            }
            else if(arg == "--output") {
                opts.outputDir = needValue(arg);
            }
            else if(arg == "--roi") {
                opts.hasRoi = parseRect(needValue(arg), opts.roi);
                if(!opts.hasRoi) {
                    throw std::runtime_error("--roi must be x,y,width,height");
                }
            }
            else if(arg == "--ground-depth") {
                opts.groundDepthM = std::stof(needValue(arg));
                opts.hasGroundDepth = true;
            }
            else if(arg == "--min-depth") {
                opts.minDepthM = std::stof(needValue(arg));
            }
            else if(arg == "--max-depth") {
                opts.maxDepthM = std::stof(needValue(arg));
            }
            else if(arg == "--min-height") {
                opts.minBoxHeightM = std::stof(needValue(arg));
            }
            else if(arg == "--max-box-height") {
                opts.maxBoxHeightM = std::stof(needValue(arg));
            }
            else if(arg == "--min-area") {
                opts.minAreaPx = std::stoi(needValue(arg));
            }
            else if(arg == "--color-width") {
                opts.colorWidth = std::stoi(needValue(arg));
            }
            else if(arg == "--color-height") {
                opts.colorHeight = std::stoi(needValue(arg));
            }
            else if(arg == "--depth-width") {
                opts.depthWidth = std::stoi(needValue(arg));
            }
            else if(arg == "--depth-height") {
                opts.depthHeight = std::stoi(needValue(arg));
            }
            else if(arg == "--fps") {
                opts.fps = std::stoi(needValue(arg));
            }
            else if(arg == "--help" || arg == "-h") {
                printUsage(argv[0]);
                return false;
            }
            else {
                throw std::runtime_error("unknown argument: " + arg);
            }
        }
        catch(const std::exception &e) {
            std::cerr << e.what() << std::endl;
            printUsage(argv[0]);
            return false;
        }
    }
    return true;
}

static cv::Rect clampRect(cv::Rect rect, const cv::Size &size) {
    cv::Rect imageRect(0, 0, size.width, size.height);
    return rect & imageRect;
}

static std::vector<float> collectValues(const cv::Mat &depthM, const cv::Mat &mask) {
    std::vector<float> values;
    values.reserve(static_cast<size_t>(cv::countNonZero(mask)));
    for(int y = 0; y < depthM.rows; ++y) {
        const float *depthRow = depthM.ptr<float>(y);
        const uint8_t *maskRow = mask.ptr<uint8_t>(y);
        for(int x = 0; x < depthM.cols; ++x) {
            if(maskRow[x]) {
                values.push_back(depthRow[x]);
            }
        }
    }
    return values;
}

static float percentile(std::vector<float> values, float p) {
    if(values.empty()) {
        return 0.0f;
    }
    p = std::max(0.0f, std::min(100.0f, p));
    const size_t idx = static_cast<size_t>(std::round((p / 100.0f) * static_cast<float>(values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + idx, values.end());
    return values[idx];
}

static cv::Mat validDepthMask(const cv::Mat &depthM, const Options &opts, const cv::Rect &roi) {
    cv::Mat valid = cv::Mat::zeros(depthM.size(), CV_8U);
    cv::Mat minMask;
    cv::Mat maxMask;
    cv::compare(depthM, opts.minDepthM, minMask, cv::CMP_GE);
    cv::compare(depthM, opts.maxDepthM, maxMask, cv::CMP_LE);
    cv::Mat rangeMask;
    cv::bitwise_and(minMask, maxMask, rangeMask);
    if(roi.area() > 0) {
        rangeMask(roi).copyTo(valid(roi));
    }
    else {
        valid = rangeMask;
    }
    return valid;
}

static float calibrateGroundDepth(const cv::Mat &depthM, const Options &opts) {
    cv::Rect roi = opts.hasRoi ? clampRect(opts.roi, depthM.size()) : cv::Rect(0, 0, depthM.cols, depthM.rows);
    cv::Mat valid = validDepthMask(depthM, opts, roi);
    auto values = collectValues(depthM, valid);
    return percentile(values, 90.0f);
}

static cv::Point3f deproject(const cv::Point2f &pixel, float depthM, const Intrinsics &intr) {
    return cv::Point3f(
        (pixel.x - intr.cx) * depthM / intr.fx,
        (pixel.y - intr.cy) * depthM / intr.fy,
        depthM);
}

static Measurement measureBox(const cv::Mat &depthM, const Intrinsics &intr, const Options &opts) {
    Measurement result;
    if(depthM.empty()) {
        result.reason = "empty depth";
        return result;
    }

    cv::Mat filtered;
    cv::medianBlur(depthM, filtered, 5);
    result.filteredDepthM = filtered;

    cv::Rect roi = opts.hasRoi ? clampRect(opts.roi, filtered.size()) : cv::Rect(0, 0, filtered.cols, filtered.rows);
    cv::Mat valid = validDepthMask(filtered, opts, roi);
    result.validDepthPx = cv::countNonZero(valid);
    if(result.validDepthPx < opts.minAreaPx) {
        result.reason = "not enough valid depth pixels";
        return result;
    }

    auto validValues = collectValues(filtered, valid);
    result.validMinM = percentile(validValues, 0.0f);
    result.validMedianM = percentile(validValues, 50.0f);
    result.validMaxM = percentile(validValues, 100.0f);
    result.validP90M = percentile(validValues, 90.0f);

    result.groundDepthM = opts.hasGroundDepth ? opts.groundDepthM : percentile(validValues, 90.0f);

    cv::Mat heightMap = result.groundDepthM - filtered;
    cv::Mat minHeightMask;
    cv::Mat maxHeightMask;
    cv::compare(heightMap, opts.minBoxHeightM, minHeightMask, cv::CMP_GE);
    cv::compare(heightMap, opts.maxBoxHeightM, maxHeightMask, cv::CMP_LE);
    cv::Mat mask;
    cv::bitwise_and(valid, minHeightMask, mask);
    cv::bitwise_and(mask, maxHeightMask, mask);

    int kernelSize = opts.morphKernel;
    if(kernelSize > 1) {
        if(kernelSize % 2 == 0) {
            ++kernelSize;
        }
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 2);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
    }
    result.mask = mask;
    result.foregroundPx = cv::countNonZero(mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    result.contourCount = static_cast<int>(contours.size());

    int bestIndex = -1;
    double bestArea = 0.0;
    for(size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        result.largestContourAreaPx = std::max(result.largestContourAreaPx, area);
        if(area >= opts.minAreaPx && area > bestArea) {
            bestArea = area;
            bestIndex = static_cast<int>(i);
        }
    }
    if(bestIndex < 0) {
        result.reason = "no raised box-like foreground found";
        return result;
    }

    cv::RotatedRect rect = cv::minAreaRect(contours[bestIndex]);
    float rectArea = std::max(1.0f, rect.size.width * rect.size.height);
    result.areaPx = static_cast<int>(std::round(bestArea));
    result.fillRatio = std::min(1.0f, static_cast<float>(bestArea / rectArea));
    if(result.fillRatio < opts.minFillRatio) {
        result.reason = "foreground fill ratio too low";
        return result;
    }

    cv::Mat objectMask = cv::Mat::zeros(mask.size(), CV_8U);
    cv::drawContours(objectMask, contours, bestIndex, cv::Scalar(255), cv::FILLED);
    cv::Mat positiveHeightMask;
    cv::compare(heightMap, 0.0f, positiveHeightMask, cv::CMP_GT);
    cv::Mat topMask;
    cv::bitwise_and(objectMask, valid, topMask);
    cv::bitwise_and(topMask, positiveHeightMask, topMask);
    auto topValues = collectValues(filtered, topMask);
    if(topValues.empty()) {
        result.reason = "no valid top depth inside contour";
        return result;
    }
    result.topDepthM = percentile(topValues, 50.0f);
    result.heightM = std::max(0.0f, result.groundDepthM - result.topDepthM);

    cv::Point2f pts[4];
    rect.points(pts);
    result.boxPoints.assign(pts, pts + 4);
    result.centerPx = rect.center;
    result.angleDeg = rect.angle;

    std::vector<cv::Point3f> world;
    for(const auto &pt: result.boxPoints) {
        world.push_back(deproject(pt, result.topDepthM, intr));
    }
    std::vector<float> sideLengths;
    for(size_t i = 0; i < world.size(); ++i) {
        const auto &a = world[i];
        const auto &b = world[(i + 1) % world.size()];
        sideLengths.push_back(cv::norm(a - b));
    }
    float sideA = (sideLengths[0] + sideLengths[2]) * 0.5f;
    float sideB = (sideLengths[1] + sideLengths[3]) * 0.5f;
    result.lengthM = std::max(sideA, sideB);
    result.widthM = std::min(sideA, sideB);
    result.volumeM3 = result.lengthM * result.widthM * result.heightM;
    result.ok = true;
    result.reason = "ok";
    return result;
}

static cv::Mat colorFrameToBgr(const std::shared_ptr<ob::ColorFrame> &frame) {
    const int width = static_cast<int>(frame->getWidth());
    const int height = static_cast<int>(frame->getHeight());
    cv::Mat rgb(height, width, CV_8UC3, frame->getData());
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    return bgr.clone();
}

static cv::Mat depthFrameToMeters(const std::shared_ptr<ob::DepthFrame> &frame) {
    const int width = static_cast<int>(frame->getWidth());
    const int height = static_cast<int>(frame->getHeight());
    const float scaleToM = frame->getValueScale() * 0.001f;
    cv::Mat raw(height, width, CV_16UC1, frame->getData());
    cv::Mat depthM;
    raw.convertTo(depthM, CV_32F, scaleToM);
    return depthM.clone();
}

static Intrinsics getIntrinsics(const std::shared_ptr<ob::VideoFrame> &frame) {
    auto profile = frame->getStreamProfile()->as<ob::VideoStreamProfile>();
    auto intr = profile->getIntrinsic();
    const int width = static_cast<int>(frame->getWidth());
    const int height = static_cast<int>(frame->getHeight());
    float sx = intr.width > 0 ? static_cast<float>(width) / static_cast<float>(intr.width) : 1.0f;
    float sy = intr.height > 0 ? static_cast<float>(height) / static_cast<float>(intr.height) : 1.0f;
    return Intrinsics{intr.fx * sx, intr.fy * sy, intr.cx * sx, intr.cy * sy, width, height};
}

static cv::Mat colorizeDepth(const cv::Mat &depthM, const Options &opts) {
    cv::Mat clipped = depthM.clone();
    cv::Mat invalid = (clipped < opts.minDepthM) | (clipped > opts.maxDepthM);
    clipped = (clipped - opts.minDepthM) / std::max(0.001f, opts.maxDepthM - opts.minDepthM);
    cv::threshold(clipped, clipped, 1.0, 1.0, cv::THRESH_TRUNC);
    cv::threshold(clipped, clipped, 0.0, 0.0, cv::THRESH_TOZERO);
    cv::Mat u8, color;
    clipped.convertTo(u8, CV_8U, 255.0);
    cv::applyColorMap(u8, color, cv::COLORMAP_TURBO);
    color.setTo(cv::Scalar(0, 0, 0), invalid);
    return color;
}

static void drawTextBlock(cv::Mat &image, const Measurement &m) {
    std::vector<std::string> lines;
    lines.push_back("Status: " + m.reason);
    lines.push_back("Ground: " + std::to_string(static_cast<int>(m.groundDepthM * 1000.0f)) + " mm");
    if(m.ok) {
        lines.push_back("L: " + std::to_string(static_cast<int>(m.lengthM * 1000.0f)) + " mm");
        lines.push_back("W: " + std::to_string(static_cast<int>(m.widthM * 1000.0f)) + " mm");
        lines.push_back("H: " + std::to_string(static_cast<int>(m.heightM * 1000.0f)) + " mm");
        lines.push_back("Vol: " + std::to_string(m.volumeM3 * 1000.0f) + " L");
    }
    else {
        lines.push_back("Valid px: " + std::to_string(m.validDepthPx));
        lines.push_back("FG px: " + std::to_string(m.foregroundPx));
        lines.push_back("Max area: " + std::to_string(static_cast<int>(m.largestContourAreaPx)));
    }

    int maxWidth = 0;
    for(const auto &line: lines) {
        int baseline = 0;
        auto size = cv::getTextSize(line, cv::FONT_HERSHEY_SIMPLEX, 0.55, 2, &baseline);
        maxWidth = std::max(maxWidth, size.width);
    }
    cv::rectangle(image, cv::Rect(6, 6, maxWidth + 24, 18 + static_cast<int>(lines.size()) * 24), cv::Scalar(0, 0, 0), cv::FILLED);
    for(size_t i = 0; i < lines.size(); ++i) {
        cv::putText(image, lines[i], cv::Point(14, 28 + static_cast<int>(i) * 24), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2);
    }
}

static cv::Mat makeDebugPanel(const cv::Mat &bgr, const cv::Mat &depthM, const Measurement &m, const Options &opts) {
    cv::Mat annotated = bgr.clone();
    if(opts.hasRoi) {
        cv::rectangle(annotated, clampRect(opts.roi, annotated.size()), cv::Scalar(0, 255, 0), 2);
    }
    if(m.ok && m.boxPoints.size() == 4) {
        std::vector<cv::Point> points;
        for(const auto &pt: m.boxPoints) {
            points.emplace_back(cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)));
        }
        cv::polylines(annotated, points, true, cv::Scalar(0, 255, 0), 2);
        cv::circle(annotated, cv::Point(static_cast<int>(m.centerPx.x), static_cast<int>(m.centerPx.y)), 4, cv::Scalar(255, 0, 0), cv::FILLED);
    }
    drawTextBlock(annotated, m);

    cv::Mat depthVis = colorizeDepth(depthM, opts);
    cv::Mat maskVis = cv::Mat::zeros(bgr.size(), CV_8UC3);
    if(!m.mask.empty()) {
        std::vector<cv::Mat> channels(3, cv::Mat::zeros(m.mask.size(), CV_8U));
        channels[1] = m.mask;
        cv::merge(channels, maskVis);
    }
    if(opts.hasRoi) {
        cv::rectangle(maskVis, clampRect(opts.roi, maskVis.size()), cv::Scalar(0, 255, 0), 2);
        cv::rectangle(depthVis, clampRect(opts.roi, depthVis.size()), cv::Scalar(0, 255, 0), 2);
    }

    cv::Mat top, bottom, panel;
    cv::hconcat(bgr, maskVis, top);
    cv::hconcat(annotated, depthVis, bottom);
    cv::vconcat(top, bottom, panel);
    cv::resize(panel, panel, cv::Size(1280, 960));
    return panel;
}

static std::string timestampStem() {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S") << "_" << std::setw(3) << std::setfill('0') << ms;
    return oss.str();
}

static void saveNpyFloat32(const std::string &path, const cv::Mat &mat) {
    std::ofstream out(path, std::ios::binary);
    std::string header = "{'descr': '<f4', 'fortran_order': False, 'shape': (" + std::to_string(mat.rows) + ", " + std::to_string(mat.cols) + "), }";
    size_t preamble = 10;
    size_t padding = 16 - ((preamble + header.size() + 1) % 16);
    header.append(padding, ' ');
    header.push_back('\n');
    out.write("\x93NUMPY", 6);
    char version[2] = {1, 0};
    out.write(version, 2);
    uint16_t headerLen = static_cast<uint16_t>(header.size());
    out.write(reinterpret_cast<const char *>(&headerLen), 2);
    out.write(header.data(), static_cast<std::streamsize>(header.size()));
    out.write(reinterpret_cast<const char *>(mat.ptr<float>(0)), static_cast<std::streamsize>(mat.total() * sizeof(float)));
}

static void ensureDirectory(const std::string &path) {
    if(path.empty()) {
        return;
    }
    mkdir(path.c_str(), 0755);
}

static void saveCapture(const Options &opts, const cv::Mat &bgr, const cv::Mat &depthM, const cv::Mat &debug, const Measurement &m) {
    ensureDirectory(opts.outputDir);
    std::string stem = opts.outputDir + "/" + timestampStem();
    cv::imwrite(stem + "_rgb.png", bgr);
    cv::imwrite(stem + "_debug.png", debug);
    saveNpyFloat32(stem + "_enhanced_depth_m.npy", depthM);

    std::ofstream json(stem + "_measurement.json");
    json << "{\n"
         << "  \"ok\": " << (m.ok ? "true" : "false") << ",\n"
         << "  \"reason\": \"" << m.reason << "\",\n"
         << "  \"length_mm\": " << m.lengthM * 1000.0f << ",\n"
         << "  \"width_mm\": " << m.widthM * 1000.0f << ",\n"
         << "  \"height_mm\": " << m.heightM * 1000.0f << ",\n"
         << "  \"volume_l\": " << m.volumeM3 * 1000.0f << ",\n"
         << "  \"ground_depth_m\": " << m.groundDepthM << ",\n"
         << "  \"top_depth_m\": " << m.topDepthM << ",\n"
         << "  \"area_px\": " << m.areaPx << ",\n"
         << "  \"fill_ratio\": " << m.fillRatio << ",\n"
         << "  \"valid_depth_px\": " << m.validDepthPx << ",\n"
         << "  \"foreground_px\": " << m.foregroundPx << "\n"
         << "}\n";
    std::cout << "Saved capture: " << stem << std::endl;
}

static void printMeasurement(const Measurement &m) {
    std::cout << "{ok:" << (m.ok ? "true" : "false")
              << ", reason:" << m.reason
              << ", L_mm:" << m.lengthM * 1000.0f
              << ", W_mm:" << m.widthM * 1000.0f
              << ", H_mm:" << m.heightM * 1000.0f
              << ", ground_m:" << m.groundDepthM
              << ", top_m:" << m.topDepthM
              << ", valid_px:" << m.validDepthPx
              << ", fg_px:" << m.foregroundPx
              << ", largest_area:" << m.largestContourAreaPx
              << ", valid_min:" << m.validMinM
              << ", valid_median:" << m.validMedianM
              << ", valid_p90:" << m.validP90M
              << "}" << std::endl;
}

int main(int argc, char **argv) try {
    Options opts;
    if(!parseArgs(argc, argv, opts)) {
        return EXIT_FAILURE;
    }

    if(!opts.modelPath.empty() && !std::ifstream(opts.modelPath).good()) {
        std::cerr << "model file not found: " << opts.modelPath << std::endl;
        return EXIT_FAILURE;
    }

    ob::Pipeline pipe;
    auto device = pipe.getDevice();
    if(!device->isLicenseAuthorizationSupported()) {
        std::cerr << "device does not support license authorization, exit." << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "license info: " << device->readLicenseInfo() << std::endl;

#if defined(__ANDROID__) || !(defined(__linux__) && defined(__aarch64__))
    std::cerr << "EnhancedDepthFilter is only supported on NVIDIA Jetson Linux arm64." << std::endl;
    return EXIT_FAILURE;
#else
    auto alignFilter = std::make_shared<ob::Align>(OB_STREAM_COLOR);
    auto enhancedDepthFilter = std::make_shared<ob::EnhancedDepthFilter>(device, opts.modelPath);

    auto config = std::make_shared<ob::Config>();
    config->enableVideoStream(OB_STREAM_COLOR, opts.colorWidth, opts.colorHeight, opts.fps, OB_FORMAT_RGB);
    config->enableVideoStream(OB_STREAM_DEPTH, opts.depthWidth, opts.depthHeight, opts.fps, OB_FORMAT_Y16);
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
    pipe.start(config);

    cv::namedWindow("Enhanced Box Measurement", cv::WINDOW_NORMAL);
    cv::resizeWindow("Enhanced Box Measurement", 1280, 960);

    Measurement lastMeasurement;
    cv::Mat lastBgr, lastDepthM, lastDebug;

    while(true) {
        auto frameset = pipe.waitForFrameset(1000);
        if(!frameset) {
            continue;
        }
        auto aligned = alignFilter->process(frameset);
        if(!aligned) {
            continue;
        }
        auto processed = enhancedDepthFilter->process(aligned);
        if(!processed || !processed->is<ob::FrameSet>()) {
            continue;
        }

        auto processedFrameset = processed->as<ob::FrameSet>();
        auto colorFrame = processedFrameset->getColorFrame();
        auto depthFrame = processedFrameset->getDepthFrame();
        if(!colorFrame || !depthFrame) {
            continue;
        }

        lastBgr = colorFrameToBgr(colorFrame);
        lastDepthM = depthFrameToMeters(depthFrame);
        Intrinsics intr = getIntrinsics(colorFrame->as<ob::VideoFrame>());
        lastMeasurement = measureBox(lastDepthM, intr, opts);
        lastDebug = makeDebugPanel(lastBgr, lastDepthM, lastMeasurement, opts);

        cv::imshow("Enhanced Box Measurement", lastDebug);
        int key = cv::waitKey(1) & 0xff;
        if(key == 27 || key == 'q') {
            break;
        }
        if(key == 's') {
            saveCapture(opts, lastBgr, lastDepthM, lastDebug, lastMeasurement);
        }
        if(key == 'b') {
            opts.groundDepthM = calibrateGroundDepth(lastDepthM, opts);
            opts.hasGroundDepth = true;
            std::cout << "Calibrated ground depth: " << opts.groundDepthM * 1000.0f << " mm" << std::endl;
        }
        if(key == 'd') {
            printMeasurement(lastMeasurement);
        }
        if(key == '-' || key == '_') {
            opts.minBoxHeightM = std::max(0.001f, opts.minBoxHeightM - 0.005f);
            std::cout << "min_box_height_m = " << opts.minBoxHeightM << std::endl;
        }
        if(key == '+' || key == '=') {
            opts.minBoxHeightM += 0.005f;
            std::cout << "min_box_height_m = " << opts.minBoxHeightM << std::endl;
        }
        if(key == '[') {
            opts.minAreaPx = std::max(100, static_cast<int>(opts.minAreaPx * 0.8));
            std::cout << "min_area_px = " << opts.minAreaPx << std::endl;
        }
        if(key == ']') {
            opts.minAreaPx = static_cast<int>(opts.minAreaPx * 1.25);
            std::cout << "min_area_px = " << opts.minAreaPx << std::endl;
        }
    }

    pipe.stop();
    return EXIT_SUCCESS;
#endif
}
catch(const ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\nstatus:" << e.getStatus()
              << "\ntype:" << e.getExceptionType() << std::endl;
    return EXIT_FAILURE;
}
catch(const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

#ifndef UV_LED_DETECT_ADAPTIVE_H
#define UV_LED_DETECT_ADAPTIVE_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

namespace uvdar {

class UVDARLedDetectAdaptive {
public:
    UVDARLedDetectAdaptive(int neighborhoodSize = 50);
    ~UVDARLedDetectAdaptive();

    bool processImageAdaptive(const cv::Mat& inputImage, const std::vector<cv::Point2i>& trackingPoints, std::vector<cv::Point2i>& detectedPoints);

private:
    cv::Mat applyAdaptiveThreshold(const cv::Mat& image, const cv::Point2i& point, int neighborhoodSize);

    int neighborhoodSize_;
};

} // namespace uvdar

#endif // UV_LED_DETECT_ADAPTIVE_H
#ifndef UV_LED_DETECT_ADAPTIVE_H
#define UV_LED_DETECT_ADAPTIVE_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

namespace uvdar {

class UVDARLedDetectAdaptive {
public:
    UVDARLedDetectAdaptive(int neighborhoodSize = 25, double point_similarity_threshold = 5.0);
    ~UVDARLedDetectAdaptive();

    bool processImageAdaptive(const cv::Mat& inputImage, const std::vector<cv::Point2i>& trackingPoints, std::vector<cv::Point2i>& detectedPoints);

private:
    std::vector<cv::Point2i> applyAdaptiveThreshold(const cv::Mat& image, const cv::Point2i& point, int neighborhoodSize);

    std::vector<cv::Point2i> detectPointsFromRoi(const cv::Mat& mask, const cv::Rect& roi);

    bool isClose(const cv::Point2i& p1, const cv::Point2i& p2, double threshold);

    std::vector<cv::Point2i> mergePoints(const std::vector<cv::Point2i>& adaptivePoints,const std::vector<cv::Point2i>& standardPoints, double threshold);

    int neighborhoodSize_;
    double point_similarity_threshold_;
};

} // namespace uvdar

#endif // UV_LED_DETECT_ADAPTIVE_H
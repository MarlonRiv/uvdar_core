#ifndef UV_LED_DETECT_ADAPTIVE_H
#define UV_LED_DETECT_ADAPTIVE_H

#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <filesystem> 
#include <cmath>
#include <numeric>
#include <iomanip> // Include for std::fixed and std::setprecision
#include <tuple> // Include for std::tuple
#include <algorithm>


namespace uvdar {

class UVDARLedDetectAdaptive {
public:
    UVDARLedDetectAdaptive(int neighborhoodSize = 25, double point_similarity_threshold = 5.0);
    ~UVDARLedDetectAdaptive();

    bool processImageAdaptive(const cv::Mat& inputImage, const std::vector<cv::Point>& trackingPoints, std::vector<cv::Point>& detectedPoints, const std::vector<cv::Point>& standardPoints);

    void generateVisualizationAdaptive(const cv::Mat& inputImage,cv::Mat& visualization_image,const std::vector<cv::Point>& detectedPoints);

    const std::vector<cv::Mat>& getLastProcessedBinaryROIs() const {
      return lastProcessedBinaryROIs_;
    }

    const std::vector<cv::Rect>& getLastProcessedROIs() const {
      return lastProcessedROIs_;
    }

private:
    std::vector<cv::Point> applyAdaptiveThreshold(const cv::Mat& image, const cv::Point& point, int neighborhoodSize);

    std::vector<cv::Point> detectPointsFromRoi(const cv::Mat& mask, const cv::Rect& roi);

    bool isClose(const cv::Point& p1, const cv::Point& p2, double threshold);

    int adjustNeighborhoodSizeBasedOnArea(const cv::Mat& roiImage, const std::vector<std::vector<cv::Point>>& contours, int currentSize);

    std::vector<cv::Point> mergePoints(const std::vector<cv::Point>& adaptivePoints,const std::vector<cv::Point>& standardPoints, double threshold);

    double calculateKLDivergence(const std::vector<double>& segmentHist, const std::vector<double>& overallHist);
    double calculateKLDivergence2(const cv::Mat& hist, const std::vector<double>& Q, int start, int end);

    std::tuple<int, double> findOptimalThresholdUsingKL(const cv::Mat& roiImage);

    void saveRoiImage(const cv::Mat& binaryRoi, const cv::Point& center, int index, int thresholdValue, double klDivergence);

    int neighborhoodSize_;
    double point_similarity_threshold_;
    int roiIndex_;
    int MAX_SIZE = 50;
    int MIN_SIZE = 5;
    std::vector<cv::Mat> lastProcessedBinaryROIs_;
    std::vector<cv::Rect> lastProcessedROIs_; // For storing ROI positions
    std::vector<cv::Point> allRoiDetectedPoints;// For storing ROI positions
};

} // namespace uvdar

#endif // UV_LED_DETECT_ADAPTIVE_H
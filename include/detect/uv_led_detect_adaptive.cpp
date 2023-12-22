#include "uv_led_detect_adaptive.h"

namespace uvdar {

UVDARLedDetectAdaptive::UVDARLedDetectAdaptive(int neighborhoodSize) : neighborhoodSize_(neighborhoodSize) {}

UVDARLedDetectAdaptive::~UVDARLedDetectAdaptive() {}

bool UVDARLedDetectAdaptive::processImageAdaptive(const cv::Mat& inputImage, const std::vector<cv::Point2i>& trackingPoints, std::vector<cv::Point2i>& detectedPoints) {
    // Reset detected points
    detectedPoints.clear();

    // Create a mask from the result of static binarization (to be implemented)
    cv::Mat initialMask; // Placeholder for initial static binarization mask

    // Process each tracking point
    for (const auto& point : trackingPoints) {
        // Apply adaptive thresholding to the ROI
        cv::Mat adaptiveMask = applyAdaptiveThreshold(inputImage, point, neighborhoodSize_);

        // Append the processed ROI to the initial mask
        // (Logic to combine adaptiveMask with initialMask to be implemented)
    }

    // Process the final image to detect points (to be implemented)
    // Example: detectPointsFromMask(initialMask, detectedPoints);

    return true;// Return true if successful
}

cv::Mat UVDARLedDetectAdaptive::applyAdaptiveThreshold(const cv::Mat& image, const cv::Point2i& point, int neighborhoodSize) {
    cv::Mat grayImage;
    if (image.channels() == 3) {
        cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    } else {
        grayImage = image.clone();
    }

    cv::Rect roi(point.x - neighborhoodSize, point.y - neighborhoodSize, neighborhoodSize * 2, neighborhoodSize * 2);
    cv::Mat roiImage = grayImage(roi);

    cv::Mat binaryRoi;
    //cv::adaptiveThreshold(roiImage, binaryRoi, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);
    cv::threshold(roiImage, binaryRoi, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);


    return binaryRoi;
}

} // namespace uvdar
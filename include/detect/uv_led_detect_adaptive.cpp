#include "uv_led_detect_adaptive.h"
#include <iostream>

namespace uvdar {

UVDARLedDetectAdaptive::UVDARLedDetectAdaptive(int neighborhoodSize, double point_similarity_threshold) : neighborhoodSize_(neighborhoodSize),point_similarity_threshold_(point_similarity_threshold){}

UVDARLedDetectAdaptive::~UVDARLedDetectAdaptive() {}

bool UVDARLedDetectAdaptive::processImageAdaptive(const cv::Mat& inputImage, const std::vector<cv::Point2i>& trackingPoints, std::vector<cv::Point2i>& detectedPoints) {
    // Reset detected points
    detectedPoints.clear();

    // Create a mask from the result of static binarization (to be implemented)
    cv::Mat initialMask; // Placeholder for initial static binarization mask

    // Process each tracking point
    for (const auto& point : trackingPoints) {
        // Apply adaptive thresholding to the ROI around the tracking point
        std::vector<cv::Point2i> roiDetectedPoints = applyAdaptiveThreshold(inputImage, point, neighborhoodSize_);
        detectedPoints.insert(detectedPoints.end(), roiDetectedPoints.begin(), roiDetectedPoints.end());
    }

    // Merge the detected points with the standard points
    //std::vector<cv::Point2i> final_detected_points = mergePoints(detectedPoints, standardPoints, point_similarity_threshold_);

    // Process the final image to detect points (to be implemented)
    // Example: detectPointsFromMask(initialMask, detectedPoints);

    return true;// Return true if successful
}

std::vector<cv::Point2i> UVDARLedDetectAdaptive::applyAdaptiveThreshold(const cv::Mat& image, const cv::Point2i& point, int neighborhoodSize) {
    cv::Mat grayImage;
    if (image.channels() == 3) {
        cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    } else {
        grayImage = image.clone();
    }

    //std::cout << "[UVDARLedDetectAdaptive]: Points to process: " << point << std::endl;
    int x = std::max(0, point.x - neighborhoodSize);
    int y = std::max(0, point.y - neighborhoodSize);
    int width = std::min(neighborhoodSize * 2, image.cols - x);
    int height = std::min(neighborhoodSize * 2, image.rows - y);


    cv::Rect roi(x, y, width, height);

    //cv::Rect roi(point.x - neighborhoodSize, point.y - neighborhoodSize, 2 * neighborhoodSize, 2 * neighborhoodSize);
    cv::Mat roiImage = grayImage(roi);

    cv::Mat binaryRoi;
    //cv::adaptiveThreshold(roiImage, binaryRoi, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);
    cv::threshold(roiImage, binaryRoi, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    std::cout << "[UVDARLedDetectAdaptive]: ROI PROCESSED  Roi size: " << binaryRoi.size() << ", " << binaryRoi.type() << std::endl;

    // Detect points within this ROI
    std::vector<cv::Point2i> roiDetectedPoints = detectPointsFromRoi(binaryRoi, roi);
    std::cout << "[UVDARLedDetectAdaptive]: ADDING ROI DETECTED POINTS: " << roiDetectedPoints.size() << std::endl;
    return roiDetectedPoints;
}




std::vector<cv::Point2i> UVDARLedDetectAdaptive::detectPointsFromRoi(const cv::Mat& mask, const cv::Rect& roi) {
    std::vector<cv::Point2i> points;

    // Use connected components to find blobs in the mask
    cv::Mat labels;
    int numComponents = cv::connectedComponents(mask, labels, 8, CV_32S);

    // Iterate through components to calculate centroid
    for (int i = 1; i < numComponents; i++) {  // Start from 1 to ignore the background
        cv::Mat componentMask = (labels == i);

        // Calculate the centroid of the component
        cv::Moments m = cv::moments(componentMask, true);
        cv::Point2i centroid(static_cast<int>(m.m10 / m.m00) + roi.x, static_cast<int>(m.m01 / m.m00) + roi.y);

        points.push_back(centroid);
    }

    return points;
}


bool  UVDARLedDetectAdaptive::isClose(const cv::Point2i& p1, const cv::Point2i& p2, double threshold) {
    double dist = cv::norm(p1 - p2);
    return dist < threshold;
}

std::vector<cv::Point2i> UVDARLedDetectAdaptive::mergePoints(const std::vector<cv::Point2i>& adaptivePoints,
                                     const std::vector<cv::Point2i>& standardPoints, double point_similarity_threshold) {
    std::vector<cv::Point2i> combinedPoints;

    // Add all adaptive points
    combinedPoints.insert(combinedPoints.end(), adaptivePoints.begin(), adaptivePoints.end());

    // Add standard points if they are not close to any adaptive point
    for (const auto& standardPoint : standardPoints) {
        bool isOverlap = false;
        for (const auto& adaptivePoint : adaptivePoints) {
            if (isClose(standardPoint, adaptivePoint, point_similarity_threshold)) {
                isOverlap = true;
                break;
            }
        }
        if (!isOverlap) {
            combinedPoints.push_back(standardPoint);
        }
    }

    return combinedPoints;
}


} // namespace uvdar
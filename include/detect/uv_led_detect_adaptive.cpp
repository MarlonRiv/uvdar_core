#include "uv_led_detect_adaptive.h"
#include <iostream>
#include <set>
#include <filesystem>  // C++17 and above
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <numeric>
#include <iomanip> // Include for std::fixed and std::setprecision
#include <tuple> // Include for std::tuple
#include <vector>
#include <algorithm>
#include <string>




namespace fs = std::filesystem;


struct PointComparator {
    bool operator() (const cv::Point& a, const cv::Point& b) const {
        return (a.x < b.x) || (a.x == b.x && a.y < b.y);
    }
};

namespace uvdar {

UVDARLedDetectAdaptive::UVDARLedDetectAdaptive(int neighborhoodSize, double point_similarity_threshold, std::string adaptive_method) : neighborhoodSize_(neighborhoodSize),
point_similarity_threshold_(point_similarity_threshold), adaptive_method_(adaptive_method){}

UVDARLedDetectAdaptive::~UVDARLedDetectAdaptive() {}

/* processImageAdaptive //{ */
bool UVDARLedDetectAdaptive::processImageAdaptive(const cv::Mat& inputImage, const std::vector<cv::Point>& trackingPoints, std::vector<cv::Point>& detectedPoints,
const std::vector<cv::Point>& standardPoints) {

    /**
     * @brief: This function processes the input image to detect UV-LEDs using adaptive thresholding
     *  
     * Args:
     * inputImage: The input image
     * trackingPoints: The points around which the adaptive thresholding is applied
     * detectedPoints: The detected points
     * standardPoints: The standard points
     *  
     * @returns:
     * True if the process is successful, False otherwise
     */
    
    // Use a set to store unique points
    std::set<cv::Point, PointComparator> uniquePoints;

    // Reset detected points
    detectedPoints.clear();
    lastProcessedBinaryROIs_.clear();
    lastProcessedROIs_.clear();

    //Reset the number of ROIs
    numRois = 0;
    numberDetectedPoints.clear();
    thresholdValue.clear();
    klDivergence.clear();
    validRoi.clear();
    


    //Print size of tracking points
    //std::cout << "[UVDARLedDetectAdaptive]: TRACKING POINTS SIZE: " << trackingPoints.size() << std::endl;


    if (trackingPoints.size() == 0 || trackingPoints.size() > 50) {
        std::cout << "[UVDARLedDetectAdaptive]: INVALID NUMBER OF TRACKING POINTS" << std::endl;
        return false;
    } 


    std::vector<cv::Rect> rois;
    for (const auto& point : trackingPoints) {
        cv::Rect roi = calculateROI(inputImage, point, neighborhoodSize_);
        rois.push_back(roi);
    }

    std::vector<cv::Rect> mergedROIs = mergeOverlappingROIs(rois,inputImage.size(), 0.05); // Overlap threshold of 50%

    for (const auto& roi : mergedROIs) {
        std::vector<cv::Point> roiDetectedPoints = applyAdaptiveThreshold(inputImage, roi);
        detectedPoints.insert(detectedPoints.end(), roiDetectedPoints.begin(), roiDetectedPoints.end());
    }

/*     // Process each tracking point
    for (const auto& point : trackingPoints) {
        // Apply adaptive thresholding to the ROI around the tracking point
        //std::cout << "[UVDARLedDetectAdaptive]: PROCESSING TRACKING POINT: " << point << std::endl;
        std::vector<cv::Point> roiDetectedPoints = applyAdaptiveThreshold(inputImage, point, neighborhoodSize_);
        numRois++;
        

        // Check if the detected points are empty
        if (roiDetectedPoints.empty()){
            //std::cout << "[UVDARLedDetectAdaptive]: EMPTY ROI DETECTED POINTS" << std::endl;
            continue;
        }
        // Check each point for uniqueness before adding
        //detectedPoints.insert(detectedPoints.end(), roiDetectedPoints.begin(), roiDetectedPoints.end());
        // Check each point for uniqueness before adding
        for (const auto& roiPoint : roiDetectedPoints) {
            if (uniquePoints.insert(roiPoint).second) { // .second is true if the insertion is successful (i.e., the point is unique)
                detectedPoints.push_back(roiPoint); // Insert only the unique point
            }
        }
    } */

    std::vector<cv::Point> final_detected_points = mergePoints(detectedPoints, standardPoints, point_similarity_threshold_);
    

    /*

    //Print the final detected points
    for (int i = 0; i < final_detected_points.size(); i++) {
        std::cout << "[UVDARLedDetectAdaptive]: Final detected point " << i << ": " << final_detected_points[i] << std::endl;
    }
    
    */
    detectedPoints = final_detected_points;
    //standardPoints = final_detected_points;
    
    //Fail if no points are detected TODO
    //if (final_detected_points.size() == 0){
    //    return false;
    //}

    return true;//if successful
}
//}

/* applyAdaptiveThreshold //{ */
std::vector<cv::Point> UVDARLedDetectAdaptive::applyAdaptiveThreshold(const cv::Mat& inputImage, const cv::Rect& roi) {

    /**
     * 
     * @brief: This function applies adaptive thresholding to the neighborhood around a point and detects the points within the thresholded region
     *  
     * Args:
     * image: The input image
     * point: The point around which the adaptive thresholding is applied
     * neighborhoodSize: The size of the neighborhood around the point
     *  
     * @returns:
     * roiDetectedPoints: The points detected within the region of interest
     */

    cv::Mat grayImage;
    if (inputImage.channels() == 3) {
        cv::cvtColor(inputImage(roi), grayImage, cv::COLOR_BGR2GRAY);
    } else {
        grayImage = inputImage(roi).clone();
    }

    int MAX_AREA = 5;


   /*  cv::Mat grayImage;
    if (image.channels() == 3) {
        cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    } else {
        grayImage = image.clone();
    }

    int x = std::max(0, point.x - neighborhoodSize);
    int y = std::max(0, point.y - neighborhoodSize);
    int width = std::min(neighborhoodSize * 2, image.cols - x);
    int height = std::min(neighborhoodSize * 2, image.rows - y);

    cv::Rect roi(x, y, width, height); // Create a rectangle around the point
    //lastProcessedROIs_.push_back(roi); // Store the ROI for visualization
    //cv::Rect roi(point.x - neighborhoodSize, point.y - neighborhoodSize, 2 * neighborhoodSize, 2 * neighborhoodSize);
    cv::Mat roiImage = grayImage(roi); // Extract the ROI from the grayscale image
    //Copy the roiImage for comparison of threshold without blur
    cv::Mat roiImageCopy = roiImage.clone();
    //saveRoiImage(roiImage, point, roiIndex_++, 0, 0.0); */

    //Apply Gaussian blur to the ROI
    cv::Mat blurred;
    double sigmaX = 6.0;
    double sigmaY = 6.0;
    cv::GaussianBlur(grayImage, blurred, cv::Size(0, 0), sigmaX, sigmaY);

    // Create unsharp mask by subtracting the blurred version from the original image
    cv::Mat unsharpMask = grayImage - blurred;

    //saveRoiImage(unsharpMask, point, roiIndex_++, 0, 0.0);
    // Apply the unsharp mask to the original image to enhance edges
    cv::Mat enhancedImage;
    cv::addWeighted(grayImage, 0.25, unsharpMask, 1.75, 0, enhancedImage);

    //saveRoiImage(enhancedImage, point, roiIndex_++, 0, 0.0);

    cv::Mat binaryRoi;
    //cv::adaptiveThreshold(roiImage, binaryRoi, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);

    //Print the adaptive method
    //std::cout << "[UVDARLedDetectAdaptive]: ADAPTIVE METHOD: " << adaptive_method_ << std::endl;

    if( adaptive_method_ == "Otsu" || adaptive_method_ == "otsu"){
        //Apply Otsu's thresholding with the enhanced ROI
        int thresholdValue= cv::threshold(enhancedImage, binaryRoi, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU); // Apply Otsu's thresholding
        thresholdValue_ = thresholdValue;
    }
    else{
        //std::cout << "[UVDARLedDetectAdaptive]: APPLYING KL DIVERGENCE" << std::endl;
        auto [thresholdValue, minKLDivergence] = findOptimalThresholdUsingKL(enhancedImage);
        //Using entropy
        //auto [thresholdValue, minKLDivergence] = findOptimalThresholdUsingEntropy(enhancedImage);
        thresholdValue_ = thresholdValue;
        minKLDivergence_ = minKLDivergence;
        cv::threshold(enhancedImage,binaryRoi, thresholdValue_, 255, cv::THRESH_BINARY);
    }
   
    // Find contours in the binary ROI
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binaryRoi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // Create a mask to draw the filtered contours
    cv::Mat mask = cv::Mat::zeros(binaryRoi.size(), CV_8UC1);

    //Get the number of contours
    std::cout << "[UVDARLedDetectAdaptive]: NUMBER OF CONTOURS: " << contours.size() << std::endl;

    if (contours.size() >= 4){
        //Return empty roiDetectedPoints
        std::vector<cv::Point> empty_roiDetectedPoints = {};
        return empty_roiDetectedPoints;

    }

    for (const auto& contour : contours) {
        // Calculate the area of the contour
        double area = cv::contourArea(contour);
        // Filter based on area
        if (area < MAX_AREA) {
            std::cout << "[UVDARLedDetectAdaptive]: IN AREA: " << area << std::endl;
            // Draw the contour on the mask
            cv::drawContours(mask, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255), cv::FILLED);
        }else{
            std::cout << "[UVDARLedDetectAdaptive]: OUT OF AREA: " << area << std::endl;
        }
    }

    // Apply the mask to the binary ROI
    binaryRoi &= mask;

    // Detect points within this ROI
    std::vector<cv::Point> roiDetectedPoints = detectPointsFromRoi(binaryRoi, roi);
    std::cout << "[UVDARLedDetectAdaptive]: ROI DETECTED POINTS: " << roiDetectedPoints.size() << std::endl;

    if (roiDetectedPoints.size() > 5){
        std::cout << "[UVDARLedDetectAdaptive]: NOISY ROI: " << roiDetectedPoints.size() << std::endl;
        //saveRoiImage(binaryRoi, point, roiIndex_++, thresholdValue, -1.0);

        numberDetectedPoints.push_back(roiDetectedPoints.size());
        thresholdValue.push_back(thresholdValue_);
        if(adaptive_method_ == "Otsu" || adaptive_method_ == "otsu"){
            klDivergence.push_back(0.0);
        }
        else{
            klDivergence.push_back(minKLDivergence_);
        }
        //klDivergence.push_back(minKLDivergence);
        validRoi.push_back(0);
        
        //Return empty roiDetectedPoints

        std::vector<cv::Point> empty_roiDetectedPoints = {};
        return empty_roiDetectedPoints;
        //Clear the lastProcessedROIs_ and lastProcessedBinaryROIs_ vectors
        lastProcessedROIs_.clear();
        lastProcessedBinaryROIs_.clear();
    }
    else{
    lastProcessedROIs_.push_back(roi); // Store the ROI for visualization
    // Store the binary ROI (For debugging/visualization)
    lastProcessedBinaryROIs_.push_back(binaryRoi);
    //saveRoiImage(binaryRoi, point, roiIndex_++, thresholdValue_, 1.0);
    //std::cout << "[UVDARLedDetectAdaptive]: ADDING ROI DETECTED POINTS: " << roiDetectedPoints.size() << std::endl;


    numberDetectedPoints.push_back(roiDetectedPoints.size());
    thresholdValue.push_back(thresholdValue_);
    if(adaptive_method_ == "Otsu" || adaptive_method_ == "otsu"){
        klDivergence.push_back(0.0);
    }
    else{
        klDivergence.push_back(minKLDivergence_);
    }
    validRoi.push_back(1);

    return roiDetectedPoints;
    }

}
//}

cv::Rect UVDARLedDetectAdaptive::adjustROI(const cv::Rect& inputROI, const cv::Size& imageSize) {
    int x = std::max(0, inputROI.x);
    int y = std::max(0, inputROI.y);
    int width = std::min(inputROI.width, imageSize.width - x);
    int height = std::min(inputROI.height, imageSize.height - y);

    return cv::Rect(x, y, width, height);
}

cv::Rect UVDARLedDetectAdaptive::calculateROI(const cv::Mat& image, const cv::Point& point, int neighborhoodSize) {
    int x = std::max(0, point.x - neighborhoodSize);
    int y = std::max(0, point.y - neighborhoodSize);
    int width = std::min(neighborhoodSize * 2, image.cols - x);
    int height = std::min(neighborhoodSize * 2, image.rows - y);
    return cv::Rect(x, y, width, height);
}

std::vector<cv::Rect> UVDARLedDetectAdaptive::mergeOverlappingROIs(const std::vector<cv::Rect>& rois, const cv::Size& imageSize, double overlapThreshold) {
    std::vector<cv::Rect> mergedROIs;
    std::vector<bool> merged(rois.size(), false);

    for (int i = 0; i < rois.size(); i++) {
        if (merged[i]) continue;
        cv::Rect currentROI = rois[i];
        for (int j = i + 1; j < rois.size(); j++) {
            if (merged[j]) continue;
            if (isOverlapping(currentROI, rois[j], overlapThreshold)) {
                //Print debug message
                //std::cout << "[UVDARLedDetectAdaptive]: MERGING OVERLAPPING ROIS" << std::endl;
                // Collect the corners of both rectangles
                std::vector<cv::Point> corners;
                corners.push_back(cv::Point(currentROI.x, currentROI.y));
                corners.push_back(cv::Point(currentROI.x + currentROI.width, currentROI.y + currentROI.height));
                corners.push_back(cv::Point(rois[j].x, rois[j].y));
                corners.push_back(cv::Point(rois[j].x + rois[j].width, rois[j].y + rois[j].height));
                // Calculate the bounding rectangle of these corners
                currentROI = cv::boundingRect(corners);
                merged[j] = true;
            }
        }

        currentROI = adjustROI(currentROI, imageSize);
        mergedROIs.push_back(currentROI);
        merged[i] = true;
    }
    return mergedROIs;
}

bool UVDARLedDetectAdaptive::isOverlapping(const cv::Rect& roi1, const cv::Rect& roi2, double overlapThreshold) {
    cv::Rect intersection = roi1 & roi2;
    double iou = double(intersection.area()) / (roi1.area() + roi2.area() - intersection.area());
    return iou > overlapThreshold;
}


/* adjustNeighborhoodSizeBasedOnArea //{ */
int UVDARLedDetectAdaptive::adjustNeighborhoodSizeBasedOnArea(const cv::Mat& roiImage, const std::vector<std::vector<cv::Point>>& contours, int currentSize) {
    double totalArea = 0;
    for (const auto& contour : contours) {
        totalArea += cv::contourArea(contour);
    }

    // Print the total area
    std::cout << "[UVDARLedDetectAdaptive]: TOTAL AREA: " << totalArea << std::endl;

    // Logic to adjust size based on the area
    // This is an example; adjust thresholds and sizes as needed
    double areaRatio = totalArea / (roiImage.size().width * roiImage.size().height);

    // Print the area ratio
    std::cout << "[UVDARLedDetectAdaptive]: AREA RATIO: " << areaRatio << std::endl;

    // Revised adjustment calculation
    // Example adjustment: scale currentSize based on deviation from a desired area ratio
    const double targetAreaRatioSparse = 0.1, targetAreaRatioDense = 0.5;
    double adjustmentFactor = 0.5; // Proportion of currentSize to adjust by, needs tuning

    if (areaRatio < targetAreaRatioSparse) {
        // For sparse features, decrease size

        return std::max(static_cast<int>(currentSize * (1 - adjustmentFactor)), MIN_SIZE);
    } else if (areaRatio > targetAreaRatioDense) {

        // For dense features, increase size
        return std::min(static_cast<int>(currentSize * (1 + adjustmentFactor)), MAX_SIZE);
    }
    return currentSize; 
}
//}

/* detectPointsFromRoi //{ */
std::vector<cv::Point> UVDARLedDetectAdaptive::detectPointsFromRoi(const cv::Mat& mask, const cv::Rect& roi) {
    /**
    *   @brief: This function detects points from the mask and returns the centroid of the detected points
    * 
    *    Args:
    *    mask: The binary mask of the ROI, where the blobs are expected to be detected.
    *    roi: The rectangle of interest, used to adjust the coordinates of the detected points to the full image coordinate system.
    *
    *    @returns:
    *    points: The centroid of the detected points
    */

    std::vector<cv::Point> points;

    // Use connected components to find blobs in the mask
    cv::Mat labels;
    int numComponents = cv::connectedComponents(mask, labels, 8, CV_32S);


    //printf("Number of components: %d\n", numComponents);

    // Iterate through components to calculate centroid
    for (int i = 1; i < numComponents; i++) {  // Start from 1 to ignore the background
        cv::Mat componentMask = (labels == i);
        // Calculate the centroid of the component
        cv::Moments m = cv::moments(componentMask, true);
        cv::Point centroid(static_cast<int>(m.m10 / m.m00) + roi.x, static_cast<int>(m.m01 / m.m00) + roi.y);
        points.push_back(centroid);
    }

    return points;
}
//}

/* isClose //{ */
bool  UVDARLedDetectAdaptive::isClose(const cv::Point& p1, const cv::Point& p2, double threshold) {
/**
*   @brief: This function checks if two points are close to each other
*
*   Args:
*   p1: The first point
*   p2: The second point
*   threshold: The maximum distance between the two points for them to be considered close
*
*   @returns:
*   True if the points are close, False otherwise
*/


    double dist = cv::norm(p1 - p2);
    return dist < threshold;
}
//}

/* mergePoints //{ */
std::vector<cv::Point> UVDARLedDetectAdaptive::mergePoints(const std::vector<cv::Point>& adaptivePoints,
                                     const std::vector<cv::Point>& standardPoints, double threshold) {

    /**
     * @brief: This function merges the adaptive points with the standard points, ensuring that no two points are too close to each other
     *  
     * Args:
     * adaptivePoints: The points detected using adaptive thresholding
     * standardPoints: The standard points
     * threshold: The maximum distance between two points for them to be considered close
     *  
     * @returns:
     * combinedPoints: The merged points
     */


    std::vector<cv::Point> combinedPoints;
    // Add all adaptive points
    combinedPoints.insert(combinedPoints.end(), adaptivePoints.begin(), adaptivePoints.end());

    // Add standard points if they are not close to any adaptive point
    for (const auto& standardPoint : standardPoints) {
        bool isOverlap = false;
        for (const auto& adaptivePoint : adaptivePoints) {
            if (isClose(standardPoint, adaptivePoint, threshold)) {
                isOverlap = true;
                break;
            }
        }
        if (!isOverlap) {
            combinedPoints.push_back(standardPoint);
        }
    }

    //Print final size of combined points
    //std::cout << "[UVDARLedDetectAdaptive]: COMBINED POINTS SIZE: " << combinedPoints.size() << std::endl;

    return combinedPoints;
}
//}



/* calculateKLDivergence //{ */
    // Function to calculate KL divergence
    double UVDARLedDetectAdaptive::calculateKLDivergence(
        const std::vector<double>& segmentHist, 
        const std::vector<double>& overallHist, 
        size_t limit) {  // Add a 'limit' parameter to restrict calculations

        /**
        * @brief: This function calculates the Kullback-Leibler divergence between two distributions
        *  
        * Args:
        * segmentHist: The histogram of the segment
        * overallHist: The histogram of the overall image
        * limit: The index up to which the KL divergence should be calculated
        *  
        * @returns:
        * klDivergence: The Kullback-Leibler divergence
        */

        double klDivergence = 0.0;
        double epsilon = 1e-10;  // Small value to avoid log(0)
        for (size_t i = 0; i < overallHist.size(); ++i) {
            // Make sure we only calculate where both distributions have positive values
            if (segmentHist[i] > 0 && overallHist[i] > 0) {
                klDivergence += overallHist[i] * log(overallHist[i] / segmentHist[i] + epsilon);
            }
        }
        return klDivergence;

/*         double klDivergence = 0.0;
        double epsilon = 1e-10;  // Small value to avoid log(0)

        for (size_t i = 0; i < limit && i < segmentHist.size() && i < overallHist.size(); ++i) {
            if (segmentHist[i] > 0 && overallHist[i] > 0) {
                klDivergence += overallHist[i] * log((overallHist[i] + epsilon) / (segmentHist[i] + epsilon));
            }
        }
        return klDivergence; */
  
}


double UVDARLedDetectAdaptive::calculateEntropy(const std::vector<double>& histSegment) {
    double entropy = 0.0;
    for (double p : histSegment) {
        if (p > 0) {  // Only calculate for positive probabilities to avoid log(0)
            entropy -= p * log(p);  // Using natural log
        }
    }
    return entropy;
}

/* findOptimalThresholdUsingKL */
std::tuple<int, double> UVDARLedDetectAdaptive::findOptimalThresholdUsingKL(const cv::Mat& roiImage) {
    /**
     * @brief: This function finds the optimal threshold for adaptive thresholding using Kullback-Leibler divergence
     *  
     * Args:
     * roiImage: The region of interest
     *  
     * @returns:
     * optimalThreshold: The optimal threshold
     */

    //Print shape of roiImage
    //std::cout << "[UVDARLedDetectAdaptive]: ROI IMAGE SHAPE: " << roiImage.size() << std::endl;
    // Assuming 'roiImage' is already in a binary form (0s and 1s)
    int countZeros = cv::countNonZero(roiImage);
    int totalPixels = roiImage.total();
    int countOnes = totalPixels - countZeros;

    //Print the count of zeros and ones
    //std::cout << "[UVDARLedDetectAdaptive]: COUNT OF ZEROS: " << countZeros << std::endl;
    //std::cout << "[UVDARLedDetectAdaptive]: COUNT OF ONES: " << countOnes << std::endl;




    if (roiImage.empty()) {
        throw std::runtime_error("Input image is empty.");
    }
    
    // Calculate the histogram of the ROI image
    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange = {range};
    cv::Mat hist;
    cv::calcHist(&roiImage, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, false);
    cv::normalize(hist, hist, 1, 0, cv::NORM_L1, -1, cv::Mat());



    double minKLDivergence = std::numeric_limits<double>::max();
    int optimalThreshold = 0;

    
    // Convert cv::Mat hist to std::vector<double> for easier manipulation
    std::vector<double> Q(histSize);
    for (int i = 0; i < histSize; ++i) {
        Q[i] = hist.at<float>(i);
    }

    double epsilon = 1e-10;

    std::vector<double> P_below(histSize, 0), P_above(Q); // Use the full Q for P_above initially
    double sumBelow = 0.0, sumAbove = std::accumulate(Q.begin(), Q.end(), 0.0);
    for (int t = 1; t < histSize - 1; ++t) {
        sumBelow += Q[t - 1];
        sumAbove -= Q[t - 1];
        P_below[t] = sumBelow; // Track cumulative sum below the threshold

        if (sumBelow == 0 || sumAbove == 0) continue; // Skip invalid cases

        // Normalize probabilities up to the current threshold
        std::vector<double> normalizedP_below(P_below.begin(), P_below.begin() + t + 1);
        std::vector<double> normalizedP_above(P_above.begin() + t + 1, P_above.end());

        for (auto& val : normalizedP_below) val /= sumBelow;
        for (auto& val : normalizedP_above) val /= sumAbove;


        double klDivBelow = calculateKLDivergence(normalizedP_below, Q, t + 1);
        double klDivAbove = calculateKLDivergence(normalizedP_above, Q, histSize - t - 1);

       /*  std::cout << "Threshold: " << t << std::endl;
        std::cout << "Sum Below: " << sumBelow << ", Sum Above: " << sumAbove << std::endl;
        std::cout << "Normalized P_below: ";
        for (double p : normalizedP_below) std::cout << p << " ";
        std::cout << std::endl;

        std::cout << "Normalized P_above: ";
        for (double p : normalizedP_above) std::cout << p << " ";
        std::cout << std::endl;

        //Print the KL divergence
        std::cout << "[UVDARLedDetectAdaptive]: KL DIVERGENCE BELOW: " << klDivBelow << std::endl;
        std::cout << "[UVDARLedDetectAdaptive]: KL DIVERGENCE ABOVE: " << klDivAbove << std::endl; */

        double totalKLDiv = klDivBelow + klDivAbove;

        if (totalKLDiv < minKLDivergence && totalKLDiv > 0.0) {
            //Print found better threshold
            //std::cout << "[UVDARLedDetectAdaptive]: FOUND BETTER THRESHOLD: " << t << std::endl;
            //std::cout << "[UVDARLedDetectAdaptive]: MIN KL DIVERGENCE: " << totalKLDiv << std::endl;

            minKLDivergence = totalKLDiv;
            optimalThreshold = t;
        }


    }
    
    //Print the minKLDivergence and optimalThreshold
    //std::cout << "[UVDARLedDetectAdaptive]: MIN KL DIVERGENCE: " << minKLDivergence << std::endl;
    //std::cout << "[UVDARLedDetectAdaptive]: OPTIMAL THRESHOLD: " << optimalThreshold << std::endl;


    return std::make_tuple(optimalThreshold, minKLDivergence);

}
//}

std::tuple<int, double> UVDARLedDetectAdaptive::findOptimalThresholdUsingEntropy(const cv::Mat& roiImage) {
    if (roiImage.empty()) {
        throw std::runtime_error("Input image is empty.");
    }

    // Calculate the histogram as before
    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange = {range};
    cv::Mat hist;
    cv::calcHist(&roiImage, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, false);
    cv::normalize(hist, hist, 1, 0, cv::NORM_L1, -1, cv::Mat());

    if (cv::sum(hist)[0] == 0) {
        throw std::runtime_error("Histogram normalization failed.");
    }

    double maxEntropy = std::numeric_limits<double>::min();
    int optimalThreshold = 0;
    std::vector<double> Q(histSize);
    for (int i = 0; i < histSize; ++i) {
        Q[i] = hist.at<float>(i);
    }

    for (int t = 1; t < histSize - 1; ++t) {
        std::vector<double> P_below(Q.begin(), Q.begin() + t);
        std::vector<double> P_above(Q.begin() + t, Q.end());

        double entropyBelow = calculateEntropy(P_below);
        double entropyAbove = calculateEntropy(P_above);
        double totalEntropy = entropyBelow + entropyAbove;

        if (totalEntropy > maxEntropy) {
            maxEntropy = totalEntropy;
            optimalThreshold = t;
        }
    }

    //std::cout << "[UVDARLedDetectAdaptive]: MAX ENTROPY: " << maxEntropy << std::endl;
    //std::cout << "[UVDARLedDetectAdaptive]: OPTIMAL THRESHOLD: " << optimalThreshold << std::endl;

    return std::make_tuple(optimalThreshold, maxEntropy);
}

/* generateVisualizationAdaptive //{ */
void UVDARLedDetectAdaptive::generateVisualizationAdaptive(const cv::Mat& inputImage,cv::Mat& visualization_image, const std::vector<cv::Point>& detectedPoints) {

    /**
     * @brief: This function generates a visualization of the detected points
     *  
     * Args:
     * inputImage: The input image
     * visualization_image: The output visualization image
     *  
     * @returns:
     * None
    */

    // Create a copy of the current image for visualization
    //3the a black background with the same size as the input image
    
    visualization_image = inputImage.clone();



    // Check if the image is not empty
    if (visualization_image.empty()) {
        return;
    }

    // Overlay binary ROIs
    for (size_t i = 0; i < lastProcessedBinaryROIs_.size(); i++) {
        const auto& binaryRoi = lastProcessedBinaryROIs_[i];
        const auto& roi = lastProcessedROIs_[i];

        // Ensure the ROI is within the bounds of the image
        cv::Rect imageBounds(0, 0, visualization_image.cols, visualization_image.rows);
        if (imageBounds.contains(cv::Point(roi.x, roi.y)) && 
            imageBounds.contains(cv::Point(roi.x + roi.width, roi.y + roi.height))) {

            // Overlay the binary ROI
            binaryRoi.copyTo(visualization_image(roi));

            // Optionally draw a rectangle around the ROI
            cv::rectangle(visualization_image, roi, cv::Scalar(0, 255, 0)); // Green rectangle

            
            //    // Draw detected points within this ROI
            //for (const auto& point : detectedPoints) {
            //    cv::circle(visualization_image, point, 3, cv::Scalar(0, 0, 255), -1); // Red circle for each point
            //}
            
            
         
        }
    }
    
    


}
//}

/* saveRoiImage //{ */


void UVDARLedDetectAdaptive::saveRoiImage(const cv::Mat& binaryRoi, const cv::Point& center, int index, int thresholdValue = 0, double minKLDivergence = 0.0) {
    /**
     * @brief: This function saves the binary ROI as an image
     *  
     * Args:
     * binaryRoi: The binary ROI
     * center: The center of the ROI
     * index: The index of the ROI
     * thresholdValue: The threshold value used for adaptive thresholding
     *  
     * @returns:
     * None
     */
    // Specify the output directory
    std::string outputDir = "/home/rivermar/Desktop/MRS_Master_Project/roi_images";

    // Ensure the output directory exists
    fs::create_directories(outputDir);

     // Convert minKLDivergence to string with fixed decimal places
    std::stringstream minKLDivStream;
    minKLDivStream << std::fixed << std::setprecision(2) << minKLDivergence; // Adjust precision as needed

    // Format the filename
    std::string filename = "roi_" + std::to_string(center.x) + "_" + std::to_string(center.y) + "_" + std::to_string(index) + "_" + std::to_string(thresholdValue) + "_" + minKLDivStream.str() + ".png";
    std::string fullPath = fs::path(outputDir) / filename;

    // Save the image
    cv::imwrite(fullPath, binaryRoi);
}
//}


/* prepareAdaptiveDataForLogging //{ */
ROIData UVDARLedDetectAdaptive::prepareAdaptiveDataForLogging() {
    /**
     * @brief: This function prepares the adaptive data for logging
     *  
     * Args:
     * None
     *  
     * @returns:
     * A tuple containing the number of detected points, the threshold value, the KL divergence, and the validity of the ROI
     */
    
    ROIData adaptiveData;
    adaptiveData.numRois = numRois;
    adaptiveData.numberDetectedPoints = numberDetectedPoints;
    adaptiveData.thresholdValue = thresholdValue;
    adaptiveData.klDivergence = klDivergence;
    adaptiveData.validRoi = validRoi;

    return adaptiveData;
}


} // namespace uvdar
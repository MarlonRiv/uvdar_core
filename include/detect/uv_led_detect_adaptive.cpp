#include "uv_led_detect_adaptive.h"
#include <iostream>
#include <set>
#include <filesystem>  // C++17 and above
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <numeric>
#include <iomanip> // Include for std::fixed and std::setprecision
#include <tuple> // Include for std::tuple


namespace fs = std::filesystem;


struct PointComparator {
    bool operator() (const cv::Point& a, const cv::Point& b) const {
        return (a.x < b.x) || (a.x == b.x && a.y < b.y);
    }
};

namespace uvdar {

UVDARLedDetectAdaptive::UVDARLedDetectAdaptive(int neighborhoodSize, double point_similarity_threshold) : neighborhoodSize_(neighborhoodSize),point_similarity_threshold_(point_similarity_threshold){}

UVDARLedDetectAdaptive::~UVDARLedDetectAdaptive() {}

/* processImageAdaptive //{ */
bool UVDARLedDetectAdaptive::processImageAdaptive(const cv::Mat& inputImage, const std::vector<cv::Point>& trackingPoints, std::vector<cv::Point>& detectedPoints, const std::vector<cv::Point>& standardPoints) {

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


    //Print size of tracking points
    std::cout << "[UVDARLedDetectAdaptive]: TRACKING POINTS SIZE: " << trackingPoints.size() << std::endl;

    int ROI_COUNT = 0;

    // Process each tracking point
    for (const auto& point : trackingPoints) {
        // Apply adaptive thresholding to the ROI around the tracking point
        std::vector<cv::Point> roiDetectedPoints = applyAdaptiveThreshold(inputImage, point, neighborhoodSize_);
        ROI_COUNT++;
        // Check if the detected points are empty
        if (roiDetectedPoints.empty()){
            std::cout << "[UVDARLedDetectAdaptive]: EMPTY ROI DETECTED POINTS" << std::endl;
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
    }

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
std::vector<cv::Point> UVDARLedDetectAdaptive::applyAdaptiveThreshold(const cv::Mat& image, const cv::Point& point, int neighborhoodSize) {

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
    if (image.channels() == 3) {
        cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    } else {
        grayImage = image.clone();
    }

    int x = std::max(0, point.x - neighborhoodSize);
    int y = std::max(0, point.y - neighborhoodSize);
    int width = std::min(neighborhoodSize * 2, image.cols - x);
    int height = std::min(neighborhoodSize * 2, image.rows - y);
    int MAX_AREA = 5;

    cv::Rect roi(x, y, width, height); // Create a rectangle around the point
    //lastProcessedROIs_.push_back(roi); // Store the ROI for visualization
    //cv::Rect roi(point.x - neighborhoodSize, point.y - neighborhoodSize, 2 * neighborhoodSize, 2 * neighborhoodSize);
    cv::Mat roiImage = grayImage(roi); // Extract the ROI from the grayscale image
    //Copy the roiImage for comparison of threshold without blur
    cv::Mat roiImageCopy = roiImage.clone();
    cv::Mat roiImageCopy2 = roiImage.clone();
    saveRoiImage(roiImage, point, roiIndex_++, 0, 0.0);


    
    /*

        //Apply CLAHE to the ROI
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(2.0);
    clahe->apply(roiImageCopy2, roiImageCopy2);
    saveRoiImage(roiImageCopy2, point, roiIndex_++, 0, 0.0);
    
    */

    //Apply Gaussian blur to the ROI
    cv::Mat blurred;
    double sigmaX = 6.0;
    double sigmaY = 6.0;
    cv::GaussianBlur(roiImageCopy, blurred, cv::Size(0, 0), sigmaX, sigmaY);

    // Create unsharp mask by subtracting the blurred version from the original image
    cv::Mat unsharpMask = roiImageCopy - blurred;

    saveRoiImage(unsharpMask, point, roiIndex_++, 0, 0.0);
    // Apply the unsharp mask to the original image to enhance edges
    cv::Mat enhancedImage;
    cv::addWeighted(roiImageCopy, 0.5, unsharpMask, 1.5, 0, enhancedImage);

    saveRoiImage(enhancedImage, point, roiIndex_++, 0, 0.0);


    cv::Mat binaryRoi;
    //cv::adaptiveThreshold(roiImage, binaryRoi, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);
    double thresholdValue= cv::threshold(enhancedImage, binaryRoi, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU); // Apply Otsu's thresholding

    //Apply Otsu's thresholding without blur
    cv::Mat binaryRoiCopy_otsu;
    double thresholdValue_no_blur=cv::threshold(roiImageCopy, binaryRoiCopy_otsu, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU); // Apply Otsu's thresholding

    saveRoiImage(binaryRoiCopy_otsu, point, roiIndex_++, thresholdValue_no_blur, 0.0);


    auto [optimalThreshold, minKLDivergence] = findOptimalThresholdUsingKL(enhancedImage);

    //Apply the optimal threshold to the original ROI
    auto [optimalThreshold_no_blur, minKLDivergence_no_blur] = findOptimalThresholdUsingKL(roiImageCopy2);

    saveRoiImage(binaryRoi, point, roiIndex_++, optimalThreshold_no_blur, minKLDivergence_no_blur);

    
    cv::Mat binaryRoiCopy;

    cv::threshold(enhancedImage,binaryRoiCopy, optimalThreshold, 255, cv::THRESH_BINARY);

    std::cout << "[UVDARLedDetectAdaptive]: THRESHOLD VALUE: " << thresholdValue << std::endl;
    std::cout << "[UVDARLedDetectAdaptive]: OPTIMAL THRESHOLD VALUE: " << optimalThreshold << std::endl;
    //Get the value of the threshold
    saveRoiImage(binaryRoi, point, roiIndex_++, thresholdValue, 0.0);
    
    saveRoiImage(binaryRoiCopy, point, roiIndex_++, optimalThreshold, minKLDivergence);

    /*

    //Apply Gaussian blur to the ROI
    double sigmaX = 0.1;
    double sigmaY = 0.1;
    cv::GaussianBlur(roiImage, roiImage, cv::Size(0, 0), sigmaX, sigmaY);
    saveRoiImage(roiImage, point, roiIndex_++);
    
    //Otstu's thresholding without blur
    //cv::Mat binaryRoiCopy;
    //cv::threshold(roiImageCopy, binaryRoiCopy, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU); // Apply Otsu's thresholding

    //saveRoiImage(binaryRoiCopy, point, roiIndex_++);

    */
    
    // Perform morphological opening to remove small noise points
    //int morphSize = 1; // This is the size of the structuring element used for morphological operations
    //cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * morphSize + 1, 2 * morphSize + 1), cv::Point(morphSize, morphSize));
    //cv::morphologyEx(binaryRoi, binaryRoi, cv::MORPH_OPEN, element);
    // Find contours in the binary ROI
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binaryRoi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // Create a mask to draw the filtered contours
    cv::Mat mask = cv::Mat::zeros(binaryRoi.size(), CV_8UC1);

    //Get the number of contours
    //std::cout << "[UVDARLedDetectAdaptive]: NUMBER OF CONTOURS: " << contours.size() << std::endl;

    if (contours.size() > 2){
        //Return empty roiDetectedPoints
        std::vector<cv::Point> empty_roiDetectedPoints = {};
        return empty_roiDetectedPoints;

    }

    for (const auto& contour : contours) {
        // Calculate the area of the contour
        double area = cv::contourArea(contour);

        // Filter based on area
        if (area < MAX_AREA) {
            //std::cout << "[UVDARLedDetectAdaptive]: IN AREA: " << area << std::endl;
            // Draw the contour on the mask
            cv::drawContours(mask, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255), cv::FILLED);

        }else{
            //std::cout << "[UVDARLedDetectAdaptive]: OUT OF AREA: " << area << std::endl;
        }
    }
    // Apply the mask to the binary ROI
    binaryRoi &= mask;

    //saveRoiImage(binaryRoi, point, roiIndex_++);

    // Store the binary ROI (For debugging/visualization)
    //lastProcessedBinaryROIs_.push_back(binaryRoi);
    //std::cout << "[UVDARLedDetectAdaptive]: ROI PROCESSED  Roi size: " << binaryRoi.size() << ", " << binaryRoi.type() << std::endl;

    // Detect points within this ROI
    std::vector<cv::Point> roiDetectedPoints = detectPointsFromRoi(binaryRoi, roi);

    std::cout << "[UVDARLedDetectAdaptive]: ROI DETECTED POINTS: " << roiDetectedPoints.size() << std::endl;

    if (roiDetectedPoints.size() > 2){
        //std::cout << "[UVDARLedDetectAdaptive]: NOISY ROI: " << roiDetectedPoints.size() << std::endl;
        //Return empty roiDetectedPoints
        std::vector<cv::Point> empty_roiDetectedPoints = {};
        return empty_roiDetectedPoints;
        //Clear the lastProcessedROIs_ and lastProcessedBinaryROIs_ vectors
        lastProcessedROIs_.clear();
        lastProcessedBinaryROIs_.clear();

    }
    else{
    //saveRoiImage(binaryRoi, point, roiIndex_++);
    lastProcessedROIs_.push_back(roi); // Store the ROI for visualization
    // Store the binary ROI (For debugging/visualization)
    lastProcessedBinaryROIs_.push_back(binaryRoi);
    //std::cout << "[UVDARLedDetectAdaptive]: ADDING ROI DETECTED POINTS: " << roiDetectedPoints.size() << std::endl;
    return roiDetectedPoints;

    }

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
double UVDARLedDetectAdaptive::calculateKLDivergence(const std::vector<double>& segmentHist, const std::vector<double>& overallHist) {
    /**
     * @brief: This function calculates the Kullback-Leibler divergence between two distributions
     *  
     * Args:
     * segmentHist: The histogram of the segment
     * overallHist: The histogram of the overall image
     *  
     * @returns:
     * klDivergence: The Kullback-Leibler divergence
     */


    
    //print size of segmentHist and overallHist
    double klDivergence = 0.0;
    for (size_t i = 0; i < overallHist.size(); ++i) {
        // Make sure we only calculate where both distributions have positive values
        if (segmentHist[i] > 0 && overallHist[i] > 0) {
            klDivergence += overallHist[i] * log(overallHist[i] / segmentHist[i]);
        }
    }
    return klDivergence;
    
    
    

   /*

      double klDivergence = 0.0;
    for (size_t i = 0; i < segmentHist.size(); ++i) {
        // Ensure we only calculate where both distributions have positive values
        if (segmentHist[i] > 0 && overallHist[i] > 0) {
            double ratio = segmentHist[i] / overallHist[i];
            double logValue = log(ratio); // Calculate the logarithm of the ratio
            double contribution = overallHist[i] * logValue; // Contribution to the KL divergence

            // Debugging prints
            std::cout << "Bin " << i << ": segmentHist = " << segmentHist[i] << ", overallHist = " << overallHist[i] << std::endl;
            std::cout << "     Ratio = " << ratio << ", Log(Ratio) = " << logValue << ", Contribution = " << contribution << std::endl;

            klDivergence += contribution;
        } else if (segmentHist[i] == 0 && overallHist[i] > 0) {
            // Special case handling if needed, e.g., when segmentHist[i] is 0 but overallHist[i] is not
            // This scenario contributes 0 to KL divergence as per its mathematical properties but check how you want to handle it
            std::cout << "Bin " << i << " has segmentHist[i] = 0 and overallHist[i] > 0, which is skipped in calculation." << std::endl;
        }
    }
    std::cout << "Total KL Divergence: " << klDivergence << std::endl;
    return klDivergence;
   
   */


  
}
//}

// Function to calculate KL divergence
double UVDARLedDetectAdaptive::calculateKLDivergence2(const cv::Mat& hist, const std::vector<double>& Q, int start, int end) {
    double klDivergence = 0.0;
    for (int i = start; i <= end; ++i) {
        double p = hist.at<float>(i);
        // Assuming Q is normalized and has the same size as hist
        if (p > 0 && Q[i] > 0) { // Avoid division by zero or log(0)
            klDivergence += p * log(p / Q[i]);
        }
    }
    return klDivergence;
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

    // Iterate over all possible thresholds to find the one that minimizes the KL divergence
    for (int t = 1; t < histSize - 1; ++t) {
        // Split the normalized histogram at threshold t to create segmented distributions
        std::vector<double> P_below(Q.begin(), Q.begin() + t + 1);
        std::vector<double> P_above(Q.begin() + t + 1, Q.end());

        // Ensure these vectors represent probabilities for their segments by re-normalizing
        double sumBelow = std::accumulate(P_below.begin(), P_below.end(), 0.0);
        double sumAbove = std::accumulate(P_above.begin(), P_above.end(), 0.0);

        std::for_each(P_below.begin(), P_below.end(), [sumBelow](double& d) { d /= sumBelow; });
        std::for_each(P_above.begin(), P_above.end(), [sumAbove](double& d) { d /= sumAbove; });

        // Calculate the KL divergence for segments below and above the threshold
        double klDivBelow = calculateKLDivergence(P_below, Q);
        double klDivAbove = calculateKLDivergence(P_above, Q);

        // Total KL divergence for the current threshold
        double totalKLDiv = klDivBelow + klDivAbove;

        // Seeking the threshold that minimizes the KL divergence
        if (totalKLDiv < minKLDivergence && totalKLDiv > 0.0) {
            minKLDivergence = totalKLDiv;
            optimalThreshold = t;
        }
    }

    //Print the minKLDivergence and optimalThreshold
    std::cout << "[UVDARLedDetectAdaptive]: MIN KL DIVERGENCE: " << minKLDivergence << std::endl;
    std::cout << "[UVDARLedDetectAdaptive]: OPTIMAL THRESHOLD: " << optimalThreshold << std::endl;


    return std::make_tuple(optimalThreshold, minKLDivergence);

}
//}



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
    //Set the a black background with the same size as the input image
    
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



} // namespace uvdar
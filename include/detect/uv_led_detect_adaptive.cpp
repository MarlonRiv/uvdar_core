#include "uv_led_detect_adaptive.h"
#include <iostream>
#include <set>
#include <filesystem>  // C++17 and above
#include <opencv2/imgcodecs.hpp>

#include <cmath>
#include <numeric>
#include <iomanip>  // Include for std::fixed and std::setprecision
#include <tuple>    // Include for std::tuple
#include <vector>
#include <algorithm>
#include <string>


namespace fs = std::filesystem;


struct PointComparator
{
  bool operator()(const cv::Point& a, const cv::Point& b) const
  {
    return (a.x < b.x) || (a.x == b.x && a.y < b.y);
  }
};

namespace uvdar
{

  UVDARLedDetectAdaptive::UVDARLedDetectAdaptive(int neighborhoodSize, double point_similarity_threshold, std::string adaptive_method, bool adaptive_debug,
                                                 int contours_size_limit, int contour_max_size_limit, int roi_detected_points_limit, double sigma_x,
                                                 double sigma_y, double grayscale_roi_weight, double sharped_roi_weight)
      : neighborhoodSize_(neighborhoodSize),
        point_similarity_threshold_(point_similarity_threshold),
        adaptive_method_(adaptive_method),
        adaptive_debug_(adaptive_debug),
        contours_size_limit_(contours_size_limit),
        contour_max_size_limit_(contour_max_size_limit),
        roi_detected_points_limit_(roi_detected_points_limit),
        sigmaX_(sigma_x),
        sigmaY_(sigma_y),
        grayscale_ROI_weight_(grayscale_roi_weight),
        sharped_ROI_weight_(sharped_roi_weight)


  {
  }

  UVDARLedDetectAdaptive::~UVDARLedDetectAdaptive()
  {
  }

  /* processImageAdaptive() //{ */
  bool UVDARLedDetectAdaptive::processImageAdaptive(const cv::Mat& inputImage, const std::vector<cv::Point>& trackingPoints,
                                                    std::vector<cv::Point>& detectedPoints, const std::vector<cv::Point>& standardPoints)
  {

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

    std::scoped_lock lock(mutex_viz_rois_, mutex_rois_data_);

    // Reset detected points
    detectedPoints.clear();
    lastProcessedBinaryROIs_.clear();
    lastProcessedROIs_.clear();

    // Reset the number of ROIs
    numRois_ = 0;
    numberDetectedPoints_.clear();
    thresholdValues_.clear();
    klDivergences_.clear();
    validRois_.clear();

    // Avoid noisy images, to prevent adding overhead into the system if too many points are detected
    if (trackingPoints.size() == 0 || trackingPoints.size() > 20)
    {
      std::cout << "[UVDARLedDetectAdaptive]: INVALID NUMBER OF TRACKING POINTS" << std::endl;
      return false;
    }

    // To include the standardPoints
    /* std::vector<cv::Point> combinedPoints; */
    // Add all adaptive points
    /* combinedPoints.insert(combinedPoints.end(), trackingPoints.begin(), trackingPoints.end()); */

    std::vector<cv::Rect> rois;
    for (const auto& point : trackingPoints)
    {
      cv::Rect roi = calculateROI(inputImage, point, neighborhoodSize_);
      rois.push_back(roi);
    }

    if (adaptive_debug_)
    {
      // Print the size of the rois
      std::cout << "[UVDARLedDetectAdaptive]: NUMBER OF ROIS: " << rois.size() << std::endl;
    }

    const auto mergedROIs = mergeOverlappingROIs(rois, inputImage.size(), 0.05);  // Overlap threshold of 5%

    numRois_ = mergedROIs.size();

    if (adaptive_debug_)
    {
      // Print number of merged ROIs
      std::cout << "[UVDARLedDetectAdaptive]: NUMBER OF MERGED ROIS: " << mergedROIs.size() << std::endl;
    }

    for (const auto& roi : mergedROIs)
    {
      auto roiDetectedPoints = applyAdaptiveThreshold(inputImage, roi);
      // Save the detected points per ROI
      detectedPoints.insert(detectedPoints.end(), roiDetectedPoints.begin(), roiDetectedPoints.end());
    }


    const auto final_detected_points = mergePoints(detectedPoints, standardPoints, point_similarity_threshold_);

    // Updating the detected points
    detectedPoints = final_detected_points;

    return true;
  }
  //}

  /* applyAdaptiveThreshold() //{ */
  std::vector<cv::Point> UVDARLedDetectAdaptive::applyAdaptiveThreshold(const cv::Mat& inputImage, const cv::Rect& roi)
  {

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
    if (inputImage.channels() == 3)
    {
      cv::cvtColor(inputImage(roi), grayImage, cv::COLOR_BGR2GRAY);
    } else
    {
      grayImage = inputImage(roi).clone();
    }

    /* saveRoiImage(grayImage, "grayImage", index_); */
    /* // Apply Gaussian blur to the ROI */
    cv::Mat lowFrequency;
    /* cv::bilateralFilter(grayImage, blurred, 9, 75, 75); */
    cv::GaussianBlur(grayImage, lowFrequency, cv::Size(0, 0), sigmaX_, sigmaY_);
    cv::Mat highPass;
    cv::subtract(grayImage, lowFrequency, highPass);

    /* minMax //{ */
    
    double minVal, maxVal;
    cv::minMaxLoc(grayImage, &minVal, &maxVal);
    if (maxVal < 40)
    {
      /* std::cout << "[UVDARLedDetectAdaptive]: maxVal under :" << std::to_string(maxVal) << std::endl; */
      // Create a mask to draw the filtered contours */
      cv::Mat mask = cv::Mat::zeros(roi.size(), CV_8UC1);

      lastProcessedROIs_.push_back(roi);  // Store the ROI for visualization */
      // Store the binary ROI (For debugging/visualization) */
      lastProcessedBinaryROIs_.push_back(mask);

      // Return empty roiDetectedPoints */
      std::vector<cv::Point> empty_roiDetectedPoints = {};
      thresholdValues_.push_back(0.0);
      klDivergences_.push_back(0.0);
      numberDetectedPoints_.push_back(0);
      validRois_.push_back(0);
      return empty_roiDetectedPoints;

    } else
    {
      /* std::cout << "[UVDARLedDetectAdaptive]: maxVal :" << std::to_string(maxVal) << std::endl; */
    }
    
    //}

    /* cv::medianBlur(highPass, highPass, 3);  // 3x3 median filter */
    // Create unsharp mask by subtracting the blurred version from the original image
    /* cv::Mat unsharpMask = grayImage - blurred; */
    // Apply the unsharp mask to the original image to enhance edges
    cv::Mat enhancedImage;
    cv::addWeighted(grayImage, grayscale_ROI_weight_, highPass, sharped_ROI_weight_, 0, enhancedImage);
    // A small kernel (e.g., 3x3) can clean up artifacts without over-smoothing.
    cv::medianBlur(enhancedImage, enhancedImage, 3);

    /* saveRoiImage(postDenoised, "enhancedImage", index_); */
    
    /* top-hat attempt //{ */

    // Define the structuring element
    // Adjust the size (e.g., 15x15) based on the expected size of your bright feature

    /* cv::medianBlur(grayImage, grayImage, 3);  // 3x3 median filter */
    /* int kernelSize = 10; */
    /* cv::Mat structElem = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize)); */

    /* // Apply the white top-hat transform */
    /* cv::Mat topHat; */
    /* cv::morphologyEx(grayImage, topHat, cv::MORPH_TOPHAT, structElem); */


    /* cv::medianBlur(topHat, topHat, 3);  // 3x3 median filter */

    /* double minVal, maxVal; */
    /* cv::minMaxLoc(topHat, &minVal, &maxVal); */
    /* cv::Mat contrastStretched; */
    /* topHat.convertTo(contrastStretched, CV_8UC1, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal)); */

    /* cv::Mat normalizedEnhanced; */
    /* cv::normalize(contrastStretched, normalizedEnhanced, 0, 255, cv::NORM_MINMAX); */

    /* cv::Point point; */
    /* point.x = roi.x; */
    /* point.y = roi.y; */
    /* saveRoiImage(normalizedEnhanced, point, index_, 0, 0); */
    /* index_++; */
    /* cv::Mat enhancedImage = contrastStretched; */

    //}

    // BinaryRoi to save after applying the threshold
    cv::Mat binaryRoi;

    /* cv::Mat binaryRoiOriginal; */
    if (adaptive_method_ == "Otsu" || adaptive_method_ == "otsu")
    {
      // Apply Otsu's thresholding with the enhanced ROI
      double thresholdValue = cv::threshold(enhancedImage, binaryRoi, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);  // Apply Otsu's thresholding
      thresholdValue_ = thresholdValue;
      // Apply Otsu's thresholding with the original ROI
      /* int thresholdValueOriginal = cv::threshold(grayImage, binaryRoiOriginal, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU); // Apply Otsu's thresholding */

      /* /1* outlier threshold //{ *1/ */
      
      /* if (thresholdValue < 10) */
      /* { */

      /*   double new_threshold = thresholdValue * 10; */
      /*   std::cout << "[UVDARLedDetectAdaptive]: OUTLIER THRESHOLD: " << thresholdValue << std::endl; */
      /*   cv::threshold(enhancedImage, binaryRoi, new_threshold, 255, cv::THRESH_BINARY); */
      /*   thresholdValue_ = new_threshold; */

      /*   saveRoiImage(binaryRoi, "binarizedImage_outlier", index_); */
      /* } else */
      /* { */

      /*   saveRoiImage(binaryRoi, "binarizedImage", index_); */
      /* } */
      
      /* //} */

      /* saveRoiImage(binaryRoi, "binarizedImage", index_); */
    } else
    {
      // std::cout << "[UVDARLedDetectAdaptive]: APPLYING KL DIVERGENCE" << std::endl;
      auto [thresholdValue, minKLDivergence] = findOptimalThresholdUsingKL(enhancedImage);
      thresholdValue_ = thresholdValue;
      minKLDivergence_ = minKLDivergence;
      cv::threshold(enhancedImage, binaryRoi, thresholdValue_, 255, cv::THRESH_BINARY);
    }

    // Find contours in the binary ROI
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binaryRoi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // Create a mask to draw the filtered contours
    cv::Mat mask = cv::Mat::zeros(binaryRoi.size(), CV_8UC1);

    lastProcessedROIs_.push_back(roi);  // Store the ROI for visualization
    lastProcessedBinaryROIs_.push_back(binaryRoi);

    /* possible to delete filter based on area //{ */
    
    /* if (adaptive_debug_) */
    /* { */
    /*   // Print the contours size */
    /*   std::cout << "[UVDARLedDetectAdaptive]: NUMBER OF CONTOURS: " << contours.size() << std::endl; */
    /* } */

    /* // TODO find proper value for noisy ROI */
    /* if (static_cast<int>(contours.size()) >= contours_size_limit_) */
    /* { */
    /*   if (adaptive_debug_) */
    /*   { */
    /*     std::cout << "[UVDARLedDetectAdaptive]: NUMBER OF CONTOURS OUTSIDE OF THE LIMIT: " << contours.size() << std::endl; */
    /*   } */

    /*   lastProcessedBinaryROIs_.push_back(mask); */
    /*   // Return empty roiDetectedPoints */
    /*   std::vector<cv::Point> empty_roiDetectedPoints = {}; */
    /*   thresholdValues_.push_back(thresholdValue_); */
    /*   klDivergences_.push_back(0.0); */
    /*   numberDetectedPoints_.push_back(0); */
    /*   validRois_.push_back(0); */
    /*   return empty_roiDetectedPoints; */
    /* } */

    // TODO find proper value for MAX_AREA
    /* int MAX_AREA = 15; */

    
    //}
    
    double minCircularity = 0.45;
    for (const auto& contour : contours)
    {
      // Calculate the area of the contour
      double area = cv::contourArea(contour);
      // Filter based on area
      if (area > contour_max_size_limit_) {
        /* std::cout << "[UVDARLedDetectAdaptive]: CONTOUR OUTSIDE AREA: " << area << std::endl; */
        continue;
      }
      /* if (area < contour_max_size_limit_) */
      /* { */
      /* if (adaptive_debug_) */
      /* { */
      /*   std::cout << "[UVDARLedDetectAdaptive]: IN AREA: " << area << std::endl; */
      /* } */
      double perimeter = cv::arcLength(contour, true);
      if (perimeter == 0)
        continue;  // Avoid division by zero

      double circularity = 4 * CV_PI * area / (perimeter * perimeter);
      if (circularity < minCircularity)
      {
        /* std::cout << "[UVDARLedDetectAdaptive]: Not circle enough: " << circularity << std::endl; */
        continue;  // Skip contours that are not circular enough
      }
      // Draw the contour on the mask
      cv::drawContours(mask, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255), cv::FILLED);
      /* } else if (adaptive_debug_) */
      /* { */
      /*   std::cout << "[UVDARLedDetectAdaptive]: OUT OF AREA: " << area << std::endl; */
      /* } */
    }
    // Apply the mask to the binary ROI
    binaryRoi &= mask;
    /* saveRoiImage(binaryRoi, "binarizedMasked", index_); */
    index_++;
    // Detect points within this ROI
    std::vector<cv::Point> roiDetectedPoints = detectPointsFromRoi(binaryRoi, roi);

    if (adaptive_debug_)
    {
      // Print the number of detected points
      std::cout << "[UVDARLedDetectAdaptive]: ROI NUMBER OF DETECTED POINTS: " << roiDetectedPoints.size() << std::endl;
    }

    numberDetectedPoints_.push_back(roiDetectedPoints.size());
    thresholdValues_.push_back(thresholdValue_);

    if (adaptive_method_ == "Otsu" || adaptive_method_ == "otsu")
    {
      klDivergences_.push_back(0.0);
    } else
    {
      klDivergences_.push_back(minKLDivergence_);
    }

    if (static_cast<int>(roiDetectedPoints.size()) > roi_detected_points_limit_)
    {

      if (adaptive_debug_)
      {
        // Print the number of detected points
        std::cout << "[UVDARLedDetectAdaptive]: NOISY ROI: " << roiDetectedPoints.size() << std::endl;
      }
      numberDetectedPoints_.push_back(roiDetectedPoints.size());
      thresholdValues_.push_back(thresholdValue_);
      if (adaptive_method_ == "Otsu" || adaptive_method_ == "otsu")
      {
        klDivergences_.push_back(0.0);
      } else
      {
        klDivergences_.push_back(minKLDivergence_);
      }

      validRois_.push_back(0);
      // Return empty roiDetectedPoints
      std::vector<cv::Point> empty_roiDetectedPoints = {};
      // Clear the lastProcessedROIs_ and lastProcessedBinaryROIs_ vectors
      /* lastProcessedROIs_.clear(); */
      /* lastProcessedBinaryROIs_.clear(); */

      return empty_roiDetectedPoints;
    } else
    {

      /* // This is the reason it seems the visualization is blinking, getting some noisy rois in between and not being used */
      /* lastProcessedROIs_.push_back(roi);  // Store the ROI for visualization */
      /* // Store the binary ROI (For debugging/visualization) */
      /* lastProcessedBinaryROIs_.push_back(binaryRoi); */
      validRois_.push_back(1);
      return roiDetectedPoints;
    }
  }
  //}

  /* updateThreshold() //{ */

  double updateThreshold(double observed_threshold)
  {
  }

  //}

  /* adjustROI() //{ */

  cv::Rect UVDARLedDetectAdaptive::adjustROI(const cv::Rect& inputROI, const cv::Size& imageSize)
  {
    int x = std::max(0, inputROI.x);
    int y = std::max(0, inputROI.y);
    int width = std::min(inputROI.width, imageSize.width - x);
    int height = std::min(inputROI.height, imageSize.height - y);

    return cv::Rect(x, y, width, height);
  }

  //}

  /* calculateROI() //{ */

  cv::Rect UVDARLedDetectAdaptive::calculateROI(const cv::Mat& image, const cv::Point& point, int neighborhoodSize)
  {
    int x = std::max(0, point.x - neighborhoodSize);
    int y = std::max(0, point.y - neighborhoodSize);
    // To avoid going outside of the image bounds
    int width = std::min(neighborhoodSize * 2, image.cols - x);
    int height = std::min(neighborhoodSize * 2, image.rows - y);
    return cv::Rect(x, y, width, height);
  }

  //}

  /* mergeOverlappingROIs() //{ */

  std::vector<cv::Rect> UVDARLedDetectAdaptive::mergeOverlappingROIs(const std::vector<cv::Rect>& rois, const cv::Size& imageSize, double overlapThreshold)
  {
    std::vector<cv::Rect> mergedROIs;
    std::vector<bool> merged(rois.size(), false);

    for (size_t i = 0; i < rois.size(); i++)
    {
      if (merged[i])
        continue;
      cv::Rect currentROI = rois[i];
      for (size_t j = i + 1; j < rois.size(); j++)
      {
        if (merged[j])
          continue;
        if (isOverlapping(currentROI, rois[j], overlapThreshold))
        {
          std::vector<cv::Point> corners;
          corners.push_back(cv::Point(currentROI.x, currentROI.y));
          corners.push_back(cv::Point(currentROI.x + currentROI.width, currentROI.y + currentROI.height));
          corners.push_back(cv::Point(rois[j].x, rois[j].y));
          corners.push_back(cv::Point(rois[j].x + rois[j].width, rois[j].y + rois[j].height));
          // Calculate the bounding rectangle of the corners
          currentROI = cv::boundingRect(corners);
          merged[j] = true;
        }
      }

      // To check if it's not out of bounds
      currentROI = adjustROI(currentROI, imageSize);
      mergedROIs.push_back(currentROI);
      merged[i] = true;
    }
    return mergedROIs;
  }

  //}

  /* isOverlapping() //{ */

  bool UVDARLedDetectAdaptive::isOverlapping(const cv::Rect& roi1, const cv::Rect& roi2, double overlapThreshold)
  {
    cv::Rect intersection = roi1 & roi2;
    double iou = double(intersection.area()) / (roi1.area() + roi2.area() - intersection.area());
    return iou > overlapThreshold;
  }

  //}

  /* adjustNeighborhoodSizeBasedOnArea() //{ */
  int UVDARLedDetectAdaptive::adjustNeighborhoodSizeBasedOnArea(const cv::Mat& roiImage, const std::vector<std::vector<cv::Point>>& contours, int currentSize)
  {
    double totalArea = 0;
    for (const auto& contour : contours)
    {
      totalArea += cv::contourArea(contour);
    }

    // Print the total area
    std::cout << "[UVDARLedDetectAdaptive]: TOTAL AREA: " << totalArea << std::endl;

    // Logic to adjust size based on the area
    // This is an example; adjust thresholds and sizes as needed
    double areaRatio = totalArea / (roiImage.size().width * roiImage.size().height);

    // Print the area ratio
    std::cout << "[UVDARLedDetectAdaptive]: AREA RATIO: " << areaRatio << std::endl;

    const double targetAreaRatioSparse = 0.1, targetAreaRatioDense = 0.5;
    double adjustmentFactor = 0.5;  // Proportion of currentSize to adjust by, needs tuning

    if (areaRatio < targetAreaRatioSparse)
    {
      // For sparse features, decrease size

      return std::max(static_cast<int>(currentSize * (1 - adjustmentFactor)), MIN_SIZE);
    } else if (areaRatio > targetAreaRatioDense)
    {

      // For dense features, increase size
      return std::min(static_cast<int>(currentSize * (1 + adjustmentFactor)), MAX_SIZE);
    }
    return currentSize;
  }
  //}

  /* detectPointsFromRoi() //{ */
  std::vector<cv::Point> UVDARLedDetectAdaptive::detectPointsFromRoi(const cv::Mat& mask, const cv::Rect& roi)
  {
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

    // Iterate through components to calculate centroid
    for (int i = 1; i < numComponents; i++)
    {  // Start from 1 to ignore the background
      cv::Mat componentMask = (labels == i);
      // Calculate the centroid of the component
      cv::Moments m = cv::moments(componentMask, true);
      cv::Point centroid(static_cast<int>(m.m10 / m.m00) + roi.x, static_cast<int>(m.m01 / m.m00) + roi.y);
      points.push_back(centroid);
    }

    return points;
  }
  //}

  /* isClose() //{ */
  bool UVDARLedDetectAdaptive::isClose(const cv::Point& p1, const cv::Point& p2, double threshold)
  {
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

  /* mergePoints() //{ */
  std::vector<cv::Point> UVDARLedDetectAdaptive::mergePoints(const std::vector<cv::Point>& adaptivePoints, const std::vector<cv::Point>& standardPoints,
                                                             double threshold)
  {

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
    for (const auto& standardPoint : standardPoints)
    {
      bool isOverlap = false;
      for (const auto& adaptivePoint : adaptivePoints)
      {
        if (isClose(standardPoint, adaptivePoint, threshold))
        {
          isOverlap = true;
          break;
        }
      }
      if (!isOverlap)
      {
        if (adaptive_debug_)
        {

          std::cout << "[UVDARLedDetectAdaptive]: ADDING STANDARD POINT " << std::endl;
        }
        combinedPoints.push_back(standardPoint);
      }
    }


    return combinedPoints;
  }
  //}

  /* calculateKLDivergence() //{ */
  // Function to calculate KL divergence
  double UVDARLedDetectAdaptive::calculateKLDivergence(const std::vector<double>& segmentHist, const std::vector<double>& overallHist, size_t limit)
  {  // Add a 'limit' parameter to restrict calculations

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
    for (size_t i = 0; i < overallHist.size(); ++i)
    {
      // Make sure we only calculate where both distributions have positive values
      if (segmentHist[i] > 0 && overallHist[i] > 0)
      {
        klDivergence += overallHist[i] * log(overallHist[i] / segmentHist[i] + epsilon);
      }
    }
    return klDivergence;
  }


  double UVDARLedDetectAdaptive::calculateEntropy(const std::vector<double>& histSegment)
  {
    double entropy = 0.0;
    for (double p : histSegment)
    {
      if (p > 0)
      {                         // Only calculate for positive probabilities to avoid log(0)
        entropy -= p * log(p);  // Using natural log
      }
    }
    return entropy;
  }


  //}

  /* findOptimalThresholdUsingKL() //{ */
  std::tuple<int, double> UVDARLedDetectAdaptive::findOptimalThresholdUsingKL(const cv::Mat& roiImage)
  {
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


    std::vector<double> Q(histSize);
    for (int i = 0; i < histSize; ++i)
    {
      Q[i] = hist.at<float>(i);
    }

    std::vector<double> P_below(histSize, 0), P_above(Q);  // Use the full Q for P_above initially
    double sumBelow = 0.0, sumAbove = std::accumulate(Q.begin(), Q.end(), 0.0);
    for (int t = 1; t < histSize - 1; ++t)
    {
      sumBelow += Q[t - 1];
      sumAbove -= Q[t - 1];
      P_below[t] = sumBelow;  // Track cumulative sum below the threshold

      if (sumBelow == 0 || sumAbove == 0)
        continue;  // Skip invalid cases

      // Normalize probabilities up to the current threshold
      std::vector<double> normalizedP_below(P_below.begin(), P_below.begin() + t + 1);
      std::vector<double> normalizedP_above(P_above.begin() + t + 1, P_above.end());
      for (auto& val : normalizedP_below)
        val /= sumBelow;
      for (auto& val : normalizedP_above)
        val /= sumAbove;

      double klDivBelow = calculateKLDivergence(normalizedP_below, Q, t + 1);
      double klDivAbove = calculateKLDivergence(normalizedP_above, Q, histSize - t - 1);
      double totalKLDiv = klDivBelow + klDivAbove;

      if (totalKLDiv < minKLDivergence && totalKLDiv > 0.0)
      {
        minKLDivergence = totalKLDiv;
        optimalThreshold = t;
      }
    }

    return std::make_tuple(optimalThreshold, minKLDivergence);
  }

  //}

  /* findOptimalThresholdUsingEntropy() //{ */


  std::tuple<int, double> UVDARLedDetectAdaptive::findOptimalThresholdUsingEntropy(const cv::Mat& roiImage)
  {
    if (roiImage.empty())
    {
      throw std::runtime_error("Input image is empty.");
    }

    // Calculate the histogram as before
    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange = {range};
    cv::Mat hist;
    cv::calcHist(&roiImage, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, false);
    cv::normalize(hist, hist, 1, 0, cv::NORM_L1, -1, cv::Mat());

    if (cv::sum(hist)[0] == 0)
    {
      throw std::runtime_error("Histogram normalization failed.");
    }

    double maxEntropy = std::numeric_limits<double>::min();
    int optimalThreshold = 0;
    std::vector<double> Q(histSize);
    for (int i = 0; i < histSize; ++i)
    {
      Q[i] = hist.at<float>(i);
    }

    for (int t = 1; t < histSize - 1; ++t)
    {
      std::vector<double> P_below(Q.begin(), Q.begin() + t);
      std::vector<double> P_above(Q.begin() + t, Q.end());

      double entropyBelow = calculateEntropy(P_below);
      double entropyAbove = calculateEntropy(P_above);
      double totalEntropy = entropyBelow + entropyAbove;

      if (totalEntropy > maxEntropy)
      {
        maxEntropy = totalEntropy;
        optimalThreshold = t;
      }
    }


    return std::make_tuple(optimalThreshold, maxEntropy);
  }


  //}

  /* generateVisualizationAdaptive() //{ */
  void UVDARLedDetectAdaptive::generateVisualizationAdaptive(const cv::Mat& inputImage, cv::Mat& visualization_image,
                                                             const std::vector<cv::Point>& detectedPoints)
  {

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
    // 3the a black background with the same size as the input image

    visualization_image = inputImage.clone();

    // Check if the image is not empty
    if (visualization_image.empty())
    {
      return;
    }


    std::scoped_lock lock(mutex_viz_rois_);
    // Overlay binary ROIs
    for (size_t i = 0; i < lastProcessedBinaryROIs_.size(); i++)
    {
      const auto& binaryRoi = lastProcessedBinaryROIs_[i];
      const auto& roi = lastProcessedROIs_[i];

      // Ensure the ROI is within the bounds of the image
      cv::Rect imageBounds(0, 0, visualization_image.cols, visualization_image.rows);
      if (imageBounds.contains(cv::Point(roi.x, roi.y)) && imageBounds.contains(cv::Point(roi.x + roi.width, roi.y + roi.height)))
      {

        // Overlay the binary ROI
        binaryRoi.copyTo(visualization_image(roi));

        // Optionally draw a rectangle around the ROI
        cv::rectangle(visualization_image, roi, cv::Scalar(0, 255, 0));  // Green rectangle
      }
    }
  }
  //}

  /* plotHistogram() //{ */

  cv::Mat UVDARLedDetectAdaptive::plotHistogram(const cv::Mat& image)
  {
    // Compute the histogram
    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange = {range};
    cv::Mat hist;
    cv::calcHist(&image, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, false);


    // Apply logarithmic scaling to the histogram values
    hist += 1;  // Avoid log(0)
    cv::log(hist, hist);

    // Create an image to display the histogram
    int hist_w = 512;
    int hist_h = 400;
    int bin_w = std::round((double)hist_w / histSize);
    cv::Mat histImage(hist_h, hist_w, CV_8UC1, cv::Scalar(0));


    // Find the maximum value of the histogram for scaling
    double maxVal;
    cv::minMaxLoc(hist, 0, &maxVal);

    // Draw the histogram
    for (int i = 0; i < histSize; i++)
    {
      int binVal = std::round(hist.at<float>(i) * hist_h / maxVal);
      if (binVal > hist_h)
        binVal = hist_h;  // Cap the bin value to fit within the image height
      cv::line(histImage, cv::Point(bin_w * i, hist_h), cv::Point(bin_w * i, hist_h - binVal), cv::Scalar(255), 2, 8, 0);
    }

    return histImage;
  }

  //}

  /* saveRoiImage() //{ */


  void UVDARLedDetectAdaptive::saveRoiImage(const cv::Mat& binaryRoi, const std::string& name, int index)
  {
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

    /* // Convert minKLDivergence to string with fixed decimal places */
    /* std::stringstream minKLDivStream; */
    /* minKLDivStream << std::fixed << std::setprecision(2) << minKLDivergence;  // Adjust precision as needed */

    // Format the filename
    std::string filename = "roi_" + name + "_" + std::to_string(index) + "_" + ".png";
    std::string fullPath = fs::path(outputDir) / filename;

    // Save the image
    cv::imwrite(fullPath, binaryRoi);
  }
  //}

  /* prepareAdaptiveDataForLogging() //{ */
  ROIData UVDARLedDetectAdaptive::prepareAdaptiveDataForLogging()
  {
    /**
     * @brief: This function prepares the adaptive data for logging
     *
     * Args:
     * None
     *
     * @returns:
     * A tuple containing the number of detected points, the threshold value, the KL divergence, and the validity of the ROI
     */

    std::scoped_lock lock(mutex_rois_data_);
    ROIData adaptiveData;
    adaptiveData.numRois = numRois_;
    adaptiveData.numberDetectedPoints = numberDetectedPoints_;
    adaptiveData.thresholdValues = thresholdValues_;
    adaptiveData.klDivergences = klDivergences_;
    adaptiveData.validRois = validRois_;

    return adaptiveData;
  }


}  // namespace uvdar

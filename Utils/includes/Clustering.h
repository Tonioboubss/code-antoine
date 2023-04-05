#include <vector>
#include <math.h>

#ifndef CLUSTERING_
#define CLUSTERING_

#include "Cluster.h"
#include "Point.h"

class Cluster;

enum clusteringMethod //!< Several methods to know if two points belong to the same cluster.
{
    polarEstimation = 0,  // To estimate the angles difference tolerance knowing the distances (from roboat) of the points.
    polarCalculation = 1, // To compute the distance between two points and compare it with a tolerance.
};

/**
 * @class Clustering
 * @brief Static Class representing the definition of the attributes and methods of the Clustering process which gather close points from a dataset.
 * @author Antoine BOURRICAT
 * @version 0.1
 * @date 16/08/2022
 */
class Clustering
{
public:
    const static int DISTANCETOLERANCE_ = 10; //!< Maximum distance difference (cm) so they can belong to the same cluster.

    /**
     * @brief Conversion of an angle in degrees unit into radians unit.
     *
     * @param degAngle
     * @return float
     */
    static float convertDegToRad(float degAngle);

    /**
     * @brief Process the points of a given dataset to gather them into clusters to recognize objects or obstacle detections. Here is used a recursive clustering approach.
     *
     * @param rawDataPoints
     * @param methodToUSe
     * @param clustersVector
     * @return std::vector<Cluster>
     */
    static std::vector<Cluster> performClustering(std::vector<Point> rawDataPoints, clusteringMethod methodToUSe, std::vector<Cluster> clustersVector);
};

#endif
#include "../includes/Clustering.h"

/**
 * @brief Conversion of an angle in degrees unit into radians unit.
 *
 * @param degAngle
 * @return float
 */
float Clustering::convertDegToRad(float degAngle)
{
    return degAngle * std::acos(-1) / 180; // factor = pi / 180
}

/**
 * @brief Process the points of a given dataset to gather them into clusters to recognize objects or obstacle detections. Here is used a recursive clustering approach.
 *
 * @param rawDataPoints
 * @param methodToUSe
 * @param clustersVector
 * @return std::vector<Cluster>
 */
std::vector<Cluster> Clustering::performClustering(std::vector<Point> rawDataPoints, clusteringMethod methodToUSe, std::vector<Cluster> clustersVector)
{
    std::vector<Cluster> localClustersVector = clustersVector;

    if (!rawDataPoints.empty()) // some points remains unprocessed
    {
        // build a new cluster
        Cluster localCluster = Cluster();
        localCluster.addPointToVector(rawDataPoints.back()); // init with the first point
        rawDataPoints.pop_back();

        std::vector<Point> restVector; // vector which do not belong to the current cluster
        bool inCluster = false;        // return whether the point belongs or not to the cluster

        for (const Point &point : rawDataPoints)
        {
            switch (methodToUSe)
            {
            case polarEstimation:
                if (localCluster.belongToClusterPolarEstimation(point))
                    inCluster = true;
                else
                    inCluster = false;
                break;

            case polarCalculation:
                if (localCluster.belongToClusterPolarCalculation(point))
                    inCluster = true;
                else
                    inCluster = false;
                break;

            default:
                inCluster = false;
                break;
            }
            if (inCluster) // the point belong to localCluster
                localCluster.addPointToVector(point);
            else // the point does not belong to localCluster
                restVector.push_back(point);
        }
        localCluster.endCluster();
        localClustersVector.push_back(localCluster);
        return Clustering::performClustering(restVector, methodToUSe, localClustersVector);
    }

    else // all points are treated
        return localClustersVector;
}
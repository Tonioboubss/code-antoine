#include <vector>
#include <math.h>

#ifndef CLUSTER_
#define CLUSTER_

#include "Clustering.h"
#include "Point.h"

class Clustering;

/**
 * @class Cluster
 * @brief Class representing the definition of the attributes and methods of the Cluster Object which is composed of close points.
 * @author Antoine BOURRICAT
 * @version 0.4
 * @date 16/08/2022
 */
class Cluster
{
private:
    std::vector<Point> clusterPointsVec_; //!< Container of all points which belong to the cluster.
    Point firstPoint_;                    //!< First point (lowest angle) of the cluster.
    Point closestPoint_;                  //!< Closest point (lowest distance from roboat) of the cluster.
    Point lastPoint_;                     //!< Last point (highest angle) of the cluster.

public:
    Cluster();                                                                                              //!< Constructor
    Cluster(Point firstPoint, Point closestPoint, Point lastPoint, std::vector<Point> clusterPointsVector); //!< Constructor (useful for test datasets creation)
    ~Cluster();                                                                                             //!< Destructor

    /**
     * @brief Get the list of the three stored points that characterize the cluster.
     *
     * @return std::vector<Point>
     */
    std::vector<Point> getPointsVector();

    /**
     * @brief Add a new point to the cluster.
     *
     * @param newPoint
     */
    void addPointToVector(Point newPoint);

    /**
     * @brief Get the first point of the cluster.
     *
     * @return Point
     */
    const Point &getFirstPoint();

    /**
     * @brief Get the closest point of the cluster.
     *
     * @return Point
     */
    const Point &getClosestPoint();

    /**
     * @brief Get the last point of the cluster.
     *
     * @return Point
     */
    const Point &getLastPoint();

    /**
     * @brief Method to check if a new point is close enough from at least one of the cluster points. The way tend to estimate the angles difference tolerance knowing the distances (from roboat) of the point.
     *
     * @param nextPoint
     * @return true
     * @return false
     */
    bool belongToClusterPolarEstimation(Point nextPoint);

    /**
     * @brief Method to check if a new point is close enough from at least one of the cluster points. The way tend to compute the distance between two points and compare it with a tolerance.
     *
     * @param nextPoint
     * @return true
     * @return false
     */
    bool belongToClusterPolarCalculation(Point nextPoint);

    /**
     * @brief Intra cluster calculation to determine the first, the closest (from roboat) and the last point of the clusters.
     *
     */
    void endCluster();
};

#endif
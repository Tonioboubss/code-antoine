#include "../includes/Cluster.h"

/**
 * @brief Construct a new Cluster
 *
 */
Cluster::Cluster()
{
}

Cluster::Cluster(Point firstPoint, Point closestPoint, Point lastPoint, std::vector<Point> clusterPointsVector)
{
    firstPoint_.setPolarCoordinates(firstPoint.getAngle(), firstPoint.getDistance());
    closestPoint_.setPolarCoordinates(closestPoint.getAngle(), closestPoint.getDistance());
    lastPoint_.setPolarCoordinates(lastPoint.getAngle(), lastPoint.getDistance());
    for (const Point &clusterPoints : clusterPointsVector)
    {
        clusterPointsVec_.push_back(clusterPoints);
    }
}

Cluster::~Cluster()
{
}

std::vector<Point> Cluster::getPointsVector()
{
    return clusterPointsVec_;
}

/**
 * @brief Add a new point to the cluster.
 *
 * @param newPoint
 */
void Cluster::addPointToVector(Point newPoint)
{
    clusterPointsVec_.push_back(newPoint);
}

/**
 * @brief Get the first point of the cluster.
 *
 * @return Point
 */
const Point &Cluster::getFirstPoint()
{
    return firstPoint_;
}

/**
 * @brief Get the closest point of the cluster.
 *
 * @return Point
 */
const Point &Cluster::getClosestPoint()
{
    return closestPoint_;
}

/**
 * @brief Get the last point of the cluster.
 *
 * @return Point
 */
const Point &Cluster::getLastPoint()
{
    return lastPoint_;
}

/**
 * @brief Method to check if a new point is close enough from at least one of the cluster points. The way tend to estimate the angles difference tolerance knowing the distances (from roboat) of the point.
 *
 * @param nextPoint
 * @return true
 * @return false
 */
bool Cluster::belongToClusterPolarEstimation(Point nextPoint)
{
    float adjustedAngleTolerance = 0;
    for (Point &clusterPoints : getPointsVector())
    {
        // check if points are not to distant from each other, we take into account their distance from roboat to compute which tolerance on angles difference must be .
        if (std::abs(nextPoint.getDistance() - clusterPoints.getDistance()) < float(Clustering::DISTANCETOLERANCE_)) // measures belong to the same cluster if their angle difference is lower than 3 degrees AND their distance difference lower than 20mm
        {
            adjustedAngleTolerance = 2 * std::asin(float(Clustering::DISTANCETOLERANCE_) / (nextPoint.getDistance() + clusterPoints.getDistance())) / Clustering::convertDegToRad(1);
            if (std::abs(nextPoint.getAngle() - clusterPoints.getAngle()) < adjustedAngleTolerance)
                return true;
        }
    }
    return false;
}

/**
 * @brief Method to check if a new point is close enough from at least one of the cluster points. The way tend to compute the distance between two points and compare it with a tolerance.
 *
 * @param nextPoint
 * @return true
 * @return false
 */
bool Cluster::belongToClusterPolarCalculation(Point nextPoint)
{
    float calculation = 0;
    for (Point &clusterPoints : getPointsVector())
    {
        calculation = std::sqrt(std::pow(clusterPoints.getDistance(), 2) + std::pow(nextPoint.getDistance(), 2) - 2 * nextPoint.getDistance() * clusterPoints.getDistance() * std::cos(Clustering::convertDegToRad((nextPoint.getAngle() - clusterPoints.getAngle()))));
        if (calculation <= float(Clustering::DISTANCETOLERANCE_)) // Belong to same cluster if points are less distant than the tolerance
            return true;
    }
    return false;
}

/**
 * @brief Intra cluster calculation to determine the first, the closest (from roboat) and the last point of the clusters.
 *
 */
void Cluster::endCluster()
{
    // initialisation of the 3 particular points
    firstPoint_.setPolarCoordinates(clusterPointsVec_.at(0).getAngle(), clusterPointsVec_.at(0).getDistance());
    closestPoint_.setPolarCoordinates(clusterPointsVec_.at(0).getAngle(), clusterPointsVec_.at(0).getDistance());
    lastPoint_.setPolarCoordinates(clusterPointsVec_.at(0).getAngle(), clusterPointsVec_.at(0).getDistance());

    // update the 3 points
    for (Point &clusterPoints : clusterPointsVec_)
    {
        if (clusterPoints.getAngle() < firstPoint_.getAngle())
            firstPoint_.setPolarCoordinates(clusterPoints.getAngle(), clusterPoints.getDistance());
        if (clusterPoints.getDistance() < closestPoint_.getDistance())
            closestPoint_.setPolarCoordinates(clusterPoints.getAngle(), clusterPoints.getDistance());
        if (clusterPoints.getAngle() > lastPoint_.getAngle())
            lastPoint_.setPolarCoordinates(clusterPoints.getAngle(), clusterPoints.getDistance());
    }
}

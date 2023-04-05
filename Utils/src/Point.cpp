#include "../includes/Point.h"

/**
 * @brief Construct a new Point whose the polar coordinates are set.
 * 
 * @param angle 
 * @param distance 
 */
Point::Point(float angle, float distance)
{
    this->setPolarCoordinates(angle, distance);
}

Point::~Point()
{
}

float Point::getAngle() const
{
    return this->angle_;
}

float Point::getDistance() const
{
    return this->distance_;
}

void Point::setPolarCoordinates(float angle, float distance)
{
    this->angle_ = angle;
    this->distance_ = distance;
}
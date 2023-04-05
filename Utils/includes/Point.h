#ifndef POINT_
#define POINT_

/**
 * \class Point 
 * \brief Class representing the definition of the attributes and methods of the point object which allow to structure sensor detections.
 * \author Antoine BOURRICAT
 * \version 0.4
 * \date 16/08/2022
 */

class Point
{
private:
    float angle_;       //!< Angle (in degrees) of the point seen from roboat.
    float distance_;    //!< Distance (in cm) between the point and the roboat.

public: 
    Point(float angle, float distance); //!< Constructors
    Point(){};
    ~Point(); //!< Destructor

    /**
     * @brief Get the float value of the point angle.
     * 
     * @return float 
     */
    float getAngle() const;

    /**
     * @brief Get the float value of the point distance.
     * 
     * @return float 
     */
    float getDistance() const;

    /**
     * @brief Set the (new) polar coordinates of the point.
     * 
     * @param angle 
     * @param distance 
     */
    void setPolarCoordinates(float angle, float distance);
};

#endif

#include <vector>
#include <string>
#include "point.h"
#include "../../IComponent.h"
#include "../../../RoboatLogger/includes/RoboatLogger.h"
#include "Cluster.h"

#ifndef FAKELIDAR_H
#define FAKELIDAR_H

enum lidarModeEnum
{
    NO_OBJECT_LIDAR = 0,
    FRONT_RIGHT_CLOSE = 1,
    FRONT_RIGHT_FAR = 2,
    FRONT_LEFT_CLOSE = 3,
    FRONT_LEFT_FAR = 4,
    BEHIND_RIGHT_CLOSE = 5,
    BEHIND_RIGHT_FAR = 6,
    BEHIND_LEFT_CLOSE = 7,
    BEHIND_LEFT_FAR = 8,
    ARRAY_VALUE_LIDAR = 9
};

/**
 * \class FakeMotor
 * \brief Class allows to simulate the behavior of a lidar for test environment
 * \author Alexandre Girard
 * \version 0.1
 * \date 08/07/2022
 */
class FakeLidar : public IComponent
{
private:
    RoboatLogger loggerFakeLidar_; //!< logger associate to this class

    lidarModeEnum lidarMode; //!< choosen mode for the fake lidar

    const int MinCloseDistance = 0;  //!< minimum distance from a close distance object
    const int MinFarDistance = 300;  //!< minimum distance from a long distance object
    const int MaxFarDistance = 1200; //!< maximum value detected by the lidar

    const int MinFrontRight = 0;  //!< minimum angle from a front right object
    const int MinFrontLeft = 90;  //!< minimum angle from a front left object
    const int MinBackLeft = 180;  //!< minimum angle from a back left object
    const int MinBackRight = 270; //!< minimum angle from a back right object
    const int MaxBackRight = 360; //!< maximum angle from a back right object

    vector<Cluster> lastValue_;

    int randomValue(int min, int max);                                                               //!< generates a random value between min and max
    Cluster createFakeCluster(float minAngle, float maxAngle, float minDistance, float maxDistance); //!< method to create a fake cluster

public:
    FakeLidar(std::string name, Distinction distinctionSensor); // Construct a new Fake Lidar object
    ~FakeLidar();                // Destroy the Fake Lidar object

    void setNoObjectMode(); // Set Lidar to no object detected

    void setFrontRightCloseDistanceMode(); // Program the lidar to detect a close object ahead on the right
    void setFrontRightFarDistanceMode();   // Program the lidar to detect a long distance object ahead on the right

    void setFrontLeftCloseDistanceMode(); // Program the lidar to detect a close object ahead on the left
    void setFrontLeftFarDistanceMode();   // Program the lidar to detect a long distance object ahead on the left

    void setBehindRightCloseDistanceMode(); // Program the lidar to detect a close object behind on the right
    void setBehindRightFarDistanceMode();   // Program the lidar to detect a long distance object behind on the right

    void setBehindLeftCloseDistanceMode(); // Program the lidar to detect a close object behind on the left
    void setBehindLeftFarDistanceMode();   // Program the lidar to detect a long distance object behind on the left

    std::vector<Cluster> getElectronicalValue(); // get the Lidar's value
    Json::Value getJsonValue() override;  // transforms the value return by "getElectronicalValue" into a json
    vector<int> getEnvironnementValue() override;       //!< Method allowing to return a table of 24 boxes which gives the various obstacles detected in an angle of vision of 360

    bool verification() override;      // Function that defines a method to check the correct operation of the sensor
    void setValue(int value) override; // Method to set a value
    bool checkLastValue() override;    //!< method to check if there has been a change between the last two retrieved values
    vector<Cluster> getClusters();     // Retrieves a random cluster based on the lidar mode
};

class FakeLidarNoObject : public FakeLidar
{
public:
    FakeLidarNoObject(std::string name, Distinction distinctionSensor); // Construct a new Fake Lidar No Object object
};

class FakeLidarFrontRightClose : public FakeLidar
{
public:
    FakeLidarFrontRightClose(std::string name, Distinction distinctionSensor); // Construct a new Fake Lidar Front Right Close object
};

class FakeLidarFrontRightFar : public FakeLidar
{
public:
    FakeLidarFrontRightFar(std::string name, Distinction distinctionSensor); // Construct a new Fake Lidar Front Right Far object
};

class FakeLidarFrontLeftClose : public FakeLidar
{
public:
    FakeLidarFrontLeftClose(std::string name, Distinction distinctionSensor); // Construct a new Fake Lidar Front Left Close object
};

class FakeLidarFrontLeftFar : public FakeLidar
{
public:
    FakeLidarFrontLeftFar(std::string name, Distinction distinctionSensor); // Construct a new Fake Lidar Front Left Far object
};

class FakeLidarBehindRightClose : public FakeLidar
{
public:
    FakeLidarBehindRightClose(std::string name, Distinction distinctionSensor); // Construct a new Fake Lidar Behind Right Close object
};

class FakeLidarBehindRightFar : public FakeLidar
{
public:
    FakeLidarBehindRightFar(std::string name, Distinction distinctionSensor); // Construct a new Fake Lidar Behind Right Far object
};

class FakeLidarBehindLeftClose : public FakeLidar
{
public:
    FakeLidarBehindLeftClose(std::string name, Distinction distinctionSensor); // Construct a new Fake Lidar Behind Left Close object
};

class FakeLidarBehindLeftFar : public FakeLidar
{
public:
    FakeLidarBehindLeftFar(std::string name, Distinction distinctionSensor); // Construct a new Fake Lidar Behind Left Far object
};

class FakeLidarArrayValue : public FakeLidar
{
private:
    const std::vector<int> arrayDistance = {0, 10, 50, 0, 0, 0, 0, 0, 0, 0};
    int cptArrayDistance = -1;
    void initArray();

public:
    FakeLidarArrayValue(std::string name, Distinction distinctionSensor);
    int getDistance();
};

#endif
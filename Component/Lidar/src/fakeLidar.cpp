#include "../includes/fakeLidar.h"
#include <iostream>
#include <random>

/**
 * \brief Construct a new Fake Lidar object
 *
 * \param name
 */
FakeLidar::FakeLidar(std::string name, Distinction distinctionSensor) : IComponent(name,distinctionSensor,LidarType)
{
    loggerFakeLidar_ = RoboatLogger(name, "ConfigRoboat.json");
    loggerFakeLidar_.info(name + " creation");
}

/**
 * \brief Destroy the Fake Lidar object
 *
 */
FakeLidar::~FakeLidar()
{
    loggerFakeLidar_.debug("Delete " + this->getName());
}

/**
 * \brief Retrieves a random cluster based on the lidar mode
 *
 * \return vector<Cluster>
 */
vector<Cluster> FakeLidar::getClusters()
{
    vector<Cluster> clusterArray;
    switch (lidarMode)
    {
    case NO_OBJECT_LIDAR:
        /* code */
        clusterArray = {};
        return clusterArray;
    case FRONT_RIGHT_CLOSE:
        return {createFakeCluster(MinFrontRight, MinFrontLeft, MinCloseDistance, MinFarDistance)};
    case FRONT_RIGHT_FAR:
        return {createFakeCluster(MinFrontRight, MinFrontLeft, MinFarDistance, MaxFarDistance)};
    case FRONT_LEFT_CLOSE:
        return {createFakeCluster(MinFrontLeft, MinBackLeft, MinCloseDistance, MinFarDistance)};
    case FRONT_LEFT_FAR:
        return {createFakeCluster(MinFrontLeft, MinBackLeft, MinFarDistance, MaxFarDistance)};
    case BEHIND_RIGHT_CLOSE:
        return {createFakeCluster(MinBackRight, MaxBackRight, MinCloseDistance, MinFarDistance)};
    case BEHIND_RIGHT_FAR:
        return {createFakeCluster(MinBackRight, MaxBackRight, MinFarDistance, MaxFarDistance)};
    case BEHIND_LEFT_CLOSE:
        return {createFakeCluster(MinBackLeft, MinBackRight, MinCloseDistance, MinFarDistance)};
    case BEHIND_LEFT_FAR:
        return {createFakeCluster(MinBackLeft, MinBackRight, MinFarDistance, MaxFarDistance)};
    default:
        break;
    }
}

/**
 * \brief generates a random value between min and max
 *
 * \param min
 * \param max
 * \return int
 */
int FakeLidar::randomValue(int min, int max)
{
    std::random_device rd;                           // obtain a random number from hardware
    std::mt19937 gen(rd());                          // seed the generator
    std::uniform_int_distribution<> distr(min, max); // define the range
    return distr(gen);
}

/**
 * \brief Normalize the angle to the general data format
 *
 * \param angle
 * \return float
 */
float normalizeAngle(float angle)
{
    if (angle > 360)
        return angle - 360;
    else if (angle < 0)
        return angle + 360;
    return angle;
}

/**
 * \brief Method to create a fake cluster
 *
 * \param minAngle
 * \param maxAngle
 * \param minDistance
 * \param maxDistance
 * \return Cluster
 */
Cluster FakeLidar::createFakeCluster(float minAngle, float maxAngle, float minDistance, float maxDistance)
{
    float angleClosestPoint = randomValue(minAngle, maxAngle);
    float distanceClosestPoint = randomValue(minDistance, maxDistance);
    float limitAngleEnds = randomValue(0, 90);

    float limitAngleLeft = angleClosestPoint + limitAngleEnds;
    float limitAngleRight = angleClosestPoint - limitAngleEnds;
    float angleLeftEnd = normalizeAngle(randomValue(angleClosestPoint, limitAngleLeft));
    float distanceLeftEnd = randomValue(distanceClosestPoint, maxDistance);

    float angleRightEnd = normalizeAngle(randomValue(limitAngleRight, angleClosestPoint));
    float distanceRightEnd = randomValue(distanceClosestPoint, maxDistance);
    Point clostestPoint = Point(angleClosestPoint, distanceClosestPoint);
    Point rightEndPoint = Point(angleRightEnd, distanceRightEnd);
    Point leftEndPoint = Point(angleLeftEnd, distanceLeftEnd);
    return Cluster({leftEndPoint, clostestPoint, rightEndPoint});
}

/**
 * \brief Set Lidar to no object detected
 *
 */
void FakeLidar::setNoObjectMode()
{
    this->lidarMode = NO_OBJECT_LIDAR;
}

/**
 * \brief Program the lidar to detect a close object ahead on the right
 *
 */
void FakeLidar::setFrontRightCloseDistanceMode()
{
    this->lidarMode = FRONT_RIGHT_CLOSE;
}

/**
 * \brief Program the lidar to detect a long distance object ahead on the right
 *
 */
void FakeLidar::setFrontRightFarDistanceMode()
{
    this->lidarMode = FRONT_RIGHT_FAR;
}

/**
 * \brief Program the lidar to detect a close object ahead on the left
 *
 */
void FakeLidar::setFrontLeftCloseDistanceMode()
{
    this->lidarMode = FRONT_LEFT_CLOSE;
}

/**
 * \brief Program the lidar to detect a long distance object ahead on the left
 *
 */
void FakeLidar::setFrontLeftFarDistanceMode()
{
    this->lidarMode = FRONT_LEFT_FAR;
}

/**
 * \brief Program the lidar to detect a close object behind on the right
 *
 */
void FakeLidar::setBehindRightCloseDistanceMode()
{
    this->lidarMode = BEHIND_RIGHT_CLOSE;
}

/**
 * \brief Program the lidar to detect a long distance object behind on the right
 *
 */
void FakeLidar::setBehindRightFarDistanceMode()
{
    this->lidarMode = BEHIND_RIGHT_FAR;
}

/**
 * \brief Program the lidar to detect a close object behind on the left
 *
 */
void FakeLidar::setBehindLeftCloseDistanceMode()
{
    this->lidarMode = BEHIND_LEFT_CLOSE;
}

/**
 * \brief Program the lidar to detect a long distance object behind on the left
 *
 */
void FakeLidar::setBehindLeftFarDistanceMode()
{
    this->lidarMode = BEHIND_LEFT_FAR;
}

/**
 * \brief Function that defines a method to check the correct operation of the sensor
 *
 * \return true
 * \return false
 */
bool FakeLidar::verification()
{
    loggerFakeLidar_.debug(this->getName() + " verification");
    return true;
}

/**
 * \brief Construct a new Fake Lidar No Object object
 *
 * \param name
 */
FakeLidarNoObject::FakeLidarNoObject(std::string name, Distinction distinctionSensor) : FakeLidar(name, distinctionSensor)
{
    this->setNoObjectMode();
}

/**
 * \brief Construct a new Fake Lidar Front Right Close object
 *
 * \param name
 */
FakeLidarFrontRightClose::FakeLidarFrontRightClose(std::string name, Distinction distinctionSensor) : FakeLidar(name, distinctionSensor)
{
    this->setFrontRightCloseDistanceMode();
}

/**
 * \brief Construct a new Fake Lidar Front Right Far object
 *
 * \param name
 */
FakeLidarFrontRightFar::FakeLidarFrontRightFar(std::string name, Distinction distinctionSensor) : FakeLidar(name, distinctionSensor)
{
    this->setFrontRightFarDistanceMode();
}

/**
 * \brief Construct a new Fake Lidar Front Left Close object
 *
 * \param name
 */
FakeLidarFrontLeftClose::FakeLidarFrontLeftClose(std::string name, Distinction distinctionSensor) : FakeLidar(name, distinctionSensor)
{
    this->setFrontLeftCloseDistanceMode();
}

/**
 * \brief Construct a new Fake Lidar Front Left Far object
 *
 * \param name
 */
FakeLidarFrontLeftFar::FakeLidarFrontLeftFar(std::string name, Distinction distinctionSensor) : FakeLidar(name, distinctionSensor)
{
    this->setFrontLeftFarDistanceMode();
}

/**
 * \brief Construct a new Fake Lidar Behind Right Close object
 *
 * \param name
 */
FakeLidarBehindRightClose::FakeLidarBehindRightClose(std::string name, Distinction distinctionSensor) : FakeLidar(name, distinctionSensor)
{
    this->setBehindRightCloseDistanceMode();
}

/**
 * \brief Construct a new Fake Lidar Behind Right Far object
 *
 * \param name
 */
FakeLidarBehindRightFar::FakeLidarBehindRightFar(std::string name, Distinction distinctionSensor) : FakeLidar(name, distinctionSensor)
{
    this->setBehindRightFarDistanceMode();
}

/**
 * \brief Construct a new Fake Lidar Behind Left Close object
 *
 * \param name
 */
FakeLidarBehindLeftClose::FakeLidarBehindLeftClose(std::string name, Distinction distinctionSensor) : FakeLidar(name, distinctionSensor)
{
    this->setBehindLeftCloseDistanceMode();
}

/**
 * \brief Construct a new Fake Lidar Behind Left Far object
 *
 * \param name
 */
FakeLidarBehindLeftFar::FakeLidarBehindLeftFar(std::string name, Distinction distinctionSensor) : FakeLidar(name, distinctionSensor)
{
    this->setBehindLeftFarDistanceMode();
}

/**
 * \brief Allows you to recover the value of the sensor
 *
 * \return vector<Cluster>
 */
vector<Cluster> FakeLidar::getElectronicalValue()
{
    vector<Cluster> clusters = this->getClusters();
    return clusters;
}

/**
 * \brief transforms the value return by "getValue" into a json
 *
 * \return Json::Value
 */
Json::Value FakeLidar::getJsonValue()
{
    Json::Value angle;
    Json::Value distance;
    Json::Value value;
    Json::Value point;
    Json::Value measures;
    Json::Value sendableJson;

    vector<Cluster> vectorCluster = this->lastValue_;
    int i = 1;
    for (Cluster cluster : vectorCluster)
    {

        int j = 1;
        
        for (Point pts : cluster.getClusterVec())
        {
            point["name"] = "Point n°" + std::to_string(j);
            angle["value"] = pts.getAngle();
            angle["unit"] = "degree";
            distance["value"] = pts.getDistance() / 10;
            distance["unit"] = "cm";

            value["angle"] = angle;
            value["distance"] = distance;

            point["values"].append(value);

            measures["value"].append(point);
            point.clear();
            j++;
        }
        measures["name"] = "Cluster n°" + std::to_string(i);
        sendableJson["measures"].append(measures);
        measures.clear();
        i++;
    }

    sendableJson["sensorType"] = "Lidar";
    sendableJson["sensorName"] = this->getName();

    return sendableJson;
}

/**
 * \brief Method allowing to return a table of 24 boxes which gives the various obstacles detected in an angle of vision of 360
 *
 * \return int*
 */
vector<int> FakeLidar::getEnvironnementValue()
{
    loggerFakeLidar_.warning("Not implemented yet !!");
    std::vector<int> vect;
    return vect;
}

/**
 * \brief Method to set a value
 * Don't use it for Lidar
 *
 * \param value
 */
void FakeLidar::setValue(int value)
{
    loggerFakeLidar_.critical("Don't use setValue");
    throw ExcepFunctionUnused();
}

/**
 * \brief method to check if there has been a change between the last two retrieved values
 *
 * \return true
 * \return false
 */
bool FakeLidar::checkLastValue()
{
    vector<Cluster> value = getElectronicalValue();
    size_t size = value.size();
    if (size != lastValue_.size())
    {
        lastValue_ = value;
        return true;
    }
    else
    {
        for (int i = 0; i < size; i++)
        {
            if (value[i] != lastValue_[i])
            {
                lastValue_ = value;
                return true;
            }
        }
    }
    return true;
}
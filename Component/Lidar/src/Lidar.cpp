#include "../includes/Lidar.h"

/**
 * @brief Construct a new Lidar, set the name and the type of component, and start serial connection
 *
 * @param name
 */
Lidar::Lidar(std::string name)
{
    this->setName(name);
    this->setDisctinctionSensor(sensorNavigate);
    loggerLidar_ = RoboatLogger(name, "ConfigRoboat.json");
    loggerLidar_.info(name + " object creation");
    try
    {
        initialize();
    }
    catch (std::exception &e)
    {
        drv_.value->disconnect();
        loggerLidar_.error(e.what());
        throw &e;
    }
}

Lidar::~Lidar()
{
    opResult_ = drv_.value->stop(); // background thread stopped and current scan operation stopped
    if (SL_IS_OK(opResult_))
    {
        if (this->OPTCHANNELTYPE_ == sl::CHANNEL_TYPE_SERIALPORT)
            opResult_ = drv_.value->setMotorSpeed(0); // stop the lidar motor
        delete drv_.value;
        loggerLidar_.info("Delete " + this->getName() + " Object");
    }
    else
        loggerLidar_.error("Lidar delete process failed");
}

/**
 * @brief Establish serial communication between lidar and roboat and enables the driver.
 *
 */
void Lidar::initialize()
{
    opResult_ = drv_.value->connect(sl::createSerialPortChannel(CHANNELPARAMETER_, BAUDRATEPARAMETER_).value); // set the connection
    if (SL_IS_OK(opResult_))                                                                                   // check the connection to Lidar via channel
    {
        sl_lidar_response_device_info_t devInfo;
        opResult_ = drv_.value->getDeviceInfo(devInfo); // get device information including the serial number, firmware version, device model etc.
        if (SL_IS_OK(opResult_))                        // check if lidar is operational
            loggerLidar_.info("Lidar serial connection established");
        else
        {
            throw ExcepSerialConnectionLidar();
        }
    }
    else
        throw ExcepInitLidarDriver();
}

/**
 * @brief Recover raw Lidar data for one 0-360 degrees frame (including the angle and distance from roboat of every point).
 *
 * @return std::vector<Cluster>
 */
std::vector<Cluster> Lidar::collectSingleFrameData()
{
    if (verification())
    {
        loggerLidar_.info("Start collecting a frame of Data");
        measuresTable measuresTab[nbMeasures_]; // Raw data array initialisation
        std::vector<Point> pointsVector;        // vector to store preprocessed points
        try
        {
            opResult_ = drv_.value->grabScanDataHq(measuresTab, nbMeasures_); // Wait and grab a complete 0-360 degrees scan data previously received
            if (SL_IS_OK(opResult_))                                          // check if raw data grab process was successful
            {
                opResult_ = drv_.value->ascendScanData(measuresTab, nbMeasures_); // arrange the scan data according to the angle value in the scan.
                if (SL_IS_OK(opResult_))
                {
                    Point tempNormalizedData;
                    int nbFalsePoint = 0;

                    // Process which stores the ascended structured measures in a vector
                    for (int i = 0; i < nbMeasures_; i++)
                    {
                        tempNormalizedData = Point(normalizeData(measuresTab[i].angle_z_q14, measuresTab[i].dist_mm_q2).first, normalizeData(measuresTab[i].angle_z_q14, measuresTab[i].dist_mm_q2).second);
                        if ((measuresTab[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) && tempNormalizedData.getDistance() > 0.0f) // check if data are really significant
                            pointsVector.push_back(tempNormalizedData);                                                                     // add one structured mesure in the vector
                        else
                            nbFalsePoint++;
                    }
                    loggerLidar_.debug("Number of meaningful data : " + std::to_string(pointsVector.size()) + " points");
                    loggerLidar_.debug("number of unsignificant data : " + std::to_string(nbFalsePoint) + " points");
                }
                else
                    loggerLidar_.error("One frame of Data has been grabbed but failed to be sorted.");
            }
            else
                throw ExcepDataCollection();
        }
        catch (ExcepInitLidarDriver e)
        {
            drv_.value->disconnect();
            loggerLidar_.error(e.what());
        }
        catch (ExcepDataCollection e)
        {
            loggerLidar_.error(e.what());
        }
        loggerLidar_.info("Data collection end : one data frame has been captured");

        // Display all collected points , ex : "Measure n°1 : theta= 0.00005°, Distance= 50cm"
        int pointPos = 0;
        while (pointPos < pointsVector.size())
        {
            loggerLidar_.debug("Measure n° " + std::to_string(pointPos + 1) + " : theta= " + std::to_string(pointsVector.at(pointPos).getAngle()) + "°, Distance= " + std::to_string((int)pointsVector.at(pointPos).getDistance()) + " cm");
            pointPos++;
        }

        return Clustering::performClustering(pointsVector, DISTANCE_CALCULATION);
    }
    else
    {
        try
        {
            initialize();
            return collectSingleFrameData();
        }
        catch (const std::exception &e)
        {
            loggerLidar_.error(e.what());
            throw &e;
        }
    }
}

/**
 * @brief Check if detections in the new collected frame are the different from the previous one.
 *
 * @return true if detections in the new frame are different than in the previous one.
 * @return false
 */
bool Lidar::checkLastValue()
{
    std::vector<Cluster> newFrameDetections = collectSingleFrameData();
    if (newFrameDetections.size() != lastFrameDetections_.size())
    {
        lastFrameDetections_ = newFrameDetections;
        return true;
    }
    else
    {
        for (int i = 0; i < newFrameDetections.size(); i++)
        {
            if (!(newFrameDetections.at(i) == lastFrameDetections_.at(i)))
            {
                lastFrameDetections_ = newFrameDetections;
                return true;
            }
        }
        return false;
    }
}

/**
 * @brief Method allowing to return a table of 24 boxes which gives the various obstacles detected in an angle of vision of 360.
 *
 * @return std::vector<float>
 */
std::vector<float> Lidar::getEnvironnementValue()
{
    if (verification())
    {
        std::vector<float> distanceTab(360 / ZONE_WIDTH + 1, 0); // 25 elements : 1 header + 24 areas of 15°
        std::vector<Point> clusterPoints;
        int zoneIndex = 0;
        for (const Cluster &cluster : lastFrameDetections_)
        {
            clusterPoints = cluster.getAllPoints();
            for (const Point &p : clusterPoints)
            {
                zoneIndex = p.getAngle() / ZONE_WIDTH; // the zone width is 15°
                zoneIndex += 1;                        // first value of vector is header : 0 since obstacles are detected
                if (distanceTab.at(zoneIndex) == 0)
                {
                    distanceTab.at(zoneIndex) = p.getDistance();
                }
                else if (distanceTab.at(zoneIndex) > p.getDistance())
                {
                    distanceTab.at(zoneIndex) = p.getDistance();
                }
            }
        }

        int cmToMetersDivider = 100;
        int count = 0;
        for (float &cmDistance : distanceTab)
        {
            if (count != 0)
            {
                cmDistance /= cmToMetersDivider; // convert cm in m
                loggerLidar_.debug("Zone " + std::to_string(count) + " : " + std::to_string(cmDistance) + "m");
            }
            count++;
        }

        return distanceTab;
    }
    else
    {
        try
        {
            initialize();
            return getEnvironnementValue();
        }
        catch (const std::exception &e)
        {
            loggerLidar_.error(e.what());
            throw &e;
        }
    }
}

/**
 * @brief Allows you to read a value from a sensor and convert it to a string
 *
 * @return Json::Value
 */
Json::Value Lidar::getJsonValue()
{
    if (verification())
    {
        Json::Value angle;
        Json::Value distance;
        Json::Value value;
        Json::Value point;
        Json::Value measures;
        Json::Value sendableJson;

        std::vector<Cluster> detectedClusters = this->lastFrameDetections_;
        int i = 1;
        for (const Cluster &cluster : detectedClusters)
        {
            // First Point
            point["name"] = "Point n°1";
            angle["value"] = cluster.getFirstPoint().getAngle();
            angle["unit"] = "degree";
            distance["value"] = cluster.getFirstPoint().getDistance() / 10;
            distance["unit"] = "cm";

            value["angle"] = angle;
            value["distance"] = distance;

            point["values"].append(value);

            measures["value"].append(point);

            // Closest Point
            point.clear();
            point["name"] = "Point n°2";
            angle["value"] = cluster.getClosestPoint().getAngle();
            angle["unit"] = "degree";
            distance["value"] = cluster.getClosestPoint().getDistance() / 10;
            distance["unit"] = "cm";

            value["angle"] = angle;
            value["distance"] = distance;

            point["values"].append(value);

            measures["value"].append(point);

            // Last Point
            point.clear();
            point["name"] = "Point n°3";
            angle["value"] = cluster.getLastPoint().getAngle();
            angle["unit"] = "degree";
            distance["value"] = cluster.getLastPoint().getDistance() / 10;
            distance["unit"] = "cm";

            value["angle"] = angle;
            value["distance"] = distance;

            point["values"].append(value);

            measures["value"].append(point);

            measures["name"] = "Cluster n°" + std::to_string(i);
            sendableJson["measures"].append(measures);
            measures.clear();
            i++;
        }

        sendableJson["sensorType"] = "Lidar";
        sendableJson["sensorName"] = this->getName();

        return sendableJson;
    }
    else
    {
        try
        {
            initialize();
            return getJsonValue();
        }
        catch (const std::exception &e)
        {
            loggerLidar_.error(e.what());
            throw &e;
        }
    }
}

/**
 * @brief Allows to recover the value of a sensor. To define during the inheritance.
 *
 * @param value
 */
void Lidar::setValue(int value)
{
    loggerLidar_.critical("Don't use setValue for Lidar");
    throw ExcepFunctionUnused();
}

/**
 * @brief Allows to check the correct operation verifying the lidar health. Returns true if it is working, otherwise returns false.
 *
 * @return true if lidar is working normally
 * @return false
 */
bool Lidar::verification()
{
    loggerLidar_.info("Start lidar Verification");
    sl_lidar_response_device_health_t healthinfo;
    opResult_ = drv_.value->getHealth(healthinfo); // Retrieve the health status of the lidar

    if (SL_IS_OK(opResult_)) // check if health of lidar is ok
    {
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR)
            loggerLidar_.error("Lidar internal error detected. Please reboot the device to retry");
        else
        {
            // Scan enabling process
            if (this->OPTCHANNELTYPE_ == sl::CHANNEL_TYPE_SERIALPORT)
                opResult_ = drv_.value->setMotorSpeed();    // set motor speed to default speed
            opResult_ = drv_.value->startScan(false, true); // enable scanning : without forcing it and using the typical scan mode
            if (SL_IS_OK(opResult_))
                loggerLidar_.debug("Lidar health is fine and ready to make measurements");
            else
                loggerLidar_.error("The scan enabling process failed.");
            return true;
        }
    }
    else
    {
        drv_.value->disconnect();
        loggerLidar_.error("Error, cannot retrieve the lidar health code.");
        return false;
    }
    return false;
}

/**
 * @brief Convert and normalize angle and distance unsigned integers from sdk structure to usable angle (°) and distance (cm) floats.
 *
 * @param angle
 * @param distance
 * @return std::pair<float, float> pair.first = angle(°) and pair.second = disatnce (cm)
 */
std::pair<float, float> Lidar::normalizeData(float angle, float distance)
{
    return std::make_pair(angle * 90.f / 16384.f, distance / 40.0f); // angle in ° and distance in cm
}

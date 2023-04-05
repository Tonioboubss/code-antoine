#include <exception>
#include <utility>

#include "../sdk/include/sl_lidar.h"
#include "../sdk/include/sl_lidar_driver.h"

#include "../../IComponent.h"
#include "../../../RoboatLogger/includes/RoboatLogger.h"
#include "../../../Utils/includes/Clustering.h"

#ifndef LIDAR_
#define LIDAR_

typedef sl_lidar_response_measurement_node_hq_t measuresTable;

/**
 * \class ExcepInitLidarDriver
 * \brief This exception occurs because lidar initialization failed.
 * \author Antoine Bourricat
 * \version 0.2
 * \date 27/07/2022
 */
class ExcepInitLidarDriver : std::exception
{
public:
    virtual const char *what() const noexcept override
    {
        return "Serial Connection failed because lidar driver is not initialized, it may be due to insufficent memory.";
    }
};

/**
 * \class ExcepSerialConnectionLidar
 * \brief This exception occurs because lidar serial Connection attempt failed.
 * \author Antoine Bourricat
 * \version 0.2
 * \date 27/07/2022
 */
class ExcepSerialConnectionLidar : std::exception
{
public:
    virtual const char *what() const noexcept override
    {
        return "Connection to Lidar failed. Binding to the Serial port has not been allowed. Please verify permission (chown).";
    }
};

/**
 * \class ExcepCheckDataCollection
 * \brief This exception occurs if data collection failed.
 * \author Antoine Bourricat
 * \version 0.1
 * \date 12/07/2022
 */
class ExcepDataCollection : std::exception
{
public:
    virtual const char *what() const noexcept override
    {
        return "Data collection failed.";
    }
};

/**
 * \class Lidar
 * \brief Class representing the definition of the attributes and methods of the Lidar sensor object.
 * \author Antoine BOURRICAT
 * \version 0.5
 * \date 09/09/2022
 */
class Lidar : public IComponent
{
private:
    sl::Result<sl::ILidarDriver *> drv_ = sl::createLidarDriver(); //!< Lidar Driver Object to access all functions of the Lidar's SDK.
    const int OPTCHANNELTYPE_ = sl::CHANNEL_TYPE_SERIALPORT;       //!< Type of channel through which the lidar communicates with the Jetson Nvidia Nano. For our study, it will always be a serial communication.
    const std::string CHANNELPARAMETER_ = "/dev/ttyUSB0";          //!< USB port number through which the lidar is connected.
    const sl_u32 BAUDRATEPARAMETER_ = 115200;                      //!< Data transmission Baudrate. 115200 bauds = 11520 bytes/s.
    sl_result opResult_;                                           //!< sl_result object is used to notify whether lidar last operations suceed or not.

    size_t nbMeasures_ = 1100; // Number of the measures done for the collection process. If equal or higher than 1100, 0-360 degrees data collection can be done. If lower than 1100, the whole frame is not completed.
    std::vector<Cluster> lastFrameDetections_;

    RoboatLogger loggerLidar_; //!< Associated logger to this class.

    void initialize();                             //!< Initialize and establish the serial connection of the lidar with the Roboat.
    std::vector<Cluster> collectSingleFrameData(); //!< Recover raw Lidar data for one 0-360 degrees frame (including the angle and distance from roboat of every point).
    std::pair<float, float> normalizeData(float angle, float distance); //!< Convert and normalize angle and distance unsigned integers from sdk structure to usable angle and distance floats.

    const int ZONE_WIDTH = 15; //!< angle length of one single zone (in °).
public:
    Lidar(std::string name); // Constructor
    ~Lidar();                // Destructor

    std::vector<float> getEnvironnementValue() override; //!< Method allowing to return a table of 24 boxes which notify each closest detection of every zone around the roboat.
    Json::Value getJsonValue() override;                 //!< Allows you to read the last collected frame of scan and convert it to a string.

    bool verification() override;      //!< Allows to check the correct operation of the sensor. Returns true if sensor is working, otherwise returns false. To define during the inheritance.
    void setValue(int value) override; //!< Allows to recover the value of a sensor. To define during the inheritance.
    bool checkLastValue() override;    //!< Check if detections in the new collected frame are the different from the previous one.
};

#endif
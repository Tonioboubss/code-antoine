#ifndef ICOMPONENT_H
#define ICOMPONENT_H

#include <iostream>
#include <list>
#include <string>
#include <map>
#include <jsoncpp/json/json.h>
#include <vector>

/**
 * \class ExcepFunctionUnused
 * \brief This exception occurs when using an unusable function
 * \author Damien Lenormand
 * \version 1.0
 * \date 04/08/2022
 */
class ExcepFunctionUnused : public std::exception { 
    public : 
        virtual const char *what() const noexcept override{
            return "Use of an unusable function";
        } 
};

/**
 * \class ExcepFunctionNonImplemented
 * \brief This exception occurs when using an unimplemented function
 * \author Damien Lenormand
 * \version 1.0
 * \date 04/08/2022
 */
class ExcepFunctionNonImplemented : public std::exception { 
    public : 
        virtual const char *what() const noexcept override{
            return "Function non implemented";
        } 
};

enum Distinction
{
  otherSensor = 3,
  activator = 2,
  sensorNavigate = 1,
  SensorInformation = 0
}; //!< Enumeration of state

enum SensorType
{
  CameraType = 0,
  LidarType = 1,
  ServoType = 2,
  MotorType = 3,
  LedType = 4,
  UltrasonType = 5,
  ButtonType = 6
};

/**
 * \author Damien Lenormand and Arthur Parrod
 * \brief Mother class of all sensor objects
 * \version 1.0
 * \date 18/07/2022
 *
 * @copyright Copyright (c) 2022
 *
 */

class IComponent
{
private:
  Distinction distinctionSensor_; //!< Attribute which allows to specify the nature of the component
  std::string name_;              //!< string representing the name of the object
  SensorType sensorType_; 

protected:
  /**
   * \brief Set the Name object
   *
   * \param name
   */
  void setName(std::string name)
  {
    this->name_ = name;
  };

public:
  IComponent(std::string name, Distinction distinction, SensorType sensorType) : distinctionSensor_(distinction), name_(name), sensorType_(sensorType) {};
  virtual ~IComponent(){};                              //!< Destroy the ISensor object
  IComponent() {};
  virtual std::vector<int> getEnvironnementValue() = 0; //!< Method allowing to return a table of 24 boxes which gives the various obstacles detected in an angle of vision of 360
  virtual Json::Value getJsonValue() = 0;        //!< Allows you to read a value from a sensor and convert it to a string
  virtual bool verification() = 0;                      //!< Allows to check the correct operation of the sensor. Returns true if sensor is working, otherwise returns false. To define during the inheritance
  virtual void setValue(int value) = 0;                 //!< Allows to recover the value of a sensor. To define during the inheritance
  virtual bool checkLastValue() = 0;                    //!< method to check if there has been a change between the last two retrieved values
  /**
   * \brief Get the Name object
   *
   * \return const std::string&
   */
  const std::string &getName()
  {
    return this->name_;
  }

  /**
   * \brief Get the Distinction Sensor object
   *
   * \return Distinction
   */
  Distinction getDistinctionSensor()
  {
    return this->distinctionSensor_;
  };

  /**
   * \brief Set the Disctinction Sensor object
   *
   * \param newDistinction
   */
  void setDisctinctionSensor(Distinction newDistinction)
  {
    this->distinctionSensor_ = newDistinction;
  }

  SensorType getSensorType(){
    return sensorType_;
  }

  void setSensorType(SensorType type){
    sensorType_ = type;
  }
};
#endif
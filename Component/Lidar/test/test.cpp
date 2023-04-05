#include "../includes/fakeLidar.h"

int main(){
    FakeLidar *lidar = new FakeLidarFrontRightClose("Lidar");
    
    vector<Cluster> c =lidar->getValue();

    std::cout << lidar->getJsonValue() << std::endl;

    delete lidar;
}
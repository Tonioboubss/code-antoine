#include "../includes/Lidar.h"
#include "../../../Utils/includes/Clustering.h"

int main(int argc, char *argv[])
{
    // Lidar Object Initialisation
    Lidar *lidar = new Lidar("Lidar");

    // Check lidar status //
    if (lidar->verification())
    {
        // make one frame of scan and perform cluster detection
        if (lidar->checkLastValue())
        {
            std::vector<float> result = lidar->getEnvironnementValue();
        }
    }
    delete lidar;

    return 0;
}

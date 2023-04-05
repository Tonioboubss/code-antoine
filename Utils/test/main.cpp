#include "../includes/Clustering.h"
#include "../includes/Cluster.h"
#include "../includes/Point.h"

#include <iostream>

int main()
{
    std::vector<Point> dataset = {Point(270.0, 20.0), Point(10.0, 500.0), Point(269.0, 20.0), Point(11.0, 499.0),Point(273.0, 20.0), Point(180.0, 1000.0)};
    std::vector<Cluster> result;
    
    result = Clustering::performClustering(dataset, polarCalculation, result);

    // display the result of clustering
    for (int i = 0; i < result.size(); i++)
    {
        std::cout << "Cluster n°" << i + 1 << std::endl;
        std::cout << "first : theta= " << result.at(i).getFirstPoint().getAngle() << "°, distance= " << result.at(i).getFirstPoint().getDistance() << "cm" << std::endl;
        std::cout << "closest : theta= " << result.at(i).getClosestPoint().getAngle() << "°, distance= " << result.at(i).getClosestPoint().getDistance() << "cm" << std::endl;
        std::cout << "last : theta= " << result.at(i).getLastPoint().getAngle() << "°, distance= " << result.at(i).getLastPoint().getDistance() << "cm" << std::endl;

        std::cout << "Number of Points : " << result.at(i).getPointsVector().size() << std::endl;
        for (int j=0; j<result.at(i).getPointsVector().size(); j++)
        {
            std::cout << "Point n°" << j+1 << " : theta= " << result.at(i).getPointsVector().at(j).getAngle() << "°, distance= " << result.at(i).getPointsVector().at(j).getDistance() << "cm" << std::endl;
        }
    }
    return 0;
}
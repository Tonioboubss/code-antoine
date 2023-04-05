#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_session.hpp>

#include <vector>
#include <iostream>
#define private public
#include "../includes/fakeLidar.h"
#undef private

TEST_CASE("Test createFakeCluster method")
{
    SECTION("")
    {

    }
}

TEST_CASE("Test generatePointsVector method")
{
    Point first, last, closest;
    int nbPointsToCreate;
    float  maxDistance;
    std::vector<Point> result;
    FakeLidar *lidar = new FakeLidar("Lidar", sensorNavigate);
    SECTION("Points in 1st and 2nd quadrants")
    {
        first = Point(358.2, 236);
        closest = Point(56, 112);
        last = Point(111, 200);
        nbPointsToCreate = 58;
        maxDistance = 300;

        result = lidar->generatePoints(first, last, closest, maxDistance, nbPointsToCreate);

        REQUIRE(result.size() == nbPointsToCreate + 3);
        int i=0;
        for (const Point &point : result)
        {
            if (i>0)
                REQUIRE(first < point);
            REQUIRE(point.getDistance() >= closest.getDistance());
            REQUIRE(point.getDistance() <= maxDistance);
            if (i<result.size()-1)
                REQUIRE(point < last);
            i++;
        }
    }
}

TEST_CASE("Test getEnvironnementValue method")
{
    FakeLidar *lidar = new FakeLidar("Lidar", sensorNavigate);
    std::vector<Point> points;
    Cluster cluster;
    std::vector<Cluster> lastFrameDetections;
    std::vector<float> resultVec, validationVec(25, 0);

    SECTION("Object close in the two first zones")
    {
        lidar->setMode(FRONT_RIGHT_CLOSE);
        points = {Point(2,200), Point(5,190), Point(12,87), Point(19,165), Point(28,235)};
        cluster = Cluster(points);
        lidar->lastFrameDetections_.push_back(cluster);
        resultVec = lidar->getEnvironnementValue();

        validationVec.at(1) = 0.87;
        validationVec.at(2) = 1.65;
        REQUIRE(resultVec == validationVec);
    }
}

TEST_CASE("Test getClusters method")
{
    SECTION("")
    {
        
    }
}

TEST_CASE("Test checkLastValue method")
{
    SECTION("")
    {
        
    }
}


int main(int argc, char *argv[])
{
    // your setup code to run before catch2 starts.

    int result = Catch::Session().run(argc, argv);

    // your clean-up code to run after catch2 finishes.

    return result;
}
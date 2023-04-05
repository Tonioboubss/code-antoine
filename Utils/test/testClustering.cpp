#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_session.hpp>

#include <vector>
#include <iostream>

#define private public
#include "../includes/Clustering.h"
#include "../includes/Cluster.h"
#include "../includes/Point.h"
#undef private

TEST_CASE("Test == operator Point overload method")
{
    Point point1, point2;

    SECTION("Both points are the same")
    {
        point1 = Point(350, 103);
        point2 = Point(350, 103);
        REQUIRE(point1 == point2);
    }

    SECTION("Both points are different")
    {
        point1 = Point(350, 103);
        point2 = Point(40, 351);
        REQUIRE(!(point1 == point2));
    }
    SECTION("Two vectors of points of same size but last point is not the same")
    {
        point1 = Point(350, 103);
        point2 = Point(40, 351);
        std::vector<Point> vec1 = {point1, point2, point1};
        std::vector<Point> vec2 = {point1, point2, point2};
        REQUIRE(vec1 != vec2);
    }
    SECTION("Identical vectors of points")
    {
        point1 = Point(350, 103);
        point2 = Point(40, 351);
        std::vector<Point> vec1 = {point1, point2, point1};
        std::vector<Point> vec2 = {point1, point2, point1};
        REQUIRE(vec1 == vec2);
    }
}

TEST_CASE("Test == operator Cluster overload method")
{
    Point point1, point2, point3, point4;
    Cluster cluster1, cluster2;
    std::vector<Point> pointsVec1, pointsVec2;
    SECTION("Two identical clusters")
    {
        point1 = Point(152, 254);
        pointsVec1.push_back(point1);
        cluster1 = Cluster(pointsVec1);
        cluster2 = Cluster(pointsVec1);

        REQUIRE(cluster1 == cluster2);
    }

    SECTION("Two different clusters")
    {
        point1 = Point(152, 254);
        pointsVec1.push_back(point1);
        cluster1 = Cluster(pointsVec1);

        point2 = Point(25, 52);
        pointsVec2.push_back(point2);
        cluster2 = Cluster(pointsVec2);

        REQUIRE(!(cluster1 == cluster2));
    }

    SECTION("Two same clusters : same first, closest, last points the same but different clusters vector")
    {
        point1 = Point(152, 254);
        point2 = Point(210, 180);
        point3 = Point(170, 190);
        point4 = Point(190, 230);

        pointsVec1.push_back(point1);
        pointsVec1.push_back(point3);
        pointsVec1.push_back(point4);
        pointsVec1.push_back(point2);
        cluster1 = Cluster(pointsVec1);

        pointsVec2.push_back(point1);
        pointsVec2.push_back(point3);
        pointsVec2.push_back(point2);
        cluster2 = Cluster(pointsVec2);

        REQUIRE(cluster1 == cluster2);
        REQUIRE(cluster1.getAllPoints() != cluster2.getAllPoints());
    }

    std::vector<Cluster> clustersVec1, clustersVec2;

    SECTION("Two identical vectors of clusters")
    {
        point1 = Point(152, 254);
        pointsVec1.push_back(point1);
        cluster1 = Cluster(pointsVec1);

        point2 = Point(25, 52);
        pointsVec2.push_back(point2);
        cluster2 = Cluster(pointsVec2);

        clustersVec1.push_back(cluster1);
        clustersVec1.push_back(cluster2);
        clustersVec2.push_back(cluster1);
        clustersVec2.push_back(cluster2);

        REQUIRE(clustersVec1 == clustersVec2);
    }

    SECTION("Two clusters vector of same size but not same clusters")
    {
        point1 = Point(152, 254);
        pointsVec1.push_back(point1);
        cluster1 = Cluster(pointsVec1);

        point2 = Point(25, 52);
        pointsVec2.push_back(point2);
        cluster2 = Cluster(pointsVec2);

        clustersVec1.push_back(cluster1);
        clustersVec1.push_back(cluster2);
        clustersVec2.push_back(cluster2);
        clustersVec2.push_back(cluster1);

        REQUIRE(clustersVec1 != clustersVec2);
    }
}

TEST_CASE("Test isBeforeRefPoint method : Test if a point should be added before or after its reference point")
{
    Point refPoint, newPoint;
    bool isBeforeResult;

    SECTION("Both points in 1st Quadrant : the new point might be located after reference one")
    {
        refPoint = Point(15, 200);
        newPoint = Point(28, 30);
        isBeforeResult = newPoint < refPoint;
        REQUIRE(isBeforeResult == false);
    }

    SECTION("Both points in 1st Quadrant : the new point might be located before reference one")
    {
        refPoint = Point(45, 55);
        newPoint = Point(10, 800);
        isBeforeResult = newPoint < refPoint;
        REQUIRE(isBeforeResult == true);
    }

    SECTION("The two points are on both sides of 0° axis #overlap : the new point might be located before reference one")
    {
        refPoint = Point(5, 200);
        newPoint = Point(355, 411);
        isBeforeResult = newPoint < refPoint;
        REQUIRE(isBeforeResult == true);
    }

    SECTION("The two points are on both sides of 0° axis #overlap : the new point might be located after reference one")
    {
        refPoint = Point(350, 200);
        newPoint = Point(5, 411);
        isBeforeResult = newPoint < refPoint;
        REQUIRE(isBeforeResult == false);
    }

    SECTION("Both points in last Quadrant : the new point might be located after reference one")
    {
        refPoint = Point(310, 252);
        newPoint = Point(343, 125);
        isBeforeResult = newPoint < refPoint;
        REQUIRE(isBeforeResult == false);
    }

    SECTION("Both points in last Quadrant : the new point might be located before reference one")
    {
        refPoint = Point(338, 55);
        newPoint = Point(316, 800);
        isBeforeResult = newPoint < refPoint;
        REQUIRE(isBeforeResult == true);
    }

    SECTION("The two points are located in two opposite quadrants : the new point might be located before reference one")
    {
        refPoint = Point(10, 777);
        newPoint = Point(250, 1000);
        isBeforeResult = newPoint < refPoint;
        REQUIRE(isBeforeResult == true);
    }
    SECTION("The two points are located in two opposite quadrants : the new point might be located after reference one")
    {
        refPoint = Point(355, 300);
        newPoint = Point(170, 286);
        isBeforeResult = newPoint < refPoint;
        REQUIRE(isBeforeResult == false);
    }

    SECTION("The two points have the same angle but reference has got the lowest distance from origin : the new point might be located after reference one")
    {
        refPoint = Point(136, 503);
        newPoint = Point(136, 602);
        isBeforeResult = newPoint < refPoint;
        REQUIRE(isBeforeResult == false);
    }

    SECTION("The two points have the same angle but the new point has got the lowest distance from origin : the new point might be located before reference one")
    {
        refPoint = Point(264, 1163);
        newPoint = Point(264, 1010);
        isBeforeResult = newPoint < refPoint;
        REQUIRE(isBeforeResult == true);
    }

    SECTION("The two points are oppose, their angle difference is exactly 180° : By default the one with smallest distance from origin is before")
    {
        refPoint = Point(310, 1200);
        newPoint = Point(130, 1163);
        isBeforeResult = newPoint < refPoint;
        REQUIRE(isBeforeResult == true);
    }

    SECTION("The two points are oppose, their angle difference is exactly 180° : By default the one with smallest distance from origin is before.")
    {
        refPoint = Point(233, 100);
        newPoint = Point(53, 230);
        isBeforeResult = newPoint < refPoint;
        REQUIRE(isBeforeResult == false);
    }
}

TEST_CASE("Test getTolerance method : compute the threshold on Distance between two points")
{
    float distanceRef, resultTolerance;
    Cluster cluster;

    SECTION("At 15cm : is around 4.5mm ?")
    {
        distanceRef = 15.0;
        resultTolerance = cluster.getTolerance(distanceRef);
        REQUIRE(resultTolerance < 0.5);
        REQUIRE(resultTolerance > 0.4);
    }

    SECTION("At 300cm : is around 9cm ?")
    {
        distanceRef = 300.0;
        resultTolerance = cluster.getTolerance(distanceRef);
        REQUIRE(resultTolerance < 10);
        REQUIRE(resultTolerance > 8);
    }

    SECTION("At 600cm : is around 18cm ?")
    {
        distanceRef = 600.0;
        resultTolerance = cluster.getTolerance(distanceRef);
        REQUIRE(resultTolerance < 19);
        REQUIRE(resultTolerance > 17);
    }

    SECTION("At 1200cm : is around 36cm ?")
    {
        distanceRef = 1200.0;
        resultTolerance = cluster.getTolerance(distanceRef);
        REQUIRE(resultTolerance < 37);
        REQUIRE(resultTolerance > 35);
    }
}

TEST_CASE("Test computeAnglesDeltaTolerance method : compute the threshold on angles Difference between two points")
{
    float anglesDiffTol = 1.7;
    float distanceRef, resultTolerance;
    Cluster cluster;

    SECTION("At 15cm : is around 1.7° ?")
    {
        distanceRef = 15.0;
        resultTolerance = cluster.computeAnglesDeltaTolerance(distanceRef, distanceRef);
        REQUIRE(resultTolerance < anglesDiffTol + 0.1);
        REQUIRE(resultTolerance > anglesDiffTol - 0.1);
    }

    SECTION("At 300cm : is around 1.7° ?")
    {
        distanceRef = 300.0;
        resultTolerance = cluster.computeAnglesDeltaTolerance(distanceRef, distanceRef);
        REQUIRE(resultTolerance < anglesDiffTol + 0.1);
        REQUIRE(resultTolerance > anglesDiffTol - 0.1);
    }

    SECTION("At 600cm : is around 1.7° ?")
    {
        distanceRef = 600.0;
        resultTolerance = cluster.computeAnglesDeltaTolerance(distanceRef, distanceRef);
        REQUIRE(resultTolerance < anglesDiffTol + 0.1);
        REQUIRE(resultTolerance > anglesDiffTol - 0.1);
    }

    SECTION("At 1200cm : is around 1.7° ?")
    {
        distanceRef = 1200.0;
        resultTolerance = cluster.computeAnglesDeltaTolerance(distanceRef, distanceRef);
        REQUIRE(resultTolerance < anglesDiffTol + 0.1);
        REQUIRE(resultTolerance > anglesDiffTol - 0.1);
    }
}

TEST_CASE("Test checkMembershipWithDistanceCalculation method : check if a point is close enough of another point.")
{
    Cluster cluster;
    bool isMemberResult;
    bool isMemberValidation;
    float angleRef, distanceRef;
    Point refPoint, testedPoint;

    SECTION("Belong to cluster member 15cm")
    {
        angleRef = 201;
        distanceRef = 15;
        refPoint = Point(angleRef, distanceRef);
        cluster = Cluster(refPoint);

        testedPoint = Point(angleRef, distanceRef - 0.75 * cluster.getTolerance(distanceRef));
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == true);

        testedPoint = Point(angleRef, distanceRef + 2 * cluster.getTolerance(distanceRef));
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == false);

        testedPoint = Point(angleRef + 2 * cluster.computeAnglesDeltaTolerance(distanceRef, distanceRef), distanceRef);
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == false);

        testedPoint = Point(angleRef - 0.75 * cluster.computeAnglesDeltaTolerance(distanceRef, distanceRef), distanceRef);
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == true);
    }

    SECTION("Belong to cluster member 100cm")
    {
        angleRef = 302;
        distanceRef = 100;
        refPoint = Point(angleRef, distanceRef);
        cluster = Cluster(refPoint);

        testedPoint = Point(angleRef, distanceRef + 0.75 * cluster.getTolerance(distanceRef));
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == true);

        testedPoint = Point(angleRef, distanceRef - 2 * cluster.getTolerance(distanceRef));
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == false);

        testedPoint = Point(angleRef - 2 * cluster.computeAnglesDeltaTolerance(distanceRef, distanceRef), distanceRef);
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == false);

        testedPoint = Point(angleRef + 0.75 * cluster.computeAnglesDeltaTolerance(distanceRef, distanceRef), distanceRef);
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == true);
    }

    SECTION("Belong to cluster member 300cm")
    {
        angleRef = 359;
        distanceRef = 300;
        refPoint = Point(angleRef, distanceRef);
        cluster = Cluster(refPoint);

        testedPoint = Point(angleRef, distanceRef - 0.75 * cluster.getTolerance(distanceRef));
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == true);

        testedPoint = Point(angleRef, distanceRef + 2 * cluster.getTolerance(distanceRef));
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == false);

        testedPoint = Point(angleRef + 2 * cluster.computeAnglesDeltaTolerance(distanceRef, distanceRef), distanceRef);
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == false);

        testedPoint = Point(angleRef - 0.75 * cluster.computeAnglesDeltaTolerance(distanceRef, distanceRef), distanceRef);
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == true);
    }

    SECTION("Belong to cluster member 600cm")
    {
        angleRef = 106;
        distanceRef = 600;
        refPoint = Point(angleRef, distanceRef);
        cluster = Cluster(refPoint);

        testedPoint = Point(angleRef, distanceRef + 0.75 * cluster.getTolerance(distanceRef));
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == true);

        testedPoint = Point(angleRef, distanceRef - 2 * cluster.getTolerance(distanceRef));
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == false);

        testedPoint = Point(angleRef - 2 * cluster.computeAnglesDeltaTolerance(distanceRef, distanceRef), distanceRef);
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == false);

        testedPoint = Point(angleRef + 0.75 * cluster.computeAnglesDeltaTolerance(distanceRef, distanceRef), distanceRef);
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == true);
    }

    SECTION("Belong to cluster member 900cm")
    {
        angleRef = 106;
        distanceRef = 900;
        refPoint = Point(angleRef, distanceRef);
        cluster = Cluster(refPoint);

        testedPoint = Point(angleRef, distanceRef - 0.75 * cluster.getTolerance(distanceRef));
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == true);

        testedPoint = Point(angleRef, distanceRef + 2 * cluster.getTolerance(distanceRef));
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == false);

        testedPoint = Point(angleRef + 2 * cluster.computeAnglesDeltaTolerance(distanceRef, distanceRef), distanceRef);
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == false);

        testedPoint = Point(angleRef - 0.75 * cluster.computeAnglesDeltaTolerance(distanceRef, distanceRef), distanceRef);
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == true);
    }

    SECTION("Belong to cluster member 1200cm")
    {
        angleRef = 54;
        distanceRef = 1200;
        refPoint = Point(angleRef, distanceRef);
        cluster = Cluster(refPoint);

        testedPoint = Point(angleRef, distanceRef + 0.75 * cluster.getTolerance(distanceRef));
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == true);

        testedPoint = Point(angleRef, distanceRef - 2 * cluster.getTolerance(distanceRef));
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == false);

        testedPoint = Point(angleRef - 2 * cluster.computeAnglesDeltaTolerance(distanceRef, distanceRef), distanceRef);
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == false);

        testedPoint = Point(angleRef + 0.75 * cluster.computeAnglesDeltaTolerance(distanceRef, distanceRef), distanceRef);
        isMemberResult = cluster.checkMembershipWithDistanceCalculation(0, testedPoint);
        REQUIRE(isMemberResult == true);
    }
}

TEST_CASE("Test findPositionInClusterVec method : find the position of a point in a vector")
{
    std::vector<Point> clusterPoints = {Point(342, 1000), Point(351, 930), Point(3, 950), Point(10, 980), Point(20, 900), Point(23, 840), Point(30, 900)};
    Cluster cluster = Cluster(clusterPoints);
    int refPointIndex, resultPosition;
    ;
    Point newPoint;

    SECTION("position just before ref point")
    {
        refPointIndex = 2;
        newPoint = Point(357, 950);
        resultPosition = cluster.findPositionInClusterVec(refPointIndex, newPoint);
        REQUIRE(resultPosition == 2);
    }

    SECTION("position just after ref point")
    {
        refPointIndex = 3;
        newPoint = Point(12, 950);
        resultPosition = cluster.findPositionInClusterVec(refPointIndex, newPoint);
        REQUIRE(resultPosition == 4);
    }

    SECTION("position three places before ref point")
    {
        refPointIndex = 5;
        newPoint = Point(357, 950);
        resultPosition = cluster.findPositionInClusterVec(refPointIndex, newPoint);
        REQUIRE(resultPosition == 2);
    }

    SECTION("position two index after ref point")
    {
        refPointIndex = 0;
        newPoint = Point(355, 950);
        resultPosition = cluster.findPositionInClusterVec(refPointIndex, newPoint);
        REQUIRE(resultPosition == 2);
    }

    SECTION("position at the beginning of cluster vector")
    {
        refPointIndex = 6;
        newPoint = Point(338, 950);
        resultPosition = cluster.findPositionInClusterVec(refPointIndex, newPoint);
        REQUIRE(resultPosition == 0);
    }

    SECTION("position at the end of cluster vector")
    {
        refPointIndex = 1;
        newPoint = Point(50, 950);
        resultPosition = cluster.findPositionInClusterVec(refPointIndex, newPoint);
        REQUIRE(resultPosition == 7);
    }

    SECTION("refPoint out of bound : negative integer")
    {
        refPointIndex = -1;
        newPoint = Point(9, 950);
        REQUIRE_THROWS_AS(cluster.findPositionInClusterVec(refPointIndex, newPoint), ExcepUnknownRefPoint);
    }

    SECTION("refPoint out of bound : too big")
    {
        refPointIndex = cluster.getAllPoints().size() + 1;
        newPoint = Point(349, 950);
        REQUIRE_THROWS_AS(cluster.findPositionInClusterVec(refPointIndex, newPoint), ExcepUnknownRefPoint);
    }
}

TEST_CASE("Test insertPoint method : add a point in the cluster vector at the desired position.")
{
    std::vector<Point> dataset = {Point(10, 526), Point(59, 255), Point(163, 362), Point(266, 824), Point(312, 614)};
    Cluster cluster = Cluster(dataset);

    Point newPoint = Point(5, 253);

    std::vector<Point> validationVec;

    SECTION("The new Point might not be added to the cluster")
    {
        cluster = Cluster(dataset);
        validationVec = dataset;

        cluster.insertPoint(-1, newPoint);
        REQUIRE(cluster.getAllPoints() == validationVec);
        cluster.insertPoint(6, newPoint);
        REQUIRE(cluster.getAllPoints() == validationVec);
    }

    SECTION("The new Point might be added in a empty cluster")
    {
        dataset = {};
        cluster = Cluster();
        validationVec.push_back(newPoint);
        cluster.insertPoint(0, newPoint);
        REQUIRE(cluster.getAllPoints() == validationVec);
    }

    SECTION("The new Point might be added before the first Point of the vector")
    {
        validationVec = {newPoint, Point(10, 526), Point(59, 255), Point(163, 362), Point(266, 824), Point(312, 614)};
        cluster.insertPoint(0, newPoint);
        REQUIRE(cluster.getAllPoints() == validationVec);
    }

    SECTION("The new Point might be added before the 4th point")
    {
        validationVec = {Point(10, 526), Point(59, 255), Point(163, 362), newPoint, Point(266, 824), Point(312, 614)};
        cluster.insertPoint(3, newPoint);
        REQUIRE(cluster.getAllPoints() == validationVec);
    }

    SECTION("The new Point might be added before the last point of the vector")
    {
        validationVec = {Point(10, 526), Point(59, 255), Point(163, 362), Point(266, 824), newPoint, Point(312, 614)};
        cluster.insertPoint(cluster.getAllPoints().size() - 1, newPoint);
        REQUIRE(cluster.getAllPoints() == validationVec);
    }

    SECTION("The new Point might be added at the end of the vector")
    {
        validationVec = {Point(10, 526), Point(59, 255), Point(163, 362), Point(266, 824), Point(312, 614), newPoint};
        cluster.insertPoint(5, newPoint);
        REQUIRE(cluster.getAllPoints() == validationVec);
    }
}

TEST_CASE("Test updateVecOfIndexes method : find the place of a point index in a vector of index and update the vector of indexes")
{
    std::vector<int> addedPointsIndexes = {1, 5, 20, 36, 51};
    int newPointIndex;
    std::vector<int> validationVec;
    Cluster cluster;

    SECTION("Might be the first index")
    {
        newPointIndex = 0;
        validationVec = {newPointIndex, 2, 6, 21, 37, 52};

        cluster.updateVecOfIndexes(newPointIndex, addedPointsIndexes);
        REQUIRE(addedPointsIndexes == validationVec);
    }

    SECTION("Might be the third index")
    {
        newPointIndex = 11;
        validationVec = {1, 5, newPointIndex, 21, 37, 52};

        cluster.updateVecOfIndexes(newPointIndex, addedPointsIndexes);
        REQUIRE(addedPointsIndexes == validationVec);
    }

    SECTION("Might be the last index")
    {
        newPointIndex = 60;
        validationVec = {1, 5, 20, 36, 51, newPointIndex};

        cluster.updateVecOfIndexes(newPointIndex, addedPointsIndexes);
        REQUIRE(addedPointsIndexes == validationVec);
    }
}

pointComparingMethod methodToComparePoints = DISTANCE_CALCULATION;

TEST_CASE("Test fill method : add points at the right place in the cluster if they are close enough to the cluster reference point.")
{
    std::vector<Point> dataset, inputDataset;
    Cluster cluster;

    SECTION("The newPoints do not belong to the cluster case")
    {
        inputDataset = {};
        cluster.fill(inputDataset, methodToComparePoints);

        REQUIRE(cluster.getAllPoints() == inputDataset);
    }
    Point point1, point2, point3, point4, point5;
    std::vector<Point> validationVec, validationRestVec;

    SECTION("The points all belong to the cluster")
    {
        point1 = Point(230, 620);
        point2 = Point(231, 620);
        point3 = Point(232, 620);
        point4 = Point(233, 620);
        point5 = Point(234, 620);

        inputDataset = {point1, point2, point3, point4, point5};
        validationVec = inputDataset;
        cluster.fill(inputDataset, methodToComparePoints);

        REQUIRE(cluster.getAllPoints() == validationVec);
        REQUIRE(inputDataset.empty() == true);
    }

    SECTION("Three points belong to the cluster and two do not")
    {
        point1 = Point(230, 620);
        point5 = Point(231, 620);
        point3 = Point(232, 620);
        point4 = Point(245, 620);
        point2 = Point(240, 620);

        inputDataset = {point1, point2, point3, point4, point5};
        validationVec = {point1, point5, point3};
        validationRestVec = {point2, point4};
        cluster.fill(inputDataset, methodToComparePoints);

        REQUIRE(cluster.getAllPoints() == validationVec);
        REQUIRE(inputDataset == validationRestVec);
    }

    SECTION("All points belong to the cluster but they come unsorted")
    {
        point1 = Point(230, 620);
        point2 = Point(231, 620);
        point3 = Point(229, 620);
        point4 = Point(228, 620);
        point5 = Point(232, 620);

        inputDataset = {point1, point2, point3, point4, point5};
        validationVec = {point4, point3, point1, point2, point5};
        validationRestVec = {};
        cluster.fill(inputDataset, methodToComparePoints);

        REQUIRE(cluster.getAllPoints() == validationVec);
        REQUIRE(inputDataset == validationRestVec);
    }

    SECTION("All points belong to the cluster but they come unsorted")
    {
        point1 = Point(230, 620);
        point2 = Point(231, 615);
        point3 = Point(229, 625);
        point4 = Point(232.5, 613);
        point5 = Point(227.5, 622);

        inputDataset = {point1, point4, point3, point5, point2};
        validationVec = {point5, point3, point1, point2, point4};
        validationRestVec = {};
        cluster.fill(inputDataset, methodToComparePoints);

        REQUIRE(cluster.getAllPoints() == validationVec);
        REQUIRE(inputDataset == validationRestVec);
    }
    Point point6, point7, point8, point9, point10;
    SECTION("All points belong to the cluster but they come unsorted")
    {
        point1 = Point(20, 1000);
        point2 = Point(19, 1013);
        point3 = Point(20.3, 985);
        point4 = Point(18, 1045);
        point5 = Point(16.9, 952);
        point6 = Point(17, 1060);
        point7 = Point(18, 970);
        point8 = Point(18.5, 1022);
        point9 = Point(18.7, 982);
        point10 = Point(21.5, 1005);

        inputDataset = {point1, point2, point3, point4, point5, point6, point7, point8, point9, point10};
        validationVec = {point5, point6, point7, point4, point8, point9, point2, point1, point3, point10};
        validationRestVec = {};
        cluster.fill(inputDataset, methodToComparePoints);

        REQUIRE(cluster.getAllPoints() == validationVec);
        REQUIRE(inputDataset == validationRestVec);

        inputDataset = {point10, point3, point9, point1, point4, point7, point5, point2, point6, point8};
        Cluster cluster2 = Cluster();
        cluster2.fill(inputDataset, methodToComparePoints);
        REQUIRE(cluster2.getAllPoints() == validationVec);
        REQUIRE(inputDataset == validationRestVec);

        inputDataset = {point7, point8, point10, point2, point6, point1, point9, point3, point4, point5};
        Cluster cluster3 = Cluster();
        cluster3.fill(inputDataset, methodToComparePoints);
        REQUIRE(cluster3.getAllPoints() == validationVec);
        REQUIRE(inputDataset == validationRestVec);
    }
}

TEST_CASE("Test method performClustering : gathering point from one or different dataset STEP")
{
    std::vector<Point> dataset;
    Point point1, point2, point3;

    std::vector<Cluster> resultClustersVec;
    std::vector<Point> resultPointsVec;

    Point validationfirstPoint, validationClosestPoint, validationLastPoint;
    std::vector<Point> validationPointsVector;
    std::vector<Cluster> validationClustersVector;
    Cluster validationCluster;

    SECTION("Input : 3 sorted points (according to angle) which belong to same cluster")
    {
        point1 = Point(76.0, 100);
        point2 = Point(77.0, 100);
        point3 = Point(78.0, 100);
        dataset = {point1, point2, point3};

        resultClustersVec = Clustering::performClustering(dataset, methodToComparePoints);

        validationCluster = Cluster(dataset);
        validationClustersVector.push_back(validationCluster);

        REQUIRE(resultClustersVec == validationClustersVector);
        REQUIRE(resultClustersVec.front().getAllPoints() == validationCluster.getAllPoints());
    }

    SECTION("Input : 3 unsorted points which belong to same cluster")
    {
        point1 = Point(76.0, 100);
        point2 = Point(77.0, 100);
        point3 = Point(78.0, 100);
        dataset = {point1, point3, point2};

        resultClustersVec = Clustering::performClustering(dataset, methodToComparePoints);

        validationCluster = Cluster({point1, point2, point3});
        validationClustersVector.push_back(validationCluster);

        REQUIRE(resultClustersVec == validationClustersVector);
        REQUIRE(resultClustersVec.front().getAllPoints() == validationCluster.getAllPoints());
    }

    SECTION("Input : 3 unsorted points which belong to same cluster")
    {
        point1 = Point(76.0, 100);
        point2 = Point(77.0, 100);
        point3 = Point(78.0, 100);
        dataset = {point3, point2, point1};

        resultClustersVec = Clustering::performClustering(dataset, methodToComparePoints);

        validationCluster = Cluster({point1, point2, point3});
        validationClustersVector.push_back(validationCluster);

        REQUIRE(resultClustersVec == validationClustersVector);
        REQUIRE(resultClustersVec.front().getAllPoints() == validationCluster.getAllPoints());
    }

    SECTION("Input : 3 unsorted points which belong to same cluster")
    {
        point1 = Point(76.0, 100);
        point2 = Point(77.0, 100);
        point3 = Point(78.0, 100);
        dataset = {point2, point3, point1};

        resultClustersVec = Clustering::performClustering(dataset, methodToComparePoints);

        validationCluster = Cluster({point1, point2, point3});
        validationClustersVector.push_back(validationCluster);

        REQUIRE(resultClustersVec == validationClustersVector);
        REQUIRE(resultClustersVec.front().getAllPoints() == validationCluster.getAllPoints());
    }

    SECTION("Input : 3 unsorted points which belong to same cluster")
    {
        point1 = Point(76.0, 100);
        point2 = Point(77.0, 100);
        point3 = Point(78.0, 100);
        dataset = {point2, point1, point3};

        resultClustersVec = Clustering::performClustering(dataset, methodToComparePoints);

        validationCluster = Cluster({point1, point2, point3});
        validationClustersVector.push_back(validationCluster);

        REQUIRE(resultClustersVec == validationClustersVector);
        REQUIRE(resultClustersVec.front().getAllPoints() == validationCluster.getAllPoints());
    }

    SECTION("Input : 3 sorted points (according to angle) which belong to same cluster")
    {
        point1 = Point(1, 800);
        point2 = Point(358, 800);
        point3 = Point(359.5, 800);
        dataset = {point1, point2, point3};

        resultClustersVec = Clustering::performClustering(dataset, methodToComparePoints);

        validationCluster = Cluster({point2, point3, point1});
        validationClustersVector.push_back(validationCluster);

        REQUIRE(resultClustersVec == validationClustersVector);
        REQUIRE(resultClustersVec.front().getAllPoints() == validationCluster.getAllPoints());
    }

    SECTION("Input : 3 unsorted points which belong to same cluster")
    {
        point1 = Point(1, 800);
        point2 = Point(358, 800);
        point3 = Point(359.5, 800);
        dataset = {point1, point3, point2};

        resultClustersVec = Clustering::performClustering(dataset, methodToComparePoints);

        validationCluster = Cluster({point2, point3, point1});
        validationClustersVector.push_back(validationCluster);

        REQUIRE(resultClustersVec == validationClustersVector);
        REQUIRE(resultClustersVec.front().getAllPoints() == validationCluster.getAllPoints());
    }

    SECTION("Input : 3 unsorted points which belong to same cluster")
    {
        point1 = Point(1, 800);
        point2 = Point(358, 800);
        point3 = Point(359.5, 800);
        dataset = {point3, point1, point2};

        resultClustersVec = Clustering::performClustering(dataset, methodToComparePoints);

        validationCluster = Cluster({point2, point3, point1});
        validationClustersVector.push_back(validationCluster);

        REQUIRE(resultClustersVec == validationClustersVector);
        REQUIRE(resultClustersVec.front().getAllPoints() == validationCluster.getAllPoints());
    }

    SECTION("Input : 3 unsorted points which belong to same cluster")
    {
        point1 = Point(1, 800);
        point2 = Point(358, 800);
        point3 = Point(359.5, 800);
        dataset = {point2, point1, point3};

        resultClustersVec = Clustering::performClustering(dataset, methodToComparePoints);

        validationCluster = Cluster({point2, point3, point1});
        validationClustersVector.push_back(validationCluster);

        REQUIRE(resultClustersVec == validationClustersVector);
        REQUIRE(resultClustersVec.front().getAllPoints() == validationCluster.getAllPoints());
    }

    Point point4, point5, point6, point7, point8, point9, point10;
    std::vector<Point> points;

    // Cluster 1
    point2 = Point(10.0, 1200.0);
    point3 = Point(11.0, 1200.0);
    point4 = Point(12.0, 1200.0);
    // Cluster 2
    point5 = Point(268.0, 600.0);
    point6 = Point(269.0, 600.0);
    point7 = Point(270.0, 600.0);
    // Cluster 3
    point8 = Point(359, 50.0);
    point9 = Point(359.4, 50.0);
    point10 = Point(0.1, 50.0);
    // Cluster 4
    point1 = Point(180.0, 300.0);
    Cluster validationCluster1, validationCluster2, validationCluster3, validationCluster4;
    SECTION("Input : 4 clusters to find from a single dataset")
    {
        dataset = {point4, point7, point9, point10, point6, point2, point3, point1, point8, point5};

        resultClustersVec = Clustering::performClustering(dataset, methodToComparePoints);

        points.push_back(point1);
        validationCluster1 = Cluster(points);
        validationCluster2 = Cluster({point2, point3, point4});
        validationCluster3 = Cluster({point5, point6, point7});
        validationCluster4 = Cluster({point8, point9, point10});
        validationClustersVector.push_back(validationCluster2);
        validationClustersVector.push_back(validationCluster3);
        validationClustersVector.push_back(validationCluster4);
        validationClustersVector.push_back(validationCluster1);

        REQUIRE(resultClustersVec == validationClustersVector);
        REQUIRE(resultClustersVec.front().getAllPoints() == validationCluster2.getAllPoints());
        REQUIRE(resultClustersVec.at(1).getAllPoints() == validationCluster3.getAllPoints());
        REQUIRE(resultClustersVec.at(2).getAllPoints() == validationCluster4.getAllPoints());
        REQUIRE(resultClustersVec.back().getAllPoints() == validationCluster1.getAllPoints());
    }

    std::vector<Point> points1, points2;

    SECTION("Input : 2 clusters composed of one point")
    {
        dataset = {point1, point2};

        resultClustersVec = Clustering::performClustering(dataset, methodToComparePoints);
        points1.push_back(point1);
        points2.push_back(point2);
        validationCluster1 = Cluster(points1);
        validationCluster2 = Cluster(points2);
        validationClustersVector.push_back(validationCluster1);
        validationClustersVector.push_back(validationCluster2);

        REQUIRE(resultClustersVec.front().getAllPoints() == validationCluster1.getAllPoints());
        REQUIRE(resultClustersVec.back().getAllPoints() == validationCluster2.getAllPoints());
        REQUIRE(resultClustersVec == validationClustersVector);
    }

    SECTION("Input : 2 spaced linear clusters of 5 points")
    {
        float startAngleC1 = 5.0;
        float startDistanceC1 = 300.0;
        float startAngleC2 = 9.0;
        float startDistanceC2 = 315.5;
        float angleStep = 1.0;

        // Cluster 1
        point1 = Point(startAngleC1, startDistanceC1);
        point2 = Point(startAngleC1 + angleStep, startDistanceC1);
        point3 = Point(startAngleC1 + 2 * angleStep, startDistanceC1);
        point4 = Point(startAngleC1 + 3 * angleStep, startDistanceC1);
        point5 = Point(startAngleC1 + 4 * angleStep, startDistanceC1);
        // Cluster 2
        point6 = Point(startAngleC2, startDistanceC2);
        point7 = Point(startAngleC2 + angleStep, startDistanceC2);
        point8 = Point(startAngleC2 + 2 * angleStep, startDistanceC2);
        point9 = Point(startAngleC2 + 3 * angleStep, startDistanceC2);
        point10 = Point(startAngleC2 + 4 * angleStep, startDistanceC2);

        dataset = {point8, point3, point1, point10, point7, point4, point9, point5, point6, point2};
        validationCluster2 = Cluster({point1, point2, point3, point4, point5});
        validationCluster1 = Cluster({point6, point7, point8, point9, point10});
        validationClustersVector.push_back(validationCluster1);
        validationClustersVector.push_back(validationCluster2);

        resultClustersVec = Clustering::performClustering(dataset, methodToComparePoints);

        REQUIRE(resultClustersVec == validationClustersVector);
        REQUIRE(resultClustersVec.front().getAllPoints() == validationCluster1.getAllPoints());
        REQUIRE(resultClustersVec.back().getAllPoints() == validationCluster2.getAllPoints());
    }
    SECTION("Points Cloud")
    {
        point2 = Point(359.0, 100.0);
        point3 = Point(358.0, 102.0);
        point4 = Point(0.2, 98.0);
        point5 = Point(1.0, 100.0);
        point6 = Point(1.4, 97.0);
        point7 = Point(357.2, 103.0);
        point8 = Point(359, 104.0);
        point9 = Point(359.4, 96.0);
        point10 = Point(0.1, 97.0);
        point1 = Point(2.0, 98.5);

        dataset = {point8, point3, point1, point10, point7, point4, point9, point5, point6, point2};
        validationCluster1 = Cluster({point7, point3, point2, point8, point9, point10, point4, point5, point6, point1});
        validationClustersVector.push_back(validationCluster1);
        resultClustersVec = Clustering::performClustering(dataset, methodToComparePoints);

        REQUIRE(resultClustersVec == validationClustersVector);
        REQUIRE(resultClustersVec.front().getAllPoints() == validationCluster1.getAllPoints());

        dataset = {point8, point3, point1, point10, point7, point4, point9, point5, point6, point2};
        resultClustersVec = Clustering::performClustering(dataset, methodToComparePoints);

        REQUIRE(resultClustersVec == validationClustersVector);
        REQUIRE(resultClustersVec.front().getAllPoints() == validationCluster1.getAllPoints());
    }
}

TEST_CASE("Test preprocessData method")
{
    std::vector<Point> inputVec, resultVec, validationVec;
    SECTION("Only Positive angles")
    {
        inputVec = {Point(5, 230), Point(520.7, 51), Point(359.6, 60), Point(0.2, 635), Point(1603.15, 704), Point(360, 57)};
        validationVec = {Point(5, 230), Point(160.7, 51), Point(359.6, 60), Point(0.2, 635), Point(163.15, 704), Point(0, 57)};
        resultVec = Clustering::preprocessData(inputVec);

        // REQUIRE(resultVec == validationVec);
        REQUIRE(resultVec.at(0) == validationVec.at(0));
        // REQUIRE(resultVec.at(1) == validationVec.at(1));
        REQUIRE(resultVec.at(2) == validationVec.at(2));
        REQUIRE(resultVec.at(3) == validationVec.at(3));
        // REQUIRE(resultVec.at(4) == validationVec.at(4));
        REQUIRE(resultVec.at(5) == validationVec.at(5));
    }
    SECTION("Only negative angles")
    {
        inputVec = {Point(0, 98), Point(-0.55, 51), Point(-360.2, 66), Point(-50, 986), Point(-504, 98), Point(-844, 100)};
        validationVec = {Point(0, 98), Point(359.45, 51), Point(359.8, 66), Point(310, 986), Point(216, 98), Point(236, 100)};
        ;
        resultVec = Clustering::preprocessData(inputVec);

        REQUIRE(resultVec == validationVec);
    }
}

int main(int argc, char *argv[])
{
    // your setup code to run before catch2 starts.

    int result = Catch::Session().run(argc, argv);

    // your clean-up code to run after catch2 finishes.

    return result;
}
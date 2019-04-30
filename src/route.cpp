
#include <sstream>
#include <fstream>
#include <iostream>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <algorithm>

#include "geometry.h"
#include "xmlparser.h"
#include "route.h"

using namespace GPS;

std::string Route::name() const
{
    return routeName.empty() ? "Unnamed Route" : routeName;
}

unsigned int Route::numPositions() const
{
    return (unsigned int)positions.size();
}

metres Route::totalLength() const
{
    // The total length of the Route; this is the sum of the distances between successive route points.
    return routeLength;
}

metres Route::netLength() const
{
    Position firstPosition = positions[0];
    Position lastPosition = positions[positions.size() - 1];

    if ( areSameLocation(firstPosition, lastPosition) )
    {
        return 0;
    }

    return Position::distanceBetween(firstPosition, lastPosition);
}

metres Route::totalHeightGain() const
{
    assert(! positions.empty());

    metres total = 0.0;
    for (unsigned int i = 1; i < numPositions(); ++i)
    {
        metres deltaV = positions[i].elevation() - positions[i-1].elevation();
        if (deltaV > 0.0) total += deltaV; // ignore negative height differences
    }
    return total;
}

metres Route::netHeightGain() const
{
    assert(! positions.empty());

    metres deltaV = positions.back().elevation() - positions.front().elevation();
    return std::max(deltaV,0.0); // ignore negative height differences
}

degrees Route::minLatitude() const
{
    if (positions.empty()) {
        throw std::out_of_range("Cannot get the minimum latitude of an empty route");
    }

    degrees lowestLatitude = positions[0].latitude();

    double epsilon = 0.0001;

    for (int i = 0; i < positions.size(); i++)
    {
        if ( (positions[i].latitude() - lowestLatitude) < epsilon)
        {
            lowestLatitude = positions[i].latitude();
        }
    }

    return lowestLatitude;
}

degrees Route::maxLatitude() const
{
    degrees currentMax = positions[0].latitude();

    for(int i = 0; i < positions.size(); i++){
        if(positions[i].latitude() > currentMax)
            currentMax = positions[i].latitude();
    }

    return currentMax;
}

degrees Route::minLongitude() const     //MY FUNCTION
{
    assert(! positions.empty());

    degrees minLon = positions.front().longitude();
    for (const Position& pos : positions)
    {
        minLon = std::min(minLon,pos.longitude());
    }
    return minLon;
}

degrees Route::maxLongitude() const
{
    assert(! positions.empty());

    degrees maxLon = positions.front().longitude();
    for (const Position& pos : positions)
    {
        maxLon = std::max(maxLon,pos.longitude());
    }
    return maxLon;

}

metres Route::minElevation() const
{
    assert(! positions.empty());

    degrees minEle = positions.front().elevation();
    for (const Position& pos : positions)
    {
        minEle = std::min(minEle,pos.elevation());
    }
    return minEle;
}

metres Route::maxElevation() const
{
    assert(! positions.empty());

    degrees maxEle = positions.front().elevation();
    for (const Position& pos : positions)
    {
        maxEle = std::max(maxEle,pos.elevation());
    }
    return maxEle;
}

degrees Route::maxGradient() const
{
    assert(! positions.empty());

    if (positions.size() == 1) return 0.0;

    degrees maxGrad = -halfRotation/2; // minimum possible value
    for (unsigned int i = 1; i < positions.size(); ++i)
    {
        metres deltaH = Position::distanceBetween(positions[i],positions[i-1]);
        metres deltaV = positions[i].elevation() - positions[i-1].elevation();
        degrees grad = radToDeg(std::atan(deltaV/deltaH));
        maxGrad = std::max(maxGrad,grad);
    }
    return maxGrad;
}

degrees Route::minGradient() const
{
    assert(! positions.empty());

    if (positions.size() == 1) return 0.0;

    degrees minGrad = halfRotation/2; // maximum possible value
    for (unsigned int i = 1; i < positions.size(); ++i)
    {
        metres deltaH = Position::distanceBetween(positions[i],positions[i-1]);
        metres deltaV = positions[i].elevation() - positions[i-1].elevation();
        degrees grad = radToDeg(std::atan(deltaV/deltaH));
        minGrad = std::min(minGrad,grad);
    }
    return minGrad;
}

degrees Route::steepestGradient() const
{
    assert(! positions.empty());

    if (positions.size() == 1) return 0.0;

    degrees maxGrad = -halfRotation/2; // minimum possible value
    for (unsigned int i = 1; i < positions.size(); ++i)
    {
        metres deltaH = Position::distanceBetween(positions[i],positions[i-1]);
        metres deltaV = positions[i].elevation() - positions[i-1].elevation();
        degrees grad = radToDeg(std::atan(deltaV/deltaH));
        maxGrad = std::max(maxGrad,std::abs(grad));
    }
    return maxGrad;
}

Position Route::operator[](unsigned int idx) const
{
    return positions.at(idx);
}

Position Route::findPosition(const std::string & soughtName) const
{
    auto nameIt = std::find(positionNames.begin(), positionNames.end(), soughtName);

    if (nameIt == positionNames.end())
    {
        throw std::out_of_range("No position with that name found in the route.");
    }
    else
    {
        return positions[std::distance(positionNames.begin(),nameIt)];
    }
}

std::string Route::findNameOf(const Position & soughtPos) const
{
    auto posIt = std::find_if(positions.begin(), positions.end(),
                              [&] (const Position& pos) {return areSameLocation(pos,soughtPos);});

    if (posIt == positions.end())
    {
        throw std::out_of_range("Position not found in route.");
    }
    else
    {
        return positionNames[std::distance(positions.begin(),posIt)];
    }
}

unsigned int Route::timesVisited(const std::string & soughtName) const
{
    unsigned int timesVisited{0};

    try{

        Position position = this->findPosition(soughtName);
        for (const auto &i: positions)
            if (areSameLocation(i, position)) timesVisited++;

    } catch(const std::out_of_range& e){}

    return timesVisited;
}

unsigned int Route::timesVisited(const Position & soughtPos) const
{
    unsigned int timesVisited{0};

    for (const auto &i: positions)
        if (areSameLocation(i, soughtPos)) timesVisited++;

    return timesVisited;
}

std::string Route::buildReport() const
{
    return report;
}

std::string checkErrors(std::string& gpsData){
    std::string newPostion;
    if (! XML::Parser::elementExists(gpsData,"rtept")) throw std::domain_error("No 'rtept' element.");
    newPostion = XML::Parser::getAndEraseElement(gpsData, "rtept");
    if (! XML::Parser::attributeExists(newPostion,"lat")) throw std::domain_error("No 'lat' attribute.");
    if (! XML::Parser::attributeExists(newPostion,"lon")) throw std::domain_error("No 'lon' attribute.");
    return newPostion;
}

Route::Route(std::string fileName, bool isFileName, metres granularity)
{
    std::string lat,lon,ele,name,line,newPostion;
    std::vector<std::string> elements ={"gpx","rte"};
    metres deltaH,deltaV;
    std::ostringstream oss,oss2;
    unsigned int num;
    this->granularity = granularity;
    std::string fileData;
    std::string gpsData;

    if (isFileName){
        std::ifstream file(fileName);
        if (! file.good()) throw std::invalid_argument("Error opening source file '" + fileName + "'.");
        oss << "Source file '" << fileName << "' opened okay." << std::endl;
        while (file.good()) {
            getline(file, line);
            oss2 << line << std::endl;
        }
        fileData = oss2.str();
    }

    for (int i = 0; i < elements.size(); ++i) {
        if (! XML::Parser::elementExists(fileData,elements[i])) throw std::domain_error("No '" + elements[i] + "' element.");
        gpsData = XML::Parser::getElementContent(XML::Parser::getElement(fileData, elements[i]));
    }

    if (XML::Parser::elementExists(gpsData, "name")) {
        routeName = XML::Parser::getElementContent(XML::Parser::getAndEraseElement(gpsData, "name"));
        oss << "Route name is: " << routeName << std::endl;
    }

    num = 0;

    while (XML::Parser::elementExists(gpsData, "rtept")) {
        newPostion = checkErrors(gpsData);
        lat = XML::Parser::getElementAttribute(newPostion, "lat");
        lon = XML::Parser::getElementAttribute(newPostion, "lon");
        newPostion = XML::Parser::getElementContent(newPostion);

        if (XML::Parser::elementExists(newPostion, "ele")) {
            ele = XML::Parser::getElementContent(XML::Parser::getElement(newPostion, "ele"));
            positions.push_back(Position(lat,lon,ele));
        } else {
            positions.push_back(Position(lat,lon));
        }

        if (positions.size() > 1 && areSameLocation(positions.back(), positions.at(positions.size()-2))){
            oss << "Position ignored: " << positions.back().toString() << std::endl;
            positions.pop_back();
        } else {
            if (XML::Parser::elementExists(newPostion,"name")) {
                name = XML::Parser::getElementContent(XML::Parser::getElement(newPostion,"name"));
            }
            positionNames.push_back(name);
            oss << "Position added: " << positions.back().toString() << std::endl;
            ++num;
        }

    }
    oss << num << " positions added." << std::endl;
    routeLength = 0;
    for (unsigned int i = 1; i < num; ++i ) {
        deltaH = Position::distanceBetween(positions[i-1], positions[i]);
        deltaV = positions[i-1].elevation() - positions[i].elevation();
        routeLength += sqrt(pow(deltaH,2) + pow(deltaV,2));
    }
    report = oss.str();
}

void Route::setGranularity(metres granularity)
{
    bool implemented = false;
    assert(implemented);
}

bool Route::areSameLocation(const Position & p1, const Position & p2) const
{
    return (Position::distanceBetween(p1,p2) < granularity);
}

#include <sstream>
#include <fstream>
#include <iostream>
#include <cassert>
#include <cmath>
#include <stdexcept>

#include "geometry.h"
#include "xmlparser.h"
#include "track.h"

using namespace GPS;

// Note: The implementation should exploit the relationship:
//   totalTime() == restingTime() + travellingTime()

seconds Track::totalTime() const
{
    assert(! departed.empty());
    return departed.back();
}

seconds Track::restingTime() const
{
    seconds total = 0;
    assert (arrived.size() == departed.size());
    for (unsigned int i = 0; i < arrived.size(); ++i)
    {
        total += departed[i] - arrived[i];
    }
    return total;
}

seconds Track::travellingTime() const
{
    return totalTime() - restingTime();
}

speed Track::maxSpeed() const
{
    assert( positions.size() == departed.size() && positions.size() == arrived.size() );
    if (positions.size() == 1) return 0.0;

    speed ms = 0;
    for (unsigned int i = 1; i < positions.size(); ++i)
    {
        metres deltaH = Position::distanceBetween(positions[i],positions[i-1]);
        metres deltaV = positions[i].elevation() - positions[i-1].elevation();
        metres distance = std::sqrt(std::pow(deltaH,2) + std::pow(deltaV,2));
        seconds time = arrived[i] - departed[i-1];
        ms = std::max(ms,distance/time);
    }
    return ms;
}

speed Track::averageSpeed(bool includeRests) const
{
    seconds time = (includeRests ? totalTime() : travellingTime());
    if (time == 0) return 0;
    else return totalLength() / time;
}

speed Track::maxRateOfAscent() const
{
    assert( positions.size() == departed.size() && positions.size() == arrived.size() );
    if (positions.size() == 1) return 0.0;

    speed ms = 0;
    for (unsigned int i = 1; i < positions.size(); ++i)
    {
        metres height = positions[i].elevation() - positions[i-1].elevation();
        seconds time = arrived[i] - departed[i-1];
        ms = std::max(ms,height/time);
    }
    return ms;
}

speed Track::maxRateOfDescent() const
{
    assert( positions.size() == departed.size() && positions.size() == arrived.size() );
    if (positions.size() == 1) return 0.0;

    speed ms = 0;
    for (unsigned int i = 1; i < positions.size(); ++i)
    {
        metres height = positions[i-1].elevation() - positions[i].elevation();
        seconds time = arrived[i] - departed[i-1];
        ms = std::max(ms,height/time);
    }
    return ms;
}

void Track::setGranularity(metres granularity)
{
    bool implemented = false;
    assert(implemented);
}

seconds Track::stringToTime(const std::string & timeStr)
{
    return stoull(timeStr);
}

seconds Track::getTime(std::string newPostion){
    if (! XML::Parser::elementExists(newPostion,"time"))
        throw std::domain_error("No 'time' element.");

    return stringToTime(XML::Parser::getElementContent(XML::Parser::getElement(newPostion,"time")));
}

void Track::addPostion(std::string newPostion){
    positions.push_back(getNewPostion(newPostion));
    seconds currentTime = getTime(newPostion);
    if (positions.size()>1 && areSameLocation(positions.back(), positions.at(positions.size()-2))) {
        // If we're still at the same location, then we haven't departed yet.
        departed.back() = currentTime;
        reportStringStream << "Position ignored: " << positions.back().toString() << std::endl;
        positions.pop_back();
    } else {
        positionNames.push_back(getName(newPostion));
        arrived.push_back(currentTime);
        departed.push_back(currentTime);
        reportStringStream << "Position added: " << positions.back().toString() << std::endl;
        reportStringStream << " at time: " << std::to_string(currentTime) << std::endl;
    }
}

Track::Track(std::string fileName, bool isFileName, metres granularity){
    std::string newPostion;
    std::string gpsData;
    std::string fileData;
    std::vector<std::string> elements = {"gpx", "trk"};
    this->granularity = granularity;

    if (isFileName){
        fileData = readFileData(fileName);
        reportStringStream << "Source file '" << fileName << "' opened okay." << std::endl;
    }

    gpsData = setupFileData(elements,fileData);

    if (XML::Parser::elementExists(gpsData, "name")) {
        routeName = XML::Parser::getElementContent(XML::Parser::getAndEraseElement(gpsData, "name"));
        reportStringStream << "Track name is: " << routeName << std::endl;
    }

    while (XML::Parser::elementExists(gpsData, "trkpt")) {
        newPostion = checkErrors(gpsData, "trkpt");
        addPostion(newPostion);

    }
    reportStringStream << positions.size() << " positions added." << std::endl;
    setRouteLength();
    report = reportStringStream.str();
}







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




std::string setupFileData(std::vector<std::string> elements,std::string fileData){
    std::string trkseg;
    for (std::string element : elements){
        if (! XML::Parser::elementExists(fileData,element)) throw std::domain_error("No '" + element + "' element.");
        fileData = XML::Parser::getElementContent(XML::Parser::getElement(fileData, element));
    }

    while (XML::Parser::elementExists(fileData, "trkseg")) {
        trkseg = XML::Parser::getElementContent(XML::Parser::getAndEraseElement(fileData, "trkseg"));
        XML::Parser::getAndEraseElement(trkseg, "name");
        fileData += trkseg;
    }

    return fileData;
}


Track::Track(std::string source, bool isFileName, metres granularity)
{


    std::string newPostion;
    std::string fileData;
    metres deltaH,deltaV;
    seconds startTime = 0;
    seconds currentTime = 0;
    seconds timeElapsed = 0;
    std::ostringstream reportStringStream;
    unsigned int num = 1;
    this->granularity = granularity;
    std::vector<std::string> elements = {"gpx", "trk"};

    // Get file data
    
    if (isFileName){
        fileData = readFileData(source, reportStringStream);
    }

    fileData = setupFileData(elements,fileData);

    if (XML::Parser::elementExists(fileData, "name")) {
        routeName = XML::Parser::getElementContent(XML::Parser::getAndEraseElement(fileData, "name"));
        reportStringStream << "Track name is: " << routeName << std::endl;
    }

    while (XML::Parser::elementExists(fileData, "trkpt")) {
        newPostion = checkErrors(fileData, "trkpt");

        positions.push_back(getNewPostion(newPostion));

        if (! XML::Parser::elementExists(newPostion,"time"))
            throw std::domain_error("No 'time' element.");

        currentTime = stringToTime(XML::Parser::getElementContent(XML::Parser::getElement(newPostion,"time")));

        if (positions.size()>1 && areSameLocation(positions.back(), positions.at(positions.size()-2))) {
            // If we're still at the same location, then we haven't departed yet.
            departed.back() = currentTime - startTime;
            reportStringStream << "Position ignored: " << positions.back().toString() << std::endl;
            positions.pop_back();
        } else {
            if (XML::Parser::elementExists(newPostion,"name")) {
               positionNames.push_back(XML::Parser::getElementContent(XML::Parser::getElement(newPostion,"name")));
            }
            timeElapsed = currentTime - startTime;
            arrived.push_back(timeElapsed);
            departed.push_back(timeElapsed);
            reportStringStream << "Position added: " << positions.back().toString() << std::endl;
            reportStringStream << " at time: " << std::to_string(timeElapsed) << std::endl;
            ++num;
        }
    }
    reportStringStream << num << " positions added." << std::endl;
    routeLength = 0;
    for (unsigned int i = 1; i < num; ++i ) {
        deltaH = Position::distanceBetween(positions[i-1], positions[i]);
        deltaV = positions[i-1].elevation() - positions[i].elevation();
        routeLength += sqrt(pow(deltaH,2) + pow(deltaV,2));
    }
    report = reportStringStream.str();
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

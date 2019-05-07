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

Track::Track(std::string source, bool isFileName, metres granularity)
{

    std::string mergedTrkSegs,trkseg,lat,lon,ele,name,time,temp,temp2;
    metres deltaH,deltaV;
    seconds startTime, currentTime, timeElapsed;
    std::ostringstream oss, reportStringStream;
    unsigned int num = 0;
    this->granularity = granularity;
    std::vector<std::string> elemetents = {"gpx", "trk"};

    // Get file data
    std::string fileData;
    if (isFileName){
        fileData = readFileData(source, reportStringStream);
    }

    for (std::string element : elemetents)
    {
        if (! XML::Parser::elementExists(fileData,element)) throw std::domain_error("No '" + element + "' element.");
        temp = XML::Parser::getElement(fileData, element);
        fileData = XML::Parser::getElementContent(temp);
    }

    if (XML::Parser::elementExists(fileData, "name")) {
        routeName = XML::Parser::getElementContent(XML::Parser::getAndEraseElement(fileData, "name"));
        oss << "Track name is: " << routeName << std::endl;
    }


    while (XML::Parser::elementExists(fileData, "trkseg")) {
        trkseg = XML::Parser::getElementContent(XML::Parser::getAndEraseElement(fileData, "trkseg"));
        XML::Parser::getAndEraseElement(trkseg, "name");
        mergedTrkSegs += trkseg;
    }


    if (! mergedTrkSegs.empty())
    {
        fileData = mergedTrkSegs;
    }

    temp = checkErrors(fileData, "trkpt");

    lat = XML::Parser::getElementAttribute(temp, "lat");
    lon = XML::Parser::getElementAttribute(temp, "lon");
    temp = XML::Parser::getElementContent(temp);
    if (XML::Parser::elementExists(temp, "ele")) {
        ele = XML::Parser::getElementContent(XML::Parser::getElement(temp, "ele"));

        positions.push_back(Position(lat,lon,ele));
        oss << "Start position added: " << positions.back().toString() << std::endl;
        ++num;
    } else {
        positions.push_back(Position(lat,lon));
        oss << "Start position added: " << positions.back().toString() << std::endl;
        ++num;
    }
    if (XML::Parser::elementExists(temp,"name")) {
        name = XML::Parser::getElementContent(XML::Parser::getElement(temp,"name"));
    }
    positionNames.push_back(name);
    arrived.push_back(0);
    departed.push_back(0);

    if (! XML::Parser::elementExists(temp,"time")){
        throw std::domain_error("No 'time' element.");
    }

    startTime = stringToTime(XML::Parser::getElementContent(XML::Parser::getElement(temp,"time")));


    while (XML::Parser::elementExists(fileData, "trkpt")) {
        temp = XML::Parser::getAndEraseElement(fileData, "trkpt");
        if (! XML::Parser::attributeExists(temp,"lat")) throw std::domain_error("No 'lat' attribute.");
        if (! XML::Parser::attributeExists(temp,"lon")) throw std::domain_error("No 'lon' attribute.");
        lat = XML::Parser::getElementAttribute(temp, "lat");
        lon = XML::Parser::getElementAttribute(temp, "lon");
        temp = XML::Parser::getElementContent(temp);
        if (XML::Parser::elementExists(temp, "ele")) {
            temp2 = XML::Parser::getElement(temp, "ele");
            ele = XML::Parser::getElementContent(temp2);
            positions.push_back(Position(lat,lon,ele));
        } else positions.push_back(Position(lat,lon));;
        if (! XML::Parser::elementExists(temp,"time")) throw std::domain_error("No 'time' element.");

        currentTime = stringToTime(XML::Parser::getElementContent(XML::Parser::getElement(temp,"time")));

        if (positions.size()>1 && areSameLocation(positions.back(), positions.at(positions.size()-2))) {
            // If we're still at the same location, then we haven't departed yet.
            departed.back() = currentTime - startTime;
            oss << "Position ignored: " << positions.back().toString() << std::endl;
            positions.pop_back();
        } else {
            if (XML::Parser::elementExists(temp,"name")) {
                temp2 = XML::Parser::getElement(temp,"name");
                name = XML::Parser::getElementContent(temp2);
            } else name = ""; // Fixed bug by adding this.
            positionNames.push_back(name);
            timeElapsed = currentTime - startTime;
            arrived.push_back(timeElapsed);
            departed.push_back(timeElapsed);
            oss << "Position added: " << positions.back().toString() << std::endl;
            oss << " at time: " << std::to_string(timeElapsed) << std::endl;
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

void Track::setGranularity(metres granularity)
{
    bool implemented = false;
    assert(implemented);
}

seconds Track::stringToTime(const std::string & timeStr)
{
    return stoull(timeStr);
}

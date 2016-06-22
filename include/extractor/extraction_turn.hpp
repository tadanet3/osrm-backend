#ifndef OSRM_EXTRACTION_TURN_HPP
#define OSRM_EXTRACTION_TURN_HPP

#include <boost/numeric/conversion/cast.hpp>

#include <cstdint>

namespace osrm
{
namespace extractor
{

struct ExtractionTurn
{
    ExtractionTurn(const double angle_, const bool is_uturn_)
        : angle(angle_), is_uturn(is_uturn_), duration(0.), weight(0.)
    {
    }

    const double angle;
    const bool is_uturn;
    double duration;
    double weight;
};
}
}

#endif

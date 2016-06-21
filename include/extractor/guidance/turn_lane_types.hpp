#ifndef OSRM_GUIDANCE_TURN_LANE_TYPES_HPP_
#define OSRM_GUIDANCE_TURN_LANE_TYPES_HPP_

#include <bitset>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <boost/assert.hpp>

#include "util/typedefs.hpp"

namespace osrm
{
namespace extractor
{
namespace guidance
{

namespace TurnLaneType
{
namespace detail
{
const constexpr std::size_t num_supported_lane_types = 11;
} // namespace detail

typedef std::uint16_t Mask;
const constexpr Mask empty = 0;
const constexpr Mask none = 1 << 0;
const constexpr Mask through = 1 << 1;
const constexpr Mask sharp_left = 1 << 2;
const constexpr Mask left = 1 << 3;
const constexpr Mask slight_left = 1 << 4;
const constexpr Mask slight_right = 1 << 5;
const constexpr Mask right = 1 << 6;
const constexpr Mask sharp_right = 1 << 7;
const constexpr Mask reverse = 1 << 8;
const constexpr Mask merge_to_left = 1 << 9;
const constexpr Mask merge_to_right = 1 << 10;

inline std::string toString(Mask lane_type)
{
    if (lane_type == 0)
        return "none";

    std::bitset<4 * sizeof(Mask)> mask(lane_type);
    const constexpr char *translations[detail::num_supported_lane_types] = {"none",
                                                                            "through",
                                                                            "sharp left",
                                                                            "left",
                                                                            "slight left",
                                                                            "slight right",
                                                                            "right",
                                                                            "sharp right",
                                                                            "reverse",
                                                                            "merge to left",
                                                                            "merge to right"};

    std::string result = "";
    for (std::size_t lane_id_nr = 0; lane_id_nr < detail::num_supported_lane_types; ++lane_id_nr)
        if (mask[lane_id_nr])
            result += result.empty() ? translations[lane_id_nr] : (std::string(";") + translations[lane_id_nr]);

    return result;
}
} // TurnLaneType

typedef std::vector<TurnLaneType::Mask> TurnLaneDescription;

} // guidance
} // extractor
} // osrm

#endif /* OSRM_GUIDANCE_TURN_LANE_TYPES_HPP_ */

#ifndef OSRM_EXTRACTOR_COORDINATE_EXTRACTOR_HPP_
#define OSRM_EXTRACTOR_COORDINATE_EXTRACTOR_HPP_

#include <vector>

#include "extractor/query_node.hpp"
#include "util/coordinate.hpp"
#include "util/coordinate_calculation.hpp"

#include "extractor/compressed_edge_container.hpp"
#include "util/node_based_graph.hpp"

namespace osrm
{
namespace extractor
{
namespace guidance
{

class CoordinateExtractor
{
  public:
    CoordinateExtractor(const util::NodeBasedDynamicGraph &node_based_graph,
                        const extractor::CompressedEdgeContainer &compressed_geometries,
                        const std::vector<extractor::QueryNode> &node_coordinates);
    ~CoordinateExtractor();

    /* Find a interpolated coordinate a long the compressed geometries. The desired coordinate
     * should be in a certain distance. This method is dedicated to find representative coordinates
     * at turns.
     */
    util::Coordinate GetCoordinateAlongRoad(const NodeID intersection_node,
                                            const EdgeID turn_edge,
                                            const bool traversed_in_reverse,
                                            const NodeID to_node,
                                            const std::uint8_t number_of_in_lanes) const;

    // instead of finding only a single coordinate, we can also list all coordinates along a road.
    std::vector<util::Coordinate> GetCoordinatesAlongRoad(const NodeID intersection_node,
                                                          const EdgeID turn_edge,
                                                          const bool traversed_in_reverse,
                                                          const NodeID to_node) const;

    /* when looking at a set of coordinates, this function allows trimming the vector to a smaller,
     * only containing coordinates up to a given distance along the path. The last coordinate might
     * be interpolated
     */
    std::vector<util::Coordinate> TrimCoordinatesToLength(std::vector<util::Coordinate> coordinates,
                                                          const double desired_length) const;

    /* when looking at a set of coordinates, this function allows trimming the vector to a smaller,
     * only containing coordinates up to a given distance along the path. The last coordinate might
     * be interpolated
     */
    std::vector<util::Coordinate>
    TrimCoordinatesByLengthFront(std::vector<util::Coordinate> coordinates,
                                 const double desired_length) const;

    /*
     * to correct for the initial offset, we move the lookahead coordinate close
     * to the original road. We do so by subtracting the difference between the
     * turn coordinate and the offset coordinate from the lookahead coordinge:
     *
     * a ------ b ------ c
     *          |
     *          d
     *             \
     *                \
     *                   e
     *
     * is converted to:
     *
     * a ------ b ------ c
     *             \
     *                \
     *                   e
     *
     * for fixpoint `b`, vector_base `d` and vector_head `e`
     */
    util::Coordinate GetCorrectedCoordinate(const util::Coordinate &fixpoint,
                                            const util::Coordinate &vector_base,
                                            const util::Coordinate &vector_head) const;

    /* generate a uniform vector of coordinates in same range distances
     *
     * Turns:
     * x ------------ x -- x - x
     *
     * Into:
     * x -- x -- x -- x -- x - x
     */
    std::vector<util::Coordinate>
    SampleCoordinates(const std::vector<util::Coordinate> &coordinates,
                      const double length,
                      const double rate) const;

  private:
    const util::NodeBasedDynamicGraph &node_based_graph;
    const extractor::CompressedEdgeContainer &compressed_geometries;
    const std::vector<extractor::QueryNode> &node_coordinates;

    double ComputeInterpolationFactor(const double desired_distance,
                                      const double distance_to_first,
                                      const double distance_to_second) const;

    std::pair<util::Coordinate, util::Coordinate>
    RegressionLine(const std::vector<util::Coordinate> &coordinates) const;

    std::size_t *times_called;
    std::size_t *times_failed;
};

} // namespace guidance
} // namespace extractor
} // namespace osrm

#endif // OSRM_EXTRACTOR_COORDINATE_EXTRACTOR_HPP_

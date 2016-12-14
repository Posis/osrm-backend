#include "extractor/guidance/mergable_road_detector.hpp"
#include "extractor/guidance/node_based_graph_walker.hpp"

#include "util/bearing.hpp"

using osrm::util::angularDeviation;

namespace osrm
{
namespace extractor
{
namespace guidance
{

namespace
{
const auto ASSUMED_LANE_WIDTH = 3.25;

// check a connected road for equality of a name
inline auto makeCheckRoadForName(const NameID name_id,
                                 const util::NodeBasedDynamicGraph &node_based_graph)
{
    return [name_id, &node_based_graph](const MergableRoadDetector::MergableRoadData &road) {
        // since we filter here, we don't want any other name than the one we are looking for
        return name_id != node_based_graph.GetEdgeData(road.eid).name_id;
    };
}
}

MergableRoadDetector::MergableRoadDetector(const util::NodeBasedDynamicGraph &node_based_graph,
                                           const std::vector<QueryNode> &node_coordinates,
                                           const IntersectionGenerator &intersection_generator,
                                           const CoordinateExtractor &coordinate_extractor)
    : node_based_graph(node_based_graph), node_coordinates(node_coordinates),
      intersection_generator(intersection_generator), coordinate_extractor(coordinate_extractor)
{
}

bool MergableRoadDetector::CanMergeRoad(const NodeID intersection_node,
                                        const IntersectionShapeData &lhs,
                                        const IntersectionShapeData &rhs) const
{
    // roads should be somewhat close
    if (angularDeviation(lhs.bearing, rhs.bearing) > 95)
        return false;

    const auto &lhs_edge_data = node_based_graph.GetEdgeData(lhs.eid);
    const auto &rhs_edge_data = node_based_graph.GetEdgeData(rhs.eid);

    // roundabouts are special, simply don't hurt them. We might not want to bear the
    // consequences
    if (lhs_edge_data.roundabout || rhs_edge_data.roundabout)
        return false;

    // and they need to describe the same road
    if (!RoadDataIsCompatible(lhs_edge_data, rhs_edge_data))
        return false;

    /* don't use any circular links, since they mess up detection we jump out early.
     *
     *          / -- \
     * a ---- b - - /
     */
    const auto road_target = [this](const MergableRoadData &road) {
        return node_based_graph.GetTarget(road.eid);
    };

    // TODO might have to skip over trivial intersections
    if (road_target(lhs) == intersection_node || road_target(lhs) == intersection_node)
        return false;

    // Don't merge turning circles/traffic loops
    if (IsTrafficLoop(intersection_node, lhs) || IsTrafficLoop(intersection_node, rhs))
        return false;

    // needs to be checked prior to link roads, since connections can seem like links
    if (ConnectAgain(intersection_node, lhs, rhs))
        return true;

    // Don't merge link roads
    if (IsLinkRoad(intersection_node, lhs) || IsLinkRoad(intersection_node, rhs))
        return false;

    // check if we simply split up prior to an intersection
    if (IsNarrowTriangle(intersection_node, lhs, rhs))
        return true;

    // finally check if two roads describe the direction
    return HaveSameDirection(intersection_node, lhs, rhs);
}

bool MergableRoadDetector::RoadDataIsCompatible(const util::NodeBasedEdgeData &lhs_edge_data,
                                                const util::NodeBasedEdgeData &rhs_edge_data) const
{
    /* to describe the same road, but in opposite directions (which is what we require for a
     * merge), the roads have to feature one reversed and one non-reversed edge
     */
    if (lhs_edge_data.reversed == rhs_edge_data.reversed)
        return false;

    /* The travel mode should be the same for both roads. If we were to merge different travel
     * modes, we would hide information/run the risk of loosing valid choices (e.g. short period
     * of pushing)
     */
    if (lhs_edge_data.travel_mode != rhs_edge_data.travel_mode)
        return false;

    // since merging is quite severe, we ask for identical names, not just similar names
    if (lhs_edge_data.name_id != rhs_edge_data.name_id)
        return false;

    return lhs_edge_data.road_classification == rhs_edge_data.road_classification;
}

bool MergableRoadDetector::IsTrafficLoop(const NodeID intersection_node,
                                         const MergableRoadData &road) const
{
    const auto connection = intersection_generator.SkipDegreeTwoNodes(intersection_node, road.eid);
    return intersection_node == node_based_graph.GetTarget(connection.via_eid);
}

bool MergableRoadDetector::IsNarrowTriangle(const NodeID intersection_node,
                                            const MergableRoadData &lhs,
                                            const MergableRoadData &rhs) const
{
    // selection data to the right and left
    IntersectionFinderAccumulator left_accumulator(5, intersection_generator),
        right_accumulator(5, intersection_generator);

    /* Standard following the straightmost road
     * Since both items have the same id, we can `select` based on any setup
     */
    SelectStraightmostRoadByNameAndOnlyChoice selector(
        node_based_graph.GetEdgeData(lhs.eid).name_id, lhs.bearing, false);

    NodeBasedGraphWalker graph_walker(node_based_graph, intersection_generator);
    graph_walker.TraverseRoad(intersection_node, lhs.eid, left_accumulator, selector);
    /* if the intersection does not have a right turn, we continue onto the next one once
     * (skipping over a single small side street)
     */
    if (angularDeviation(left_accumulator.intersection.findClosestTurn(90)->angle, 90) >
        NARROW_TURN_ANGLE)
    {
        graph_walker.TraverseRoad(node_based_graph.GetTarget(left_accumulator.via_edge_id),
                                  left_accumulator.intersection.findClosestTurn(180)->eid,
                                  left_accumulator,
                                  selector);
    }
    const auto distance_to_triangle = util::coordinate_calculation::haversineDistance(
        node_coordinates[intersection_node],
        node_coordinates[node_based_graph.GetTarget(left_accumulator.via_edge_id)]);

    // don't move to far down the road
    if (distance_to_triangle > 80)
        return false;

    graph_walker.TraverseRoad(intersection_node, rhs.eid, right_accumulator, selector);
    if (angularDeviation(right_accumulator.intersection.findClosestTurn(270)->angle, 270) >
        NARROW_TURN_ANGLE)
    {
        graph_walker.TraverseRoad(node_based_graph.GetTarget(right_accumulator.via_edge_id),
                                  right_accumulator.intersection.findClosestTurn(180)->eid,
                                  right_accumulator,
                                  selector);
    }

    BOOST_ASSERT(!left_accumulator.intersection.empty() && !right_accumulator.intersection.empty());

    // find the closes resembling a right turn
    const auto connector_turn = left_accumulator.intersection.findClosestTurn(90);
    /* check if that right turn connects to the right_accumulator intersection (i.e. we have a
     * triangle)
     * a connection should be somewhat to the right, when looking at the left side of the
     * triangle
     *
     *    b ..... c
     *     \     /
     *      \   /
     *       \ /
     *        a
     *
     * e.g. here when looking at `a,b`, a narrow triangle should offer a turn to the right, when
     * we want to connect to c
     */
    if (angularDeviation(connector_turn->angle, 90) > NARROW_TURN_ANGLE)
        return false;

    const auto num_lanes = [this](const MergableRoadData &road) {
        return std::max<std::uint8_t>(
            node_based_graph.GetEdgeData(road.eid).road_classification.GetNumberOfLanes(), 1);
    };

    // the width we can bridge at the intersection
    const auto assumed_road_width = (num_lanes(lhs) + num_lanes(rhs)) * ASSUMED_LANE_WIDTH;
    const auto distance_between_triangle_corners = util::coordinate_calculation::haversineDistance(
        node_coordinates[node_based_graph.GetTarget(left_accumulator.via_edge_id)],
        node_coordinates[node_based_graph.GetTarget(right_accumulator.via_edge_id)]);
    if (distance_between_triangle_corners > (assumed_road_width + 10))
        return false;

    // check if both intersections are connected
    IntersectionFinderAccumulator connect_accumulator(5, intersection_generator);
    graph_walker.TraverseRoad(node_based_graph.GetTarget(left_accumulator.via_edge_id),
                              connector_turn->eid,
                              connect_accumulator,
                              selector);
    // the if both items are connected
    return node_based_graph.GetTarget(connect_accumulator.via_edge_id) ==
           node_based_graph.GetTarget(right_accumulator.via_edge_id);
}

bool MergableRoadDetector::HaveSameDirection(const NodeID intersection_node,
                                             const MergableRoadData &lhs,
                                             const MergableRoadData &rhs) const
{
    if (angularDeviation(lhs.bearing, rhs.bearing) > 95)
        return false;

    // Find a coordinate following a road that is far away
    NodeBasedGraphWalker graph_walker(node_based_graph, intersection_generator);
    const auto getCoordinatesAlongWay = [&](const EdgeID edge_id, const double max_length) {
        LengthLimitedCoordinateAccumulator accumulator(
            coordinate_extractor, node_based_graph, max_length);
        SelectStraightmostRoadByNameAndOnlyChoice selector(
            node_based_graph.GetEdgeData(edge_id).name_id, lhs.bearing, false);
        graph_walker.TraverseRoad(intersection_node, edge_id, accumulator, selector);

        return std::make_pair(accumulator.accumulated_length, accumulator.coordinates);
    };

    std::vector<util::Coordinate> coordinates_to_the_left, coordinates_to_the_right;
    double distance_traversed_to_the_left, distance_traversed_to_the_right;

    const double constexpr distance_to_extract = 100;

    std::tie(distance_traversed_to_the_left, coordinates_to_the_left) =
        getCoordinatesAlongWay(lhs.eid, distance_to_extract);

    // quit early if the road is not very long
    if (distance_traversed_to_the_left <= 40)
        return false;

    std::tie(distance_traversed_to_the_right, coordinates_to_the_right) =
        getCoordinatesAlongWay(rhs.eid, distance_to_extract);

    if (distance_traversed_to_the_right <= 40)
        return false;

    coordinates_to_the_left = coordinate_extractor.SampleCoordinates(
        std::move(coordinates_to_the_left), distance_to_extract, 5);

    coordinates_to_the_right = coordinate_extractor.SampleCoordinates(
        std::move(coordinates_to_the_right), distance_to_extract, 5);

    /* extract the number of lanes for a road
     * restricts a vector to the last two thirds of the length
     */
    const auto prune = [](auto &data_vector) {
        BOOST_ASSERT(data_vector.size() >= 3);
        data_vector.erase(data_vector.begin(), data_vector.begin() + data_vector.size() / 3);
    };

    prune(coordinates_to_the_left);
    prune(coordinates_to_the_right);

    const auto are_parallel = util::coordinate_calculation::areParallel(coordinates_to_the_left,
                                                                        coordinates_to_the_right);

    if (!are_parallel)
        return false;

    // compare reference distance:
    const auto distance_between_roads = util::coordinate_calculation::findClosestDistance(
        coordinates_to_the_left[coordinates_to_the_left.size() / 2], coordinates_to_the_right);

    const auto combined_road_width =
        0.5 *
        (std::max<int>(
             1, node_based_graph.GetEdgeData(lhs.eid).road_classification.GetNumberOfLanes()) +
         std::max<int>(
             1, node_based_graph.GetEdgeData(rhs.eid).road_classification.GetNumberOfLanes())) *
        ASSUMED_LANE_WIDTH;
    return distance_between_roads <= combined_road_width + 8;
}

bool MergableRoadDetector::ConnectAgain(const NodeID intersection_node,
                                        const MergableRoadData &lhs,
                                        const MergableRoadData &rhs) const
{
    /* compute the set of all intersection_nodes along the way of an edge, until it reaches a
     * location with the same name repeatet at least three times
     */
    const auto left_connection =
        intersection_generator.SkipDegreeTwoNodes(intersection_node, lhs.eid);
    const auto right_connection =
        intersection_generator.SkipDegreeTwoNodes(intersection_node, rhs.eid);

    const auto left_candidate = node_based_graph.GetTarget(left_connection.via_eid);
    const auto right_candidate = node_based_graph.GetTarget(right_connection.via_eid);

    const auto candidate_is_valid =
        left_candidate == right_candidate && left_candidate != intersection_node;

    if (!candidate_is_valid)
        return false;

    // check if all entries at the destination or at the source are the same
    const auto all_same_name_and_degree_three = [this](const NodeID nid) {
        // check if the intersection found has degree three
        if (node_based_graph.GetOutDegree(nid) != 3)
            return false;

        // check if all items share a name
        const auto range = node_based_graph.GetAdjacentEdgeRange(nid);
        const auto required_name_id = node_based_graph.GetEdgeData(range.front()).name_id;

        const auto has_required_name = [this, required_name_id](const auto edge_id) {
            return node_based_graph.GetEdgeData(edge_id).name_id == required_name_id;
        };

        /* the beautiful way would be:
         * return range.end() == std::find_if_not(range.begin(), range.end(), has_required_name);
         * but that does not work due to range concepts
         */
        for( const auto eid : range )
            if( !has_required_name(eid) )
                return false;

        return true;
    };

    const auto degree_three_connect_in = all_same_name_and_degree_three(intersection_node);
    const auto degree_three_connect_out = all_same_name_and_degree_three(left_candidate);

    if (!degree_three_connect_in && !degree_three_connect_out)
        return false;

    if (degree_three_connect_in && degree_three_connect_out)
        return true;

    const auto distance_between_candidates = util::coordinate_calculation::haversineDistance(
        node_coordinates[intersection_node], node_coordinates[left_candidate]);

    return distance_between_candidates < 15;
}

bool MergableRoadDetector::IsLinkRoad(const NodeID intersection_node,
                                      const MergableRoadData &road) const
{
    const auto next_intersection_parameters =
        intersection_generator.SkipDegreeTwoNodes(intersection_node, road.eid);
    const auto next_intersection_along_road = intersection_generator.GetConnectedRoads(
        next_intersection_parameters.nid, next_intersection_parameters.via_eid);
    const auto extract_name = [this](const MergableRoadData &road) {
        return node_based_graph.GetEdgeData(road.eid).name_id;
    };

    const auto requested_name = extract_name(road);
    const auto next_road_along_path = next_intersection_along_road.findClosestTurn(
        STRAIGHT_ANGLE, makeCheckRoadForName(requested_name, node_based_graph));

    // we need to have a continuing road to successfully detect a link road
    if (next_road_along_path == next_intersection_along_road.end())
        return false;

    const auto opposite_of_next_road_along_path = next_intersection_along_road.findClosestTurn(
        util::restrictAngleToValidRange(next_road_along_path->angle + 180));

    // we cannot be looking at the same road we came from
    if (node_based_graph.GetTarget(opposite_of_next_road_along_path->eid) ==
        next_intersection_parameters.nid)
        return false;

    /* check if the opposite of the next road decision was sane. It could have been just as well our
     * incoming road.
     */
    if (angularDeviation(angularDeviation(next_road_along_path->angle, 180),
                         angularDeviation(opposite_of_next_road_along_path->angle, 0)) <
        FUZZY_ANGLE_DIFFERENCE)
        return false;

    // near straight road that continues
    return angularDeviation(opposite_of_next_road_along_path->angle, next_road_along_path->angle) >=
               160 &&
           RoadDataIsCompatible(
               node_based_graph.GetEdgeData(next_road_along_path->eid),
               node_based_graph.GetEdgeData(opposite_of_next_road_along_path->eid));
}

} // namespace guidance
} // namespace extractor
} // namespace osrm

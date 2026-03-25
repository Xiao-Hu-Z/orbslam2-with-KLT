/**
 * lddata_types.h
 *
 * Minimal self-contained type definitions for LDData and all constituent
 * structs, matching the vexus perception ldmap types used in production.
 *
 * All types live in the `ldmap` namespace to avoid collisions with the
 * POSIX id_t typedef from <sys/types.h>.
 *
 * Point3d_ uses float (Point3d = Point3d_<float>) with an additional weight
 * field.  id_t is uint64_t.  The vexus::Message base is an empty stub so this
 * header requires no external vexus SDK.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace vexus {

// Minimal stub so LDData : public vexus::Message compiles standalone.
struct Message {
    virtual ~Message() = default;
};

} // namespace vexus

namespace ldmap {

// ============================================================
//  Type aliases
// ============================================================

using id_t = uint64_t;

// ============================================================
//  Basic geometry
// ============================================================

template <typename T>
struct Point_ {
    T x = 0;
    T y = 0;
};

template <typename Dtype>
struct Point3d_ {
    Dtype x      = 0;
    Dtype y      = 0;
    Dtype z      = 0;
    Dtype weight = 0;

    Point3d_() = default;

    inline Point3d_(Dtype x_, Dtype y_, Dtype z_)
        : x(x_), y(y_), z(z_), weight(0) {}

    inline Point3d_(Point_<Dtype> point)
        : x(point.x), y(point.y), z(0), weight(0) {}

    inline Point3d_(Point_<Dtype> point, Dtype z_)
        : x(point.x), y(point.y), z(z_), weight(0) {}

    inline Point3d_(Dtype x_, Dtype y_, Dtype z_, Dtype weight_)
        : x(x_), y(y_), z(z_), weight(weight_) {}

    inline Point3d_(Point_<Dtype> point, Dtype z_, Dtype weight_)
        : x(point.x), y(point.y), z(z_), weight(weight_) {}
};

using Point3d = Point3d_<float>;

// ============================================================
//  Quaternion
// ============================================================

struct Quaternion {
    double qw{1};
    double qx{0};
    double qy{0};
    double qz{0};

    Quaternion() = default;
    Quaternion(double _qw, double _qx, double _qy, double _qz)
        : qw(_qw), qx(_qx), qy(_qy), qz(_qz) {}

    Quaternion operator*(const Quaternion& quat) const {
        double w = qw * quat.qw - qx * quat.qx - qy * quat.qy - qz * quat.qz;
        double x = qw * quat.qx + qx * quat.qw + qy * quat.qz - qz * quat.qy;
        double y = qw * quat.qy - qx * quat.qz + qy * quat.qw + qz * quat.qx;
        double z = qw * quat.qz + qx * quat.qy - qy * quat.qx + qz * quat.qw;
        return Quaternion(w, x, y, z);
    }
};

// ============================================================
//  Base structs
// ============================================================

struct RoadStructureBaseData {
    id_t    id               = 0;
    uint8_t detection_status = 0;
    float   confidence       = 100.0f;
};

// ============================================================
//  ObjectData
// ============================================================

struct ObjectData {
    id_t       id               = 0;
    Point3d    centroid{0, 0, 0};
    uint8_t    detection_status = 0;
    float      confidence       = 100.0f;
    Quaternion quaternion;
    float      length           = 0;
    float      width            = 0;
    float      height           = 0;
    std::vector<id_t> lane_ids;
    uint8_t    confidence_level = 0;
    id_t       junction_id      = 0;
    Point3d    centroid_lla{0, 0, 0};
    Point3d    centroid_ego{0, 0, 0};
};

// ============================================================
//  Semantic-map structures
// ============================================================

struct AreaData : public RoadStructureBaseData {
    uint8_t              area_type = 0;
    std::vector<Point3d> boundary;
    std::vector<id_t>    lane_ids;
    std::vector<Point3d> boundary_lla;
    std::vector<Point3d> boundary_ego;
};

struct JunctionData {
    id_t                 id                     = 0;
    std::vector<Point3d> boundary;
    std::vector<id_t>    from_section_ids;
    std::vector<id_t>    to_section_ids;
    uint8_t              junction_type           = 0;
    uint8_t              force_using_map_version = 0;
    uint8_t              map_source              = 0;
    std::vector<Point3d> boundary_lla;
    std::vector<Point3d> boundary_ego;
};

struct CrosswalkData : public RoadStructureBaseData {
    std::vector<Point3d> boundary;
    std::vector<id_t>    lane_ids;
    std::vector<Point3d> boundary_lla;
    std::vector<Point3d> boundary_ego;
};

struct StopLineData {
    id_t                 id               = 0;
    std::vector<Point3d> points;
    std::vector<id_t>    lane_ids;
    uint8_t              detection_status = 0;
    float                confidence       = 100.0f;
    std::vector<Point3d> points_lla;
    std::vector<Point3d> points_ego;
};

struct LineSegmentData {
    id_t                 id               = 0;
    uint8_t              line_type        = 0;
    uint8_t              line_style       = 0;
    uint8_t              color            = 0;
    float                width            = 0;
    uint8_t              detection_status = 0;
    float                confidence       = 100.0f;
    std::vector<Point3d> points;
    std::vector<Point3d> points_lla;
    std::vector<Point3d> points_ego;
};

struct LineData {
    id_t                         id            = 0;
    id_t                         left_lane_id  = 0;
    id_t                         right_lane_id = 0;
    std::vector<LineSegmentData> line_segments;
    uint64_t                     obs_age       = 0;
    bool                         is_opposite{false};
};

struct PoleData : public RoadStructureBaseData {
    std::vector<id_t> lane_ids;
    Point3d           bottom_point;
    Point3d           top_point;
    uint8_t           type = 0;
};

struct SpeedBumpData : public RoadStructureBaseData {
    uint8_t              type = 0;
    std::vector<Point3d> points;
    std::vector<id_t>    lane_ids;
    std::vector<Point3d> points_lla;
    std::vector<Point3d> points_ego;
};

// ============================================================
//  Route-map structures
// ============================================================

struct LaneData {
    id_t                  id               = 0;
    std::vector<Point3d>  center_points;
    uint8_t               lane_type        = 0;
    uint32_t              turn_type        = 0x0;
    uint8_t               trans_type       = 0;
    uint8_t               lane_direction   = 0;
    id_t                  left_line_id     = 0;
    id_t                  right_line_id    = 0;
    id_t                  section_id       = 0;
    id_t                  intersection_id  = 0;
    id_t                  left_lane_id     = 0;
    id_t                  right_lane_id    = 0;
    float                 length           = 0.0f;
    std::vector<float>    lane_widths;
    std::vector<float>    headings;
    std::vector<float>    longitudinal_slopes;
    std::vector<float>    lateral_slopes;
    std::vector<float>    curvature_radius;
    float                 max_speed        = 0.0f;
    float                 min_speed        = 0.0f;
    std::vector<id_t>     successor_link_ids;
    std::vector<id_t>     predecessor_link_ids;
    std::vector<id_t>     stop_line_ids;
    std::vector<id_t>     cross_walk_ids;
    std::vector<id_t>     speed_bump_ids;
    std::vector<id_t>     area_ids;
    uint8_t               detection_status = 0;
    float                 visual_diff      = 0;
    bool                  is_opposite{false};
    uint16_t              experience_spdlmt    = 255;
    bool                  force_avoidance_line{false};
    std::vector<Point3d>  center_points_lla;
    std::vector<Point3d>  center_points_ego;
};

struct SectionData {
    id_t                 id                   = 0;
    std::vector<Point3d> center_points;
    uint8_t              section_class        = 0;
    uint8_t              section_type         = 0;
    uint8_t              section_direction    = 0;
    uint8_t              positive_slope       = 0U;
    uint32_t             toll_type            = 0U;
    bool                 is_routing_section   = false;
    id_t                 intersection_id      = 0;
    float                length               = 0.0f;
    std::vector<id_t>    lane_ids;
    std::vector<id_t>    left_boundary_ids;
    std::vector<id_t>    right_boundary_ids;
    std::vector<id_t>    successor_link_ids;
    std::vector<id_t>    predecessor_link_ids;
    std::vector<id_t>    traffic_light_ids;
    std::vector<id_t>    traffic_sign_ids;
    std::vector<id_t>    from_junction_ids;
    std::vector<id_t>    to_junction_ids;
    std::vector<id_t>    opposite_section_ids;
    bool                 is_strategic_section = false;
    bool                 is_sparsify          = false;
    uint8_t              experience_spd       = 255;
    bool                 force_using_map      = false;
    std::vector<Point3d> center_points_lla;
    std::vector<Point3d> center_points_ego;
};

struct LaneLinkData {
    id_t                 id              = 0;
    id_t                 from_lane_id    = 0;
    id_t                 to_lane_id      = 0;
    uint8_t              link_type       = 0;
    std::vector<Point3d> ref_points;
    id_t                 section_link_id = 0;
};

// ============================================================
//  Traffic objects
// ============================================================

struct TrafficSignData {
    ObjectData object_data;
    uint8_t    type = 0;
};

struct TrafficLightData {
    ObjectData object_data;
    double     distance    = 0;
    int16_t    countdown   = -255;
    uint8_t    status      = 0;
    double     countdown_d = -255;
};

struct TrafficConeData {
    ObjectData object_data;
    uint8_t    type           = 0;
    uint16_t   obs_age        = 0;
    uint16_t   attribute_age  = 0;
    int        loc_estimation = 0;
};

struct RoadMarkerData {
    ObjectData  object_data;
    uint8_t     type             = 0;
    uint8_t     orientation_type = 0;
    uint8_t     color            = 0;
    std::string semantic         = "";
    uint16_t    obs_age          = 0;
    uint16_t    attribute_age    = 0;
};

// ============================================================
//  Top-level LDData
// ============================================================

class LDData : public vexus::Message {
 public:
    LDData()           = default;
    ~LDData() override = default;

    std::vector<AreaData>         areas;
    std::vector<CrosswalkData>    cross_walks;
    std::vector<SectionData>      sections;
    std::vector<LaneData>         lanes;
    std::vector<LineData>         lines;
    std::vector<StopLineData>     stop_lines;
    std::vector<JunctionData>     junctions;
    std::vector<TrafficLightData> traffic_lights;
    std::vector<TrafficSignData>  traffic_signs;
    std::vector<RoadMarkerData>   road_markers;
    std::vector<PoleData>         poles;
    std::vector<SpeedBumpData>    speed_bumps;
    std::vector<TrafficConeData>  traffic_cones;
    std::vector<uint64_t>         mpp_ids;
    std::vector<LaneLinkData>     lane_line;
};

} // namespace ldmap

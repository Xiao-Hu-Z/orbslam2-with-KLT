/**
 * load_road_structure.cpp
 *
 * Reads a RoadStructure JSON file (as produced by the map_routing_topic
 * serialiser) into the in-memory RoadStructure / RouteMapData /
 * SemanticMapData data-structures.
 *
 * The JSON uses arrays for every collection, IDs are stored as decimal
 * strings (uint64 values that may overflow int64).  This parser converts
 * them back to the appropriate integer types.
 *
 * Build (standalone):
 *   g++ -std=c++17 -O2 -o load_road_structure load_road_structure.cpp
 *
 * Usage:
 *   ./load_road_structure <path-to-json-file>
 */

#include <cstdint>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "json.hpp"

using json = nlohmann::json;

// ============================================================
//  Minimal type aliases & enumerations matching ldmap_base_data.hpp
// ============================================================

namespace vexus {
namespace perception {

using id_t      = uint64_t;
using float32_t = float;
using float64_t = double;

// ---- enums (only values present in the JSON are listed) ----

enum class LineType    : int { Unknown = 0 };
enum class LineStyle   : int { Unknown = 0 };
enum class Color       : int { None    = 0 };
enum class LaneType    : int { Unknown = 0 };
enum class TransType   : int { Unknown = 0 };
enum class SectionClass: int { Unknown = 0 };
enum class SectionType : int { Unknown = 0 };
enum class LinkType    : int { Unknown = 0 };
enum class JunctionType: int { kDefault= 0 };
enum class DetectionStatus : int { Unknown = 0 };
enum class CoordinateType  : int { ENU    = 0 };
enum class MapVendor       : int { Default= 0 };
enum class MapUsage        : int { MapFree= 0 };
enum class GnssPoseType    : int { Default= 0 };
enum class TrafficLightShape  : int { Unknown = 0, None = 0 };
enum class TrafficLightStatus : int { Unknown = 0 };
enum class TrafficLightBoard  : int { Unknown = 0 };
enum class TrafficSignType    : int { Unknown = 0 };
enum class RoadMarkerType     : int { Unknown = 0 };
enum class RMDOrientationType : int { Unknown = 0 };
enum class AreaType      : int { None    = 0 };
enum class PoleType      : int { Unknown = 0 };
enum class SpeedBumpType : int { Unknown = 0 };
enum class ObstacleType  : int { Unknown = 0 };
enum class GroupType     : int { Unknown = 0 };
enum class DataSource    : int { Default = 0 };
enum class LocEstimation : int { NO_INIT = 0 };
enum class RoadSpdLimit  : int { Default = 0 };
enum class RoadSpdLimitCancel: int { Default = 0 };
enum class RoadSpdLimitState : int { Default = 0 };
enum class SpeedLimitType    : int { Default = 0 };
enum class ExitRampProbable  : int { Default = 0 };
enum class OnlineMapStatus   : int { Normal  = 0 };
enum class SdNavigationStatus: int { None    = 0 };

// ---- basic geometry ----

struct Point3D {
    double   x = 0, y = 0, z = 0;
    double   weight = 0;
    double   Norm2D() const {
        return std::sqrt(x * x + y * y);
    }
    Point3D operator-(const Point3D& o) const {
        return {x - o.x, y - o.y, z - o.z, 0.0};
    }
    friend std::ostream& operator<<(std::ostream& os, const Point3D& p) {
        return os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
    }
};

struct PointLLH { double lon = 0, lat = 0, height = 0; };

struct Quaternion { double qw = 1, qx = 0, qy = 0, qz = 0; };

// ---- per-object base ----

struct RoadStructureBaseData {
    id_t id = 0;
    DetectionStatus detection_status = DetectionStatus::Unknown;
    uint16_t  obs_age       = 0;
    uint16_t  attribute_age = 0;
    float32_t confidence    = 100.0f;
    virtual ~RoadStructureBaseData() = default;
};

struct ObjectData : public RoadStructureBaseData {
    Point3D   centroid;
    Quaternion quaternion;
    float32_t length = 0, width = 0, height = 0;
    std::vector<id_t> lane_ids;
    bool      has_3d = false;
    LocEstimation loc_estimation = LocEstimation::NO_INIT;
    uint8_t   confidence_level = 0;
    ~ObjectData() override = default;
};

// ---- semantic map sub-structures ----

struct LineSegmentData : public RoadStructureBaseData {
    LineType  line_type  = LineType::Unknown;
    LineStyle line_style = LineStyle::Unknown;
    Color     color      = Color::None;
    float32_t width      = 0.15f;
    std::vector<Point3D> points;
};

struct LineData {
    id_t     id = 0;
    uint64_t obs_age = 0;
    id_t     left_lane_id  = 0;
    id_t     right_lane_id = 0;
    id_t     predecessor_laneline_id = 0;
    id_t     successor_laneline_id   = 0;
    bool     is_opposite  = false;
    bool     is_frommap   = false;
    bool     is_otherline = false;
    bool     is_cut       = false;
    std::vector<LineSegmentData> line_segments;
};

struct LineSegObjData : public RoadStructureBaseData {
    std::vector<Point3D> points;
    std::vector<id_t>    lane_ids;
};

using StopLineData = LineSegObjData;

struct SpeedBumpData : public LineSegObjData {
    SpeedBumpType type = SpeedBumpType::Unknown;
};

struct TrafficSignal {
    TrafficLightShape  shape  = TrafficLightShape::None;
    TrafficLightStatus status = TrafficLightStatus::Unknown;
};

struct TrafficLightData : public ObjectData {
    double     countdown_d = -255;
    int16_t    countdown   = -255;
    std::set<id_t> timer_ids;
    TrafficLightShape  shape  = TrafficLightShape::None;
    TrafficLightStatus status = TrafficLightStatus::Unknown;
    TrafficLightBoard  board  = TrafficLightBoard::Unknown;
    std::vector<TrafficSignal> signals;
    double     distance   = 0;
    DataSource data_source = DataSource::Default;
    GroupType  shape_from    = GroupType::Unknown;
    GroupType  status_from   = GroupType::Unknown;
    GroupType  countdown_from= GroupType::Unknown;
    GroupType  timer_id_from = GroupType::Unknown;
    GroupType  board_from    = GroupType::Unknown;
};

struct TrafficSignData : public ObjectData {
    TrafficSignType type = TrafficSignType::Unknown;
};

struct RoadMarkerData : public ObjectData {
    RoadMarkerType     type             = RoadMarkerType::Unknown;
    RMDOrientationType orientation_type = RMDOrientationType::Unknown;
    Color              color            = Color::None;
    std::string        semantic;
};

struct TrafficConeData : public ObjectData {
    ObstacleType type = ObstacleType::Unknown;
};

struct AreaData : public RoadStructureBaseData {
    AreaType          area_type = AreaType::None;
    std::vector<Point3D> boundary;
    std::vector<id_t>    lane_ids;
};

struct CrosswalkData : public RoadStructureBaseData {
    std::vector<Point3D> boundary;
    std::vector<id_t>    lane_ids;
};

struct ManholeCoverData : public RoadStructureBaseData {
    std::vector<Point3D> boundary;
};

struct PoleData : public RoadStructureBaseData {
    std::vector<id_t> lane_ids;
    Point3D bottom_point, top_point;
    PoleType type = PoleType::Unknown;
};

struct VcsLineData {
    uint64_t id = 0;
    LineType  type  = LineType::Unknown;
    LineStyle style = LineStyle::Unknown;
    Color     color = Color::None;
    float32_t line_width = 0.15f;
    std::deque<Point3D> points;
    bool      is_opposite = false;
    float32_t confidence  = 100.0f;
};

// ---- route-map sub-structures ----

struct LaneData {
    id_t     id            = 0;
    id_t     section_id    = 0;
    uint8_t  sequence_id   = 0;
    id_t     intersection_id = 0;
    LaneType lane_type     = LaneType::Unknown;
    DetectionStatus detection_status = DetectionStatus::Unknown;
    float    visual_diff   = 0;
    uint32_t turn_type     = 0;
    TransType trans_type   = TransType::Unknown;
    id_t     left_line_id  = 0;
    id_t     right_line_id = 0;
    id_t     left_lane_id  = 0;
    id_t     right_lane_id = 0;
    float32_t length       = 0;
    std::vector<Point3D>    center_points;
    std::vector<float32_t>  lane_widths;
    std::vector<float32_t>  headings;
    std::vector<float32_t>  longitudinal_slopes;
    std::vector<float32_t>  lateral_slopes;
    std::vector<float32_t>  curvature_radius;
    float32_t max_speed = 0;
    float32_t min_speed = 0;
    std::vector<id_t> successor_link_ids;
    std::vector<id_t> predecessor_link_ids;
    std::vector<id_t> map_lane_successor_lane_ids;
    std::vector<id_t> map_lane_predecessor_lane_ids;
    std::vector<id_t> traffic_light_ids;
    std::vector<id_t> traffic_sign_ids;
    std::vector<id_t> road_marker_ids;
    std::vector<id_t> stop_line_ids;
    std::vector<id_t> cross_walk_ids;
    std::vector<id_t> speed_bump_ids;
    std::vector<id_t> area_ids;
    bool is_opposite   = false;
    bool is_new_offset = false;
    uint16_t experience_spdlmt = 255;
    bool force_avoidance_line  = false;
};

struct SectionData {
    id_t         id             = 0;
    SectionClass section_class  = SectionClass::Unknown;
    SectionType  section_type   = SectionType::Unknown;
    id_t         intersection_id = 0;
    float32_t    length         = 0;
    std::vector<Point3D> center_points;
    std::vector<id_t> lane_ids;
    std::vector<id_t> traffic_light_ids;
    std::vector<id_t> left_boundary_ids;
    std::vector<id_t> right_boundary_ids;
    std::vector<id_t> successor_link_ids;
    std::vector<id_t> predecessor_link_ids;
    std::vector<id_t> opposite_section_ids;
    std::vector<id_t> from_junction_ids;
    std::vector<id_t> to_junction_ids;
    bool is_routing_section    = false;
    bool is_strategic_section  = false;
    bool is_sparsify           = false;
    uint8_t experience_spd     = 255;
    bool force_using_map       = false;
    bool is_frommap            = false;
};

struct LaneLinkData {
    id_t     id           = 0;
    id_t     from_lane_id = 0;
    id_t     to_lane_id   = 0;
    LinkType link_type    = LinkType::Unknown;
    std::vector<Point3D> ref_points;
    id_t     section_link_id = 0;
};

struct SectionLinkData {
    id_t     id             = 0;
    id_t     from_section_id = 0;
    id_t     to_section_id  = 0;
    LinkType link_type      = LinkType::Unknown;
    bool     is_routing_link = true;
};

struct JunctionData {
    id_t id = 0;
    std::vector<Point3D> boundary;
    std::vector<id_t> from_section_ids;
    std::vector<id_t> to_section_ids;
    std::vector<id_t> from_lane_ids;
    std::vector<id_t> to_lane_ids;
    JunctionType junction_type = JunctionType::kDefault;
};

// ---- table typedefs ----

using LineDataTable         = std::unordered_map<id_t, LineData>;
using StopLineDataTable     = std::unordered_map<id_t, StopLineData>;
using AreaDataTable         = std::unordered_map<id_t, AreaData>;
using TrafficLightDataTable = std::unordered_map<id_t, TrafficLightData>;
using TrafficSignDataTable  = std::unordered_map<id_t, TrafficSignData>;
using TrafficConeDataTable  = std::unordered_map<id_t, TrafficConeData>;
using RoadMarkerDataTable   = std::unordered_map<id_t, RoadMarkerData>;
using CrosswalkDataTable    = std::unordered_map<id_t, CrosswalkData>;
using ManholeCoverDataTable = std::unordered_map<id_t, ManholeCoverData>;
using PoleDataTable         = std::unordered_map<id_t, PoleData>;
using SpeedBumpDataTable    = std::unordered_map<id_t, SpeedBumpData>;
using VcsLineDataTable      = std::unordered_map<id_t, VcsLineData>;
using LaneDataTable         = std::unordered_map<id_t, LaneData>;
using SectionDataTable      = std::unordered_map<id_t, SectionData>;
using LaneLinkDataTable     = std::unordered_map<id_t, LaneLinkData>;
using SectionLinkDataTable  = std::unordered_map<id_t, SectionLinkData>;
using JunctionDataTable     = std::unordered_map<id_t, JunctionData>;

// ---- top-level aggregates ----

struct SemanticMapData {
    LineDataTable         lines;
    StopLineDataTable     stop_lines;
    AreaDataTable         areas;
    TrafficLightDataTable traffic_lights;
    TrafficSignDataTable  traffic_signs;
    TrafficConeDataTable  traffic_cones;
    RoadMarkerDataTable   road_markers;
    CrosswalkDataTable    cross_walks;
    ManholeCoverDataTable manhole_covers;
    PoleDataTable         poles;
    SpeedBumpDataTable    speed_bumps;
    VcsLineDataTable      vcs_lines;
};

struct RouteMapData {
    LaneDataTable        lanes;
    SectionDataTable     sections;
    LaneLinkDataTable    lane_links;
    SectionLinkDataTable section_links;
    JunctionDataTable    junctions;
};

struct EgoPose {
    Point3D position;
    Quaternion quat;
    GnssPoseType gnssPosType = GnssPoseType::Default;
};

struct EgoTrajData {
    std::vector<EgoPose> trajs;
    std::vector<id_t>    ego_lanes;
};

struct Header {
    uint64_t      timestamp_ns       = 0;
    uint64_t      sync_timestamp_ns  = 0;
    uint64_t      recv_timestamp_ns  = 0;
    uint64_t      datacall_timestamp_ns = 0;
    uint64_t      frame_id           = 0;
    uint64_t      sensor_id          = 0;
    CoordinateType coord_type        = CoordinateType::ENU;
    MapVendor      vendor            = MapVendor::Default;
    MapUsage       map_usage         = MapUsage::MapFree;
};

struct RoadStructure {
    Header         header;
    SemanticMapData semantic_map_data;
    RouteMapData    route_map_data;
    EgoTrajData     ego_layer_data;
};

// ============================================================
//  Helper: parse a possibly-quoted integer string to uint64_t
// ============================================================

static id_t parseId(const json& j) {
    if (j.is_string()) {
        return std::stoull(j.get<std::string>());
    }
    return j.get<uint64_t>();
}

static id_t parseIdOpt(const json& j, const std::string& key, id_t def = 0) {
    if (!j.contains(key)) return def;
    return parseId(j.at(key));
}

// Parse a JSON array of id strings/ints into a vector<id_t>
static std::vector<id_t> parseIdArray(const json& j) {
    std::vector<id_t> result;
    if (!j.is_array()) return result;
    for (const auto& item : j) {
        result.push_back(parseId(item));
    }
    return result;
}

static Point3D parsePoint3D(const json& j) {
    Point3D p;
    p.x      = j.value("x",      0.0);
    p.y      = j.value("y",      0.0);
    p.z      = j.value("z",      0.0);
    p.weight = j.value("weight", 0.0);
    return p;
}

static std::vector<Point3D> parsePoint3DArray(const json& j) {
    std::vector<Point3D> pts;
    if (!j.is_array()) return pts;
    for (const auto& jp : j) {
        pts.push_back(parsePoint3D(jp));
    }
    return pts;
}

static Quaternion parseQuaternion(const json& j) {
    Quaternion q;
    q.qw = j.value("qw", 1.0);
    q.qx = j.value("qx", 0.0);
    q.qy = j.value("qy", 0.0);
    q.qz = j.value("qz", 0.0);
    return q;
}

// ============================================================
//  Semantic map parsers
// ============================================================

static LineData parseLineData(const json& j) {
    LineData l;
    l.id            = parseId(j.at("id"));
    l.left_lane_id  = parseIdOpt(j, "left_lane_id");
    l.right_lane_id = parseIdOpt(j, "right_lane_id");
    l.is_opposite   = j.value("is_opposite", false);
    l.is_frommap    = j.value("is_frommap",  false);
    l.is_otherline  = j.value("is_otherline",false);

    // obs_age may be a quoted integer
    if (j.contains("obs_age")) {
        l.obs_age = static_cast<uint64_t>(parseId(j.at("obs_age")));
    }

    for (const auto& jseg : j.at("line_segments")) {
        LineSegmentData seg;
        seg.id         = parseIdOpt(jseg, "id");
        seg.line_type  = static_cast<LineType>(jseg.value("line_type",  0));
        seg.line_style = static_cast<LineStyle>(jseg.value("line_style",0));
        seg.color      = static_cast<Color>(jseg.value("color",         0));
        seg.width      = jseg.value("width", 0.0f);
        seg.detection_status = static_cast<DetectionStatus>(
            jseg.value("detection_status", 0));
        seg.confidence = jseg.value("confidence", 100.0f);
        seg.points     = parsePoint3DArray(jseg.at("points"));
        l.line_segments.push_back(std::move(seg));
    }
    return l;
}

static StopLineData parseStopLineData(const json& j) {
    StopLineData s;
    s.id               = parseId(j.at("id"));
    s.detection_status = static_cast<DetectionStatus>(j.value("detection_status", 0));
    s.confidence       = j.value("confidence", 100.0f);
    if (j.contains("points"))   s.points   = parsePoint3DArray(j.at("points"));
    if (j.contains("lane_ids")) s.lane_ids = parseIdArray(j.at("lane_ids"));
    return s;
}

static void parseObjectDataBase(const json& j, ObjectData& o) {
    o.id               = parseId(j.at("id"));
    o.detection_status = static_cast<DetectionStatus>(j.value("detection_status", 0));
    o.confidence       = j.value("confidence", 100.0f);
    o.confidence_level = j.value("confidence_level", 0);
    o.length           = j.value("length", 0.0f);
    o.width            = j.value("width",  0.0f);
    o.height           = j.value("height", 0.0f);
    if (j.contains("centroid"))   o.centroid   = parsePoint3D(j.at("centroid"));
    if (j.contains("quaternion")) o.quaternion = parseQuaternion(j.at("quaternion"));
    if (j.contains("lane_ids"))   o.lane_ids   = parseIdArray(j.at("lane_ids"));
}

static TrafficLightData parseTrafficLightData(const json& j) {
    TrafficLightData tl;
    // In the JSON the object fields are nested under "object_data"
    if (j.contains("object_data")) {
        parseObjectDataBase(j.at("object_data"), tl);
    }
    tl.countdown   = static_cast<int16_t>(j.value("countdown",   -255));
    tl.countdown_d = j.value("countdown_d", -255.0);
    tl.distance    = j.value("distance", 0.0);
    tl.status      = static_cast<TrafficLightStatus>(j.value("state", 0));
    if (j.contains("traffic_signals")) {
        for (const auto& js : j.at("traffic_signals")) {
            TrafficSignal sig;
            sig.shape  = static_cast<TrafficLightShape>(js.value("shape",  0));
            sig.status = static_cast<TrafficLightStatus>(js.value("status", 0));
            tl.signals.push_back(sig);
        }
    }
    return tl;
}

static CrosswalkData parseCrosswalkData(const json& j) {
    CrosswalkData cw;
    cw.id               = parseId(j.at("id"));
    cw.detection_status = static_cast<DetectionStatus>(j.value("detection_status", 0));
    cw.confidence       = j.value("confidence", 100.0f);
    if (j.contains("boundary")) cw.boundary = parsePoint3DArray(j.at("boundary"));
    if (j.contains("lane_ids")) cw.lane_ids = parseIdArray(j.at("lane_ids"));
    return cw;
}

// Generic helpers for types with fewer fields in this dataset
template<typename T>
static T parseAreaData(const json& j) {
    T a;
    a.id               = parseId(j.at("id"));
    a.detection_status = static_cast<DetectionStatus>(j.value("detection_status", 0));
    a.confidence       = j.value("confidence", 100.0f);
    if (j.contains("boundary")) a.boundary = parsePoint3DArray(j.at("boundary"));
    if (j.contains("lane_ids")) a.lane_ids = parseIdArray(j.at("lane_ids"));
    return a;
}

// ============================================================
//  Route-map parsers
// ============================================================

static LaneData parseLaneData(const json& j) {
    LaneData lane;
    lane.id              = parseId(j.at("id"));
    lane.section_id      = parseIdOpt(j, "section_id");
    lane.intersection_id = parseIdOpt(j, "intersection_id");
    lane.lane_type       = static_cast<LaneType>(j.value("lane_type", 0));
    lane.turn_type       = j.value("turn_type", 0u);
    lane.trans_type      = static_cast<TransType>(j.value("trans_type", 0));
    lane.detection_status= static_cast<DetectionStatus>(j.value("detection_status", 0));
    lane.visual_diff     = j.value("visual_diff", 0.0f);
    lane.left_line_id    = parseIdOpt(j, "left_line_id");
    lane.right_line_id   = parseIdOpt(j, "right_line_id");
    lane.left_lane_id    = parseIdOpt(j, "left_lane_id");
    lane.right_lane_id   = parseIdOpt(j, "right_lane_id");
    lane.length          = j.value("length", 0.0f);
    lane.max_speed       = j.value("max_speed", 0.0f);
    lane.min_speed       = j.value("min_speed", 0.0f);
    lane.is_opposite     = j.value("is_opposite", false);
    lane.experience_spdlmt = j.value("experience_spdlmt", 255);
    lane.force_avoidance_line = j.value("force_avoidance_line", false);

    if (j.contains("center_points"))
        lane.center_points = parsePoint3DArray(j.at("center_points"));

    auto parseFloatArray = [&](const std::string& key) {
        std::vector<float32_t> v;
        if (j.contains(key) && j.at(key).is_array()) {
            for (const auto& x : j.at(key)) v.push_back(x.get<float>());
        }
        return v;
    };
    lane.lane_widths         = parseFloatArray("lane_widths");
    lane.headings            = parseFloatArray("headings");
    lane.longitudinal_slopes = parseFloatArray("longitudinal_slopes");
    lane.lateral_slopes      = parseFloatArray("lateral_slopes");
    lane.curvature_radius    = parseFloatArray("curvature_radius");

    if (j.contains("successor_link_ids"))   lane.successor_link_ids   = parseIdArray(j.at("successor_link_ids"));
    if (j.contains("predecessor_link_ids")) lane.predecessor_link_ids = parseIdArray(j.at("predecessor_link_ids"));
    if (j.contains("traffic_light_ids"))    lane.traffic_light_ids    = parseIdArray(j.at("traffic_light_ids"));
    if (j.contains("traffic_sign_ids"))     lane.traffic_sign_ids     = parseIdArray(j.at("traffic_sign_ids"));
    if (j.contains("road_marker_ids"))      lane.road_marker_ids      = parseIdArray(j.at("road_marker_ids"));
    if (j.contains("stop_line_ids"))        lane.stop_line_ids        = parseIdArray(j.at("stop_line_ids"));
    if (j.contains("cross_walk_ids"))       lane.cross_walk_ids       = parseIdArray(j.at("cross_walk_ids"));
    if (j.contains("speed_bump_ids"))       lane.speed_bump_ids       = parseIdArray(j.at("speed_bump_ids"));
    if (j.contains("area_ids"))             lane.area_ids             = parseIdArray(j.at("area_ids"));
    return lane;
}

static SectionData parseSectionData(const json& j) {
    SectionData s;
    s.id              = parseId(j.at("id"));
    s.section_class   = static_cast<SectionClass>(j.value("section_class",  0));
    s.section_type    = static_cast<SectionType>(j.value("section_type",   0));
    s.intersection_id = parseIdOpt(j, "intersection_id");
    s.length          = j.value("length", 0.0f);
    s.is_routing_section   = j.value("is_routing_section",   false);
    s.is_strategic_section = j.value("is_strategic_section", false);
    s.is_sparsify          = j.value("is_sparsify",          false);
    s.experience_spd       = j.value("experience_spd",       uint8_t{255});
    s.force_using_map      = j.value("force_using_map",      false);

    if (j.contains("center_points"))       s.center_points       = parsePoint3DArray(j.at("center_points"));
    if (j.contains("lane_ids"))            s.lane_ids            = parseIdArray(j.at("lane_ids"));
    if (j.contains("traffic_light_ids"))   s.traffic_light_ids   = parseIdArray(j.at("traffic_light_ids"));
    if (j.contains("left_boundary_ids"))   s.left_boundary_ids   = parseIdArray(j.at("left_boundary_ids"));
    if (j.contains("right_boundary_ids"))  s.right_boundary_ids  = parseIdArray(j.at("right_boundary_ids"));
    if (j.contains("successor_link_ids"))  s.successor_link_ids  = parseIdArray(j.at("successor_link_ids"));
    if (j.contains("predecessor_link_ids"))s.predecessor_link_ids= parseIdArray(j.at("predecessor_link_ids"));
    if (j.contains("opposite_section_ids"))s.opposite_section_ids= parseIdArray(j.at("opposite_section_ids"));
    if (j.contains("from_junction_ids"))   s.from_junction_ids   = parseIdArray(j.at("from_junction_ids"));
    if (j.contains("to_junction_ids"))     s.to_junction_ids     = parseIdArray(j.at("to_junction_ids"));
    return s;
}

static LaneLinkData parseLaneLinkData(const json& j) {
    LaneLinkData ll;
    ll.id             = parseId(j.at("id"));
    ll.from_lane_id   = parseIdOpt(j, "from_lane_id");
    ll.to_lane_id     = parseIdOpt(j, "to_lane_id");
    ll.link_type      = static_cast<LinkType>(j.value("link_type", 0));
    ll.section_link_id= parseIdOpt(j, "section_link_id");
    if (j.contains("ref_points")) ll.ref_points = parsePoint3DArray(j.at("ref_points"));
    return ll;
}

static SectionLinkData parseSectionLinkData(const json& j) {
    SectionLinkData sl;
    sl.id              = parseId(j.at("id"));
    sl.from_section_id = parseIdOpt(j, "from_section_id");
    sl.to_section_id   = parseIdOpt(j, "to_section_id");
    sl.link_type       = static_cast<LinkType>(j.value("link_type", 0));
    sl.is_routing_link = j.value("is_routing_link", false);
    return sl;
}

static JunctionData parseJunctionData(const json& j) {
    JunctionData jd;
    jd.id           = parseId(j.at("id"));
    jd.junction_type= static_cast<JunctionType>(j.value("junction_type", 0));
    if (j.contains("boundary"))         jd.boundary         = parsePoint3DArray(j.at("boundary"));
    if (j.contains("from_section_ids")) jd.from_section_ids = parseIdArray(j.at("from_section_ids"));
    if (j.contains("to_section_ids"))   jd.to_section_ids   = parseIdArray(j.at("to_section_ids"));
    if (j.contains("from_lane_ids"))    jd.from_lane_ids    = parseIdArray(j.at("from_lane_ids"));
    if (j.contains("to_lane_ids"))      jd.to_lane_ids      = parseIdArray(j.at("to_lane_ids"));
    return jd;
}

// ============================================================
//  Top-level loader
// ============================================================

/**
 * Load a RoadStructure from the JSON file produced by the map_routing_topic
 * serialiser.  Returns nullptr on error.
 *
 * JSON format notes:
 *  - Collections are JSON **arrays** (not objects keyed by ID).
 *  - Numeric IDs that could exceed INT64_MAX are stored as decimal strings.
 *  - The header fields are in "mapheader" (not "header").
 *  - TrafficLightData fields are nested under "object_data".
 */
std::shared_ptr<RoadStructure> LoadRoadStructure(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "[LoadRoadStructure] cannot open: " << path << "\n";
        return nullptr;
    }

    json j;
    try {
        ifs >> j;
    } catch (const std::exception& e) {
        std::cerr << "[LoadRoadStructure] JSON parse error: " << e.what() << "\n";
        return nullptr;
    }

    auto rs = std::make_shared<RoadStructure>();

    // ---- header ----
    if (j.contains("mapheader")) {
        const auto& jh = j["mapheader"];
        rs->header.timestamp_ns = parseIdOpt(jh, "timestamp_ns");
        rs->header.sensor_id    = parseIdOpt(jh, "sensor_id");
        rs->header.coord_type   = static_cast<CoordinateType>(jh.value("coord_type", 0));
        rs->header.vendor       = static_cast<MapVendor>(jh.value("map_vendor",  0));
        rs->header.map_usage    = static_cast<MapUsage>(jh.value("map_usage",    0));
    } else if (j.contains("header")) {
        const auto& jh = j["header"];
        rs->header.frame_id     = parseIdOpt(jh, "frame_id");
        rs->header.coord_type   = static_cast<CoordinateType>(jh.value("coord_type", 0));
    }

    // ---- semantic_map_data ----
    if (j.contains("semantic_map_data")) {
        const auto& smd = j["semantic_map_data"];

        for (const auto& jl  : smd.value("lines",        json::array())) {
            auto line = parseLineData(jl);
            rs->semantic_map_data.lines[line.id] = std::move(line);
        }
        for (const auto& jsl : smd.value("stop_lines",   json::array())) {
            auto sl = parseStopLineData(jsl);
            rs->semantic_map_data.stop_lines[sl.id] = std::move(sl);
        }
        for (const auto& jtl : smd.value("traffic_lights", json::array())) {
            auto tl = parseTrafficLightData(jtl);
            rs->semantic_map_data.traffic_lights[tl.id] = std::move(tl);
        }
        for (const auto& jcw : smd.value("cross_walks",  json::array())) {
            auto cw = parseCrosswalkData(jcw);
            rs->semantic_map_data.cross_walks[cw.id] = std::move(cw);
        }
        for (const auto& ja  : smd.value("areas",        json::array())) {
            AreaData ad;
            ad.id               = parseId(ja.at("id"));
            ad.detection_status = static_cast<DetectionStatus>(ja.value("detection_status", 0));
            ad.confidence       = ja.value("confidence", 100.0f);
            ad.area_type        = static_cast<AreaType>(ja.value("area_type", 0));
            if (ja.contains("boundary")) ad.boundary = parsePoint3DArray(ja.at("boundary"));
            if (ja.contains("lane_ids")) ad.lane_ids = parseIdArray(ja.at("lane_ids"));
            rs->semantic_map_data.areas[ad.id] = std::move(ad);
        }
        for (const auto& jp  : smd.value("poles",        json::array())) {
            PoleData pd;
            pd.id   = parseId(jp.at("id"));
            pd.type = static_cast<PoleType>(jp.value("type", 0));
            if (jp.contains("bottom_point")) pd.bottom_point = parsePoint3D(jp.at("bottom_point"));
            if (jp.contains("top_point"))    pd.top_point    = parsePoint3D(jp.at("top_point"));
            if (jp.contains("lane_ids"))     pd.lane_ids     = parseIdArray(jp.at("lane_ids"));
            rs->semantic_map_data.poles[pd.id] = std::move(pd);
        }
        for (const auto& jsb : smd.value("speed_bumps",  json::array())) {
            SpeedBumpData sb;
            sb.id     = parseId(jsb.at("id"));
            sb.type   = static_cast<SpeedBumpType>(jsb.value("type", 0));
            if (jsb.contains("points"))   sb.points   = parsePoint3DArray(jsb.at("points"));
            if (jsb.contains("lane_ids")) sb.lane_ids = parseIdArray(jsb.at("lane_ids"));
            rs->semantic_map_data.speed_bumps[sb.id] = std::move(sb);
        }
        for (const auto& jmh : smd.value("manhole_covers", json::array())) {
            ManholeCoverData mh;
            mh.id = parseId(jmh.at("id"));
            if (jmh.contains("boundary")) mh.boundary = parsePoint3DArray(jmh.at("boundary"));
            rs->semantic_map_data.manhole_covers[mh.id] = std::move(mh);
        }
        for (const auto& jts : smd.value("traffic_signs", json::array())) {
            TrafficSignData ts;
            if (jts.contains("object_data")) parseObjectDataBase(jts.at("object_data"), ts);
            ts.type = static_cast<TrafficSignType>(jts.value("type", 0));
            rs->semantic_map_data.traffic_signs[ts.id] = std::move(ts);
        }
        for (const auto& jrm : smd.value("road_markers", json::array())) {
            RoadMarkerData rm;
            if (jrm.contains("object_data")) parseObjectDataBase(jrm.at("object_data"), rm);
            rm.type     = static_cast<RoadMarkerType>(jrm.value("type",  0));
            rm.color    = static_cast<Color>(jrm.value("color",           0));
            rm.semantic = jrm.value("semantic", "");
            rs->semantic_map_data.road_markers[rm.id] = std::move(rm);
        }
        for (const auto& jtc : smd.value("traffic_cones", json::array())) {
            TrafficConeData tc;
            if (jtc.contains("object_data")) parseObjectDataBase(jtc.at("object_data"), tc);
            tc.type = static_cast<ObstacleType>(jtc.value("type", 0));
            rs->semantic_map_data.traffic_cones[tc.id] = std::move(tc);
        }
    }

    // ---- route_map_data ----
    if (j.contains("route_map_data")) {
        const auto& rmd = j["route_map_data"];

        for (const auto& jl  : rmd.value("lanes",         json::array())) {
            auto lane = parseLaneData(jl);
            rs->route_map_data.lanes[lane.id] = std::move(lane);
        }
        for (const auto& js  : rmd.value("sections",      json::array())) {
            auto sec = parseSectionData(js);
            rs->route_map_data.sections[sec.id] = std::move(sec);
        }
        for (const auto& jll : rmd.value("lane_links",    json::array())) {
            auto ll = parseLaneLinkData(jll);
            rs->route_map_data.lane_links[ll.id] = std::move(ll);
        }
        for (const auto& jsl : rmd.value("section_links", json::array())) {
            auto sl = parseSectionLinkData(jsl);
            rs->route_map_data.section_links[sl.id] = std::move(sl);
        }
        for (const auto& jj  : rmd.value("junctions",     json::array())) {
            auto jd = parseJunctionData(jj);
            rs->route_map_data.junctions[jd.id] = std::move(jd);
        }
    }

    // ---- ego_layer_data ----
    if (j.contains("ego_layer_data")) {
        const auto& eld = j["ego_layer_data"];
        if (eld.contains("ego_lanes"))
            rs->ego_layer_data.ego_lanes = parseIdArray(eld.at("ego_lanes"));
        if (eld.contains("trajs")) {
            for (const auto& jt : eld.at("trajs")) {
                EgoPose ep;
                if (jt.contains("position"))  ep.position = parsePoint3D(jt.at("position"));
                if (jt.contains("quat"))       ep.quat     = parseQuaternion(jt.at("quat"));
                rs->ego_layer_data.trajs.push_back(ep);
            }
        }
    }

    return rs;
}

} // namespace perception
} // namespace vexus

int main(int argc, char** argv) {
    using namespace vexus::perception;

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <road_structure.json>\n";
        return 1;
    }

    auto rs = LoadRoadStructure(argv[1]);
    if (!rs) {
        std::cerr << "Failed to load road structure.\n";
        return 1;
    }

    std::cout << "=== RoadStructure loaded ===\n";
    std::cout << "Header timestamp_ns : " << rs->header.timestamp_ns << "\n";
    std::cout << "Header sensor_id    : " << rs->header.sensor_id    << "\n";
    std::cout << "Header coord_type   : " << static_cast<int>(rs->header.coord_type) << "\n";
    std::cout << "\n--- SemanticMapData ---\n";
    std::cout << "  lines           : " << rs->semantic_map_data.lines.size()          << "\n";
    std::cout << "  stop_lines      : " << rs->semantic_map_data.stop_lines.size()     << "\n";
    std::cout << "  traffic_lights  : " << rs->semantic_map_data.traffic_lights.size() << "\n";
    std::cout << "  cross_walks     : " << rs->semantic_map_data.cross_walks.size()    << "\n";
    std::cout << "  areas           : " << rs->semantic_map_data.areas.size()          << "\n";
    std::cout << "  poles           : " << rs->semantic_map_data.poles.size()          << "\n";
    std::cout << "  speed_bumps     : " << rs->semantic_map_data.speed_bumps.size()    << "\n";
    std::cout << "  traffic_signs   : " << rs->semantic_map_data.traffic_signs.size()  << "\n";
    std::cout << "  road_markers    : " << rs->semantic_map_data.road_markers.size()   << "\n";
    std::cout << "  traffic_cones   : " << rs->semantic_map_data.traffic_cones.size()  << "\n";
    std::cout << "\n--- RouteMapData ---\n";
    std::cout << "  lanes           : " << rs->route_map_data.lanes.size()         << "\n";
    std::cout << "  sections        : " << rs->route_map_data.sections.size()      << "\n";
    std::cout << "  lane_links      : " << rs->route_map_data.lane_links.size()    << "\n";
    std::cout << "  section_links   : " << rs->route_map_data.section_links.size() << "\n";
    std::cout << "  junctions       : " << rs->route_map_data.junctions.size()     << "\n";
    std::cout << "\n--- EgoTrajData ---\n";
    std::cout << "  trajs           : " << rs->ego_layer_data.trajs.size()      << "\n";
    std::cout << "  ego_lanes       : " << rs->ego_layer_data.ego_lanes.size()  << "\n";

    if (!rs->semantic_map_data.lines.empty()) {
        const auto& [id, line] = *rs->semantic_map_data.lines.begin();
        std::cout << "\nFirst line id=" << line.id
                  << "  segments=" << line.line_segments.size()
                  << "  points_in_seg0=";
        if (!line.line_segments.empty())
            std::cout << line.line_segments[0].points.size();
        else
            std::cout << "0";
        std::cout << "\n";
    }

    if (!rs->route_map_data.lanes.empty()) {
        const auto& [id, lane] = *rs->route_map_data.lanes.begin();
        std::cout << "First lane id=" << lane.id
                  << "  section_id=" << lane.section_id
                  << "  center_points=" << lane.center_points.size()
                  << "  max_speed=" << lane.max_speed << "\n";
    }

    return 0;
}

/**
 * save_lddata_to_json.h
 *
 * Provides:
 *   inline bool SaveLDDataToJson(const ldmap::LDData& ld_data,
 *                                const std::string& file_path,
 *                                int indent = 2)
 *
 * Serialises every field of LDData (and all nested structs) to a JSON file
 * using nlohmann/json.  ID values (uint64_t) are stored as decimal strings
 * to avoid precision loss in JSON parsers that treat numbers as doubles.
 *
 * The produced JSON mirrors the format consumed by LoadRoadStructure in
 * tools/load_road_structure/load_road_structure.cpp so the two tools are
 * interoperable.
 *
 * Usage:
 *   #include "lddata_types.h"
 *   #include "save_lddata_to_json.h"
 *   ...
 *   ldmap::LDData ld;
 *   // ... fill ld ...
 *   bool ok = SaveLDDataToJson(ld, "/tmp/output.json");
 */

#pragma once

#include <fstream>
#include <string>

#include "json.hpp"
#include "lddata_types.h"

using json = nlohmann::json;

// ============================================================
//  Internal helpers (anonymous namespace for linkage safety)
// ============================================================

namespace {

inline std::string idToStr(ldmap::id_t id) {
    return std::to_string(id);
}

inline json serializePoint3d(const ldmap::Point3d& p) {
    return json{{"x", p.x}, {"y", p.y}, {"z", p.z}, {"weight", p.weight}};
}

inline json serializePoint3dArray(const std::vector<ldmap::Point3d>& pts) {
    json arr = json::array();
    for (const auto& p : pts) arr.push_back(serializePoint3d(p));
    return arr;
}

inline json serializeIdArray(const std::vector<ldmap::id_t>& ids) {
    json arr = json::array();
    for (ldmap::id_t id : ids) arr.push_back(idToStr(id));
    return arr;
}

inline json serializeFloatArray(const std::vector<float>& v) {
    json arr = json::array();
    for (float f : v) arr.push_back(f);
    return arr;
}

inline json serializeQuaternion(const ldmap::Quaternion& q) {
    return json{{"qw", q.qw}, {"qx", q.qx}, {"qy", q.qy}, {"qz", q.qz}};
}

inline json serializeObjectData(const ldmap::ObjectData& o) {
    return json{
        {"id",               idToStr(o.id)},
        {"centroid",         serializePoint3d(o.centroid)},
        {"detection_status", o.detection_status},
        {"confidence",       o.confidence},
        {"quaternion",       serializeQuaternion(o.quaternion)},
        {"length",           o.length},
        {"width",            o.width},
        {"height",           o.height},
        {"lane_ids",         serializeIdArray(o.lane_ids)},
        {"confidence_level", o.confidence_level},
        {"junction_id",      idToStr(o.junction_id)},
        {"centroid_lla",     serializePoint3d(o.centroid_lla)},
        {"centroid_ego",     serializePoint3d(o.centroid_ego)},
    };
}

inline json serializeAreaData(const ldmap::AreaData& a) {
    return json{
        {"id",               idToStr(a.id)},
        {"detection_status", a.detection_status},
        {"confidence",       a.confidence},
        {"area_type",        a.area_type},
        {"boundary",         serializePoint3dArray(a.boundary)},
        {"lane_ids",         serializeIdArray(a.lane_ids)},
        {"boundary_lla",     serializePoint3dArray(a.boundary_lla)},
        {"boundary_ego",     serializePoint3dArray(a.boundary_ego)},
    };
}

inline json serializeJunctionData(const ldmap::JunctionData& jd) {
    return json{
        {"id",                      idToStr(jd.id)},
        {"boundary",                serializePoint3dArray(jd.boundary)},
        {"from_section_ids",        serializeIdArray(jd.from_section_ids)},
        {"to_section_ids",          serializeIdArray(jd.to_section_ids)},
        {"junction_type",           jd.junction_type},
        {"force_using_map_version", jd.force_using_map_version},
        {"map_source",              jd.map_source},
        {"boundary_lla",            serializePoint3dArray(jd.boundary_lla)},
        {"boundary_ego",            serializePoint3dArray(jd.boundary_ego)},
    };
}

inline json serializeLaneData(const ldmap::LaneData& l) {
    return json{
        {"id",                   idToStr(l.id)},
        {"center_points",        serializePoint3dArray(l.center_points)},
        {"lane_type",            l.lane_type},
        {"turn_type",            l.turn_type},
        {"trans_type",           l.trans_type},
        {"lane_direction",       l.lane_direction},
        {"left_line_id",         idToStr(l.left_line_id)},
        {"right_line_id",        idToStr(l.right_line_id)},
        {"section_id",           idToStr(l.section_id)},
        {"intersection_id",      idToStr(l.intersection_id)},
        {"left_lane_id",         idToStr(l.left_lane_id)},
        {"right_lane_id",        idToStr(l.right_lane_id)},
        {"length",               l.length},
        {"lane_widths",          serializeFloatArray(l.lane_widths)},
        {"headings",             serializeFloatArray(l.headings)},
        {"longitudinal_slopes",  serializeFloatArray(l.longitudinal_slopes)},
        {"lateral_slopes",       serializeFloatArray(l.lateral_slopes)},
        {"curvature_radius",     serializeFloatArray(l.curvature_radius)},
        {"max_speed",            l.max_speed},
        {"min_speed",            l.min_speed},
        {"successor_link_ids",   serializeIdArray(l.successor_link_ids)},
        {"predecessor_link_ids", serializeIdArray(l.predecessor_link_ids)},
        {"stop_line_ids",        serializeIdArray(l.stop_line_ids)},
        {"cross_walk_ids",       serializeIdArray(l.cross_walk_ids)},
        {"speed_bump_ids",       serializeIdArray(l.speed_bump_ids)},
        {"area_ids",             serializeIdArray(l.area_ids)},
        {"detection_status",     l.detection_status},
        {"visual_diff",          l.visual_diff},
        {"is_opposite",          l.is_opposite},
        {"experience_spdlmt",    l.experience_spdlmt},
        {"force_avoidance_line", l.force_avoidance_line},
        {"center_points_lla",    serializePoint3dArray(l.center_points_lla)},
        {"center_points_ego",    serializePoint3dArray(l.center_points_ego)},
    };
}

inline json serializeSectionData(const ldmap::SectionData& s) {
    return json{
        {"id",                   idToStr(s.id)},
        {"center_points",        serializePoint3dArray(s.center_points)},
        {"section_class",        s.section_class},
        {"section_type",         s.section_type},
        {"section_direction",    s.section_direction},
        {"positive_slope",       s.positive_slope},
        {"toll_type",            s.toll_type},
        {"is_routing_section",   s.is_routing_section},
        {"intersection_id",      idToStr(s.intersection_id)},
        {"length",               s.length},
        {"lane_ids",             serializeIdArray(s.lane_ids)},
        {"left_boundary_ids",    serializeIdArray(s.left_boundary_ids)},
        {"right_boundary_ids",   serializeIdArray(s.right_boundary_ids)},
        {"successor_link_ids",   serializeIdArray(s.successor_link_ids)},
        {"predecessor_link_ids", serializeIdArray(s.predecessor_link_ids)},
        {"traffic_light_ids",    serializeIdArray(s.traffic_light_ids)},
        {"traffic_sign_ids",     serializeIdArray(s.traffic_sign_ids)},
        {"from_junction_ids",    serializeIdArray(s.from_junction_ids)},
        {"to_junction_ids",      serializeIdArray(s.to_junction_ids)},
        {"opposite_section_ids", serializeIdArray(s.opposite_section_ids)},
        {"is_strategic_section", s.is_strategic_section},
        {"is_sparsify",          s.is_sparsify},
        {"experience_spd",       s.experience_spd},
        {"force_using_map",      s.force_using_map},
        {"center_points_lla",    serializePoint3dArray(s.center_points_lla)},
        {"center_points_ego",    serializePoint3dArray(s.center_points_ego)},
    };
}

inline json serializeLaneLinkData(const ldmap::LaneLinkData& ll) {
    return json{
        {"id",              idToStr(ll.id)},
        {"from_lane_id",    idToStr(ll.from_lane_id)},
        {"to_lane_id",      idToStr(ll.to_lane_id)},
        {"link_type",       ll.link_type},
        {"ref_points",      serializePoint3dArray(ll.ref_points)},
        {"section_link_id", idToStr(ll.section_link_id)},
    };
}

inline json serializeCrosswalkData(const ldmap::CrosswalkData& cw) {
    return json{
        {"id",               idToStr(cw.id)},
        {"detection_status", cw.detection_status},
        {"confidence",       cw.confidence},
        {"boundary",         serializePoint3dArray(cw.boundary)},
        {"lane_ids",         serializeIdArray(cw.lane_ids)},
        {"boundary_lla",     serializePoint3dArray(cw.boundary_lla)},
        {"boundary_ego",     serializePoint3dArray(cw.boundary_ego)},
    };
}

inline json serializeStopLineData(const ldmap::StopLineData& sl) {
    return json{
        {"id",               idToStr(sl.id)},
        {"points",           serializePoint3dArray(sl.points)},
        {"lane_ids",         serializeIdArray(sl.lane_ids)},
        {"detection_status", sl.detection_status},
        {"confidence",       sl.confidence},
        {"points_lla",       serializePoint3dArray(sl.points_lla)},
        {"points_ego",       serializePoint3dArray(sl.points_ego)},
    };
}

inline json serializeLineSegmentData(const ldmap::LineSegmentData& seg) {
    return json{
        {"id",               idToStr(seg.id)},
        {"line_type",        seg.line_type},
        {"line_style",       seg.line_style},
        {"color",            seg.color},
        {"width",            seg.width},
        {"detection_status", seg.detection_status},
        {"confidence",       seg.confidence},
        {"points",           serializePoint3dArray(seg.points)},
        {"points_lla",       serializePoint3dArray(seg.points_lla)},
        {"points_ego",       serializePoint3dArray(seg.points_ego)},
    };
}

inline json serializeLineData(const ldmap::LineData& ld) {
    json segs = json::array();
    for (const auto& seg : ld.line_segments) segs.push_back(serializeLineSegmentData(seg));
    return json{
        {"id",            idToStr(ld.id)},
        {"left_lane_id",  idToStr(ld.left_lane_id)},
        {"right_lane_id", idToStr(ld.right_lane_id)},
        {"line_segments", segs},
        {"obs_age",       std::to_string(ld.obs_age)},
        {"is_opposite",   ld.is_opposite},
    };
}

inline json serializeTrafficLightData(const ldmap::TrafficLightData& tl) {
    return json{
        {"object_data", serializeObjectData(tl.object_data)},
        {"distance",    tl.distance},
        {"countdown",   tl.countdown},
        {"status",      tl.status},
        {"countdown_d", tl.countdown_d},
    };
}

inline json serializeTrafficSignData(const ldmap::TrafficSignData& ts) {
    return json{
        {"object_data", serializeObjectData(ts.object_data)},
        {"type",        ts.type},
    };
}

inline json serializeTrafficConeData(const ldmap::TrafficConeData& tc) {
    return json{
        {"object_data",    serializeObjectData(tc.object_data)},
        {"type",           tc.type},
        {"obs_age",        tc.obs_age},
        {"attribute_age",  tc.attribute_age},
        {"loc_estimation", tc.loc_estimation},
    };
}

inline json serializeRoadMarkerData(const ldmap::RoadMarkerData& rm) {
    return json{
        {"object_data",      serializeObjectData(rm.object_data)},
        {"type",             rm.type},
        {"orientation_type", rm.orientation_type},
        {"color",            rm.color},
        {"semantic",         rm.semantic},
        {"obs_age",          rm.obs_age},
        {"attribute_age",    rm.attribute_age},
    };
}

inline json serializePoleData(const ldmap::PoleData& p) {
    return json{
        {"id",               idToStr(p.id)},
        {"detection_status", p.detection_status},
        {"confidence",       p.confidence},
        {"lane_ids",         serializeIdArray(p.lane_ids)},
        {"bottom_point",     serializePoint3d(p.bottom_point)},
        {"top_point",        serializePoint3d(p.top_point)},
        {"type",             p.type},
    };
}

inline json serializeSpeedBumpData(const ldmap::SpeedBumpData& sb) {
    return json{
        {"id",               idToStr(sb.id)},
        {"detection_status", sb.detection_status},
        {"confidence",       sb.confidence},
        {"type",             sb.type},
        {"points",           serializePoint3dArray(sb.points)},
        {"lane_ids",         serializeIdArray(sb.lane_ids)},
        {"points_lla",       serializePoint3dArray(sb.points_lla)},
        {"points_ego",       serializePoint3dArray(sb.points_ego)},
    };
}

template <typename T, typename Fn>
json serializeArray(const std::vector<T>& vec, Fn&& fn) {
    json arr = json::array();
    for (const auto& item : vec) arr.push_back(fn(item));
    return arr;
}

} // anonymous namespace

// ============================================================
//  Public API
// ============================================================

/**
 * Serialize @p ld_data to a JSON file at @p file_path.
 *
 * @param ld_data    The LDData instance to serialize.
 * @param file_path  Destination file path (created or overwritten).
 * @param indent     JSON pretty-print indentation width in spaces (default 2).
 *                   Pass -1 for compact (no whitespace) output.
 * @return true on success, false if the file could not be opened or written.
 */
inline bool SaveLDDataToJson(const ldmap::LDData& ld_data,
                             const std::string& file_path,
                             int indent = 2) {
    json root;

    root["areas"]          = serializeArray(ld_data.areas,          serializeAreaData);
    root["cross_walks"]    = serializeArray(ld_data.cross_walks,     serializeCrosswalkData);
    root["sections"]       = serializeArray(ld_data.sections,        serializeSectionData);
    root["lanes"]          = serializeArray(ld_data.lanes,           serializeLaneData);
    root["lines"]          = serializeArray(ld_data.lines,           serializeLineData);
    root["stop_lines"]     = serializeArray(ld_data.stop_lines,      serializeStopLineData);
    root["junctions"]      = serializeArray(ld_data.junctions,       serializeJunctionData);
    root["traffic_lights"] = serializeArray(ld_data.traffic_lights,  serializeTrafficLightData);
    root["traffic_signs"]  = serializeArray(ld_data.traffic_signs,   serializeTrafficSignData);
    root["road_markers"]   = serializeArray(ld_data.road_markers,    serializeRoadMarkerData);
    root["poles"]          = serializeArray(ld_data.poles,           serializePoleData);
    root["speed_bumps"]    = serializeArray(ld_data.speed_bumps,     serializeSpeedBumpData);
    root["traffic_cones"]  = serializeArray(ld_data.traffic_cones,   serializeTrafficConeData);
    root["lane_line"]      = serializeArray(ld_data.lane_line,       serializeLaneLinkData);

    {
        json arr = json::array();
        for (uint64_t id : ld_data.mpp_ids) arr.push_back(std::to_string(id));
        root["mpp_ids"] = std::move(arr);
    }

    std::ofstream ofs(file_path);
    if (!ofs.is_open()) {
        return false;
    }
    try {
        ofs << root.dump(indent);
        ofs.flush();
        return ofs.good();
    } catch (const std::exception&) {
        return false;
    }
}

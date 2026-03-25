#pragma once

#include "vision_type/vision_type.hpp"
#include "multiperception/common/nlohmann/json.hpp"

#include <fstream>
#include <string>

namespace vexus {
namespace vision {

using json = nlohmann::json;

// ─────────────────────────────────────────────
//  基础类型
// ─────────────────────────────────────────────

inline void to_json(json &j, const Point3d &p) {
    j = json{{"x", p.x}, {"y", p.y}, {"z", p.z}};
}

inline void to_json(json &j, const Quaternion &q) {
    j = json{{"qw", q.qw}, {"qx", q.qx}, {"qy", q.qy}, {"qz", q.qz}};
}

// ─────────────────────────────────────────────
//  RoadStructureBaseData
// ─────────────────────────────────────────────

inline void to_json(json &j, const RoadStructureBaseData &d) {
    j = json{
        {"id",               d.id},
        {"detection_status", d.detection_status},
        {"confidence",       d.confidence}
    };
}

// ─────────────────────────────────────────────
//  Route Map Data
// ─────────────────────────────────────────────

inline void to_json(json &j, const JunctionData &d) {
    j = json{
        {"id",                    d.id},
        {"junction_type",         d.junction_type},
        {"boundary",              d.boundary},
        {"from_section_ids",      d.from_section_ids},
        {"to_section_ids",        d.to_section_ids},
        {"force_using_map_version", d.force_using_map_version},
        {"map_source",            d.map_source},
        {"boundary_lla",          d.boundary_lla},
        {"boundary_ego",          d.boundary_ego}
    };
}

inline void to_json(json &j, const LaneData &d) {
    j = json{
        {"id",                   d.id},
        {"lane_type",            d.lane_type},
        {"turn_type",            d.turn_type},
        {"trans_type",           d.trans_type},
        {"lane_direction",       d.lane_direction},
        {"left_line_id",         d.left_line_id},
        {"right_line_id",        d.right_line_id},
        {"center_points",        d.center_points},
        {"section_id",           d.section_id},
        {"intersection_id",      d.intersection_id},
        {"left_lane_id",         d.left_lane_id},
        {"right_lane_id",        d.right_lane_id},
        {"length",               d.length},
        {"lane_widths",          d.lane_widths},
        {"headings",             d.headings},
        {"longitudinal_slopes",  d.longitudinal_slopes},
        {"lateral_slopes",       d.lateral_slopes},
        {"curvature_radius",     d.curvature_radius},
        {"max_speed",            d.max_speed},
        {"min_speed",            d.min_speed},
        {"successor_link_ids",   d.successor_link_ids},
        {"predecessor_link_ids", d.predecessor_link_ids},
        {"stop_line_ids",        d.stop_line_ids},
        {"cross_walk_ids",       d.cross_walk_ids},
        {"speed_bump_ids",       d.speed_bump_ids},
        {"area_ids",             d.area_ids},
        {"detection_status",     d.detection_status},
        {"visual_diff",          d.visual_diff},
        {"is_opposite",          d.is_opposite},
        {"experience_spdlmt",    d.experience_spdlmt},
        {"force_avoidance_line", d.force_avoidance_line},
        {"center_points_lla",    d.center_points_lla},
        {"center_points_ego",    d.center_points_ego}
    };
}

inline void to_json(json &j, const SectionData &d) {
    j = json{
        {"id",                   d.id},
        {"section_class",        d.section_class},
        {"section_type",         d.section_type},
        {"section_direction",    d.section_direction},
        {"positive_slope",       d.positive_slope},
        {"toll_type",            d.toll_type},
        {"is_routing_section",   d.is_routing_section},
        {"center_points",        d.center_points},
        {"intersection_id",      d.intersection_id},
        {"length",               d.length},
        {"lane_ids",             d.lane_ids},
        {"left_boundary_ids",    d.left_boundary_ids},
        {"right_boundary_ids",   d.right_boundary_ids},
        {"successor_link_ids",   d.successor_link_ids},
        {"predecessor_link_ids", d.predecessor_link_ids},
        {"traffic_light_ids",    d.traffic_light_ids},
        {"traffic_sign_ids",     d.traffic_sign_ids},
        {"from_junction_ids",    d.from_junction_ids},
        {"to_junction_ids",      d.to_junction_ids},
        {"opposite_section_ids", d.opposite_section_ids},
        {"is_strategic_section", d.is_strategic_section},
        {"is_sparsify",          d.is_sparsify},
        {"experience_spd",       d.experience_spd},
        {"force_using_map",      d.force_using_map},
        {"center_points_lla",    d.center_points_lla},
        {"center_points_ego",    d.center_points_ego}
    };
}

inline void to_json(json &j, const LaneLinkData &d) {
    j = json{
        {"id",             d.id},
        {"from_lane_id",   d.from_lane_id},
        {"to_lane_id",     d.to_lane_id},
        {"link_type",      d.link_type},
        {"ref_points",     d.ref_points},
        {"section_link_id", d.section_link_id}
    };
}

// ─────────────────────────────────────────────
//  Semantic Map Data
// ─────────────────────────────────────────────

inline void to_json(json &j, const AreaData &d) {
    j = json{
        {"id",               d.id},
        {"detection_status", d.detection_status},
        {"confidence",       d.confidence},
        {"area_type",        d.area_type},
        {"boundary",         d.boundary},
        {"lane_ids",         d.lane_ids},
        {"boundary_lla",     d.boundary_lla},
        {"boundary_ego",     d.boundary_ego}
    };
}

inline void to_json(json &j, const CrosswalkData &d) {
    j = json{
        {"id",               d.id},
        {"detection_status", d.detection_status},
        {"confidence",       d.confidence},
        {"boundary",         d.boundary},
        {"lane_ids",         d.lane_ids},
        {"boundary_lla",     d.boundary_lla},
        {"boundary_ego",     d.boundary_ego}
    };
}

inline void to_json(json &j, const StopLineData &d) {
    j = json{
        {"id",               d.id},
        {"points",           d.points},
        {"lane_ids",         d.lane_ids},
        {"detection_status", d.detection_status},
        {"confidence",       d.confidence},
        {"points_lla",       d.points_lla},
        {"points_ego",       d.points_ego}
    };
}

inline void to_json(json &j, const LineSegmentData &d) {
    j = json{
        {"id",               d.id},
        {"line_type",        d.line_type},
        {"line_style",       d.line_style},
        {"color",            d.color},
        {"width",            d.width},
        {"detection_status", d.detection_status},
        {"confidence",       d.confidence},
        {"points",           d.points},
        {"points_lla",       d.points_lla},
        {"points_ego",       d.points_ego}
    };
}

inline void to_json(json &j, const LineData &d) {
    j = json{
        {"id",            d.id},
        {"left_lane_id",  d.left_lane_id},
        {"right_lane_id", d.right_lane_id},
        {"line_segments", d.line_segments},
        {"obs_age",       d.obs_age},
        {"is_opposite",   d.is_opposite}
    };
}

inline void to_json(json &j, const ObjectData &d) {
    j = json{
        {"id",               d.id},
        {"centroid",         d.centroid},
        {"detection_status", d.detection_status},
        {"confidence",       d.confidence},
        {"quaternion",       d.quaternion},
        {"length",           d.length},
        {"width",            d.width},
        {"height",           d.height},
        {"lane_ids",         d.lane_ids},
        {"confidence_level", d.confidence_level},
        {"junction_id",      d.junction_id},
        {"centroid_lla",     d.centroid_lla},
        {"centroid_ego",     d.centroid_ego}
    };
}

inline void to_json(json &j, const TrafficLightData &d) {
    json obj_j;
    to_json(obj_j, d.object_data);
    j = obj_j;
    j["distance"]    = d.distance;
    j["countdown"]   = d.countdown;
    j["status"]      = d.status;
    j["countdown_d"] = d.countdown_d;
}

inline void to_json(json &j, const TrafficSignData &d) {
    json obj_j;
    to_json(obj_j, d.object_data);
    j = obj_j;
    j["type"] = d.type;
}

inline void to_json(json &j, const TrafficConeData &d) {
    json obj_j;
    to_json(obj_j, d.object_data);
    j = obj_j;
    j["type"]          = d.type;
    j["obs_age"]       = d.obs_age;
    j["attribute_age"] = d.attribute_age;
    j["loc_estimation"] = d.loc_estimation;
}

inline void to_json(json &j, const RoadMarkerData &d) {
    json obj_j;
    to_json(obj_j, d.object_data);
    j = obj_j;
    j["type"]             = d.type;
    j["orientation_type"] = d.orientation_type;
    j["color"]            = d.color;
    j["semantic"]         = d.semantic;
    j["obs_age"]          = d.obs_age;
    j["attribute_age"]    = d.attribute_age;
}

inline void to_json(json &j, const PoleData &d) {
    j = json{
        {"id",               d.id},
        {"detection_status", d.detection_status},
        {"confidence",       d.confidence},
        {"lane_ids",         d.lane_ids},
        {"bottom_point",     d.bottom_point},
        {"top_point",        d.top_point},
        {"type",             d.type}
    };
}

inline void to_json(json &j, const SpeedBumpData &d) {
    j = json{
        {"id",               d.id},
        {"detection_status", d.detection_status},
        {"confidence",       d.confidence},
        {"type",             d.type},
        {"points",           d.points},
        {"lane_ids",         d.lane_ids},
        {"points_lla",       d.points_lla},
        {"points_ego",       d.points_ego}
    };
}

// ─────────────────────────────────────────────
//  顶层：LDData
// ─────────────────────────────────────────────

inline void to_json(json &j, const LDData &d) {
    j = json{
        {"areas",          d.areas},
        {"cross_walks",    d.cross_walks},
        {"sections",       d.sections},
        {"lanes",          d.lanes},
        {"lines",          d.lines},
        {"stop_lines",     d.stop_lines},
        {"junctions",      d.junctions},
        {"traffic_lights", d.traffic_lights},
        {"traffic_signs",  d.traffic_signs},
        {"road_markers",   d.road_markers},
        {"poles",          d.poles},
        {"speed_bumps",    d.speed_bumps},
        {"traffic_cones",  d.traffic_cones},
        {"mpp_ids",        d.mpp_ids},
        {"lane_line",      d.lane_line}
    };
}

// ─────────────────────────────────────────────
//  便捷工具函数
// ─────────────────────────────────────────────

/**
 * @brief 将 LDData 序列化并写入 JSON 文件
 * @param ld_data   待序列化数据
 * @param file_path 输出文件路径（如 "/tmp/lddata.json"）
 * @param indent    缩进空格数，-1 表示紧凑模式
 * @return true  写入成功
 * @return false 文件打开失败
 */
inline bool SaveLDDataToJson(const LDData &ld_data,
                             const std::string &file_path,
                             int indent = 2) {
    std::ofstream ofs(file_path);
    if (!ofs.is_open()) {
        return false;
    }
    json j;
    to_json(j, ld_data);
    ofs << j.dump(indent);
    return ofs.good();
}

}  // namespace vision
}  // namespace vexus

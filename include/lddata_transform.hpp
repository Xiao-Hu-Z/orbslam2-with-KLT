#pragma once

#include "vision_type/vision_type.hpp"

namespace vexus {
namespace vision {

/**
 * @brief 将 LDData 中所有 _lla 坐标字段转换为 ENU 坐标，
 *        结果同时写入对应的 _ego 字段和 base 字段（center_points / boundary / points / centroid）。
 *
 * 逻辑对应 Python：
 *   lla_converter = LLAConverter()
 *   lla_converter.set_origin([lla_ref[0], lla_ref[1], lla_ref[2]])
 *   enu_pt = lla_converter.lla2enu([lla[0], lla[1], lla[2]])
 *
 * @param ld_data   待转换数据（in-place 修改）
 * @param lla_ref   ENU 原点，格式与 Point3d 一致（x=lon, y=lat, z=alt）
 * @return true  转换成功
 * @return false ld_data 为空指针
 */
inline bool TransformLDDataToENU(LDData *ld_data, const Point3d &lla_ref) {
    if (ld_data == nullptr) {
        return false;
    }

    // 对应 Python：lla_converter = LLAConverter(); lla_converter.set_origin(lla_ref)
    LLAConverter lla_converter;
    lla_converter.SetOrigin(PointLLH(lla_ref.x, lla_ref.y, lla_ref.z));

    // 对应 Python：enu_pt = lla_converter.lla2enu([lla[0], lla[1], lla[2]])
    auto lla_to_enu = [&](const Point3d &lla_pt) -> Point3d {
        Point3D result;
        lla_converter.LLA2XYZ(PointLLH(lla_pt.x, lla_pt.y, lla_pt.z), &result);
        return Point3d{static_cast<float>(result.x),
                       static_cast<float>(result.y),
                       static_cast<float>(result.z)};
    };

    // 批量转换点列表，同时写入 ego 容器和 base 容器
    auto transform_points = [&](const std::vector<Point3d> &lla_pts,
                                std::vector<Point3d>       &ego_pts,
                                std::vector<Point3d>       &base_pts) {
        ego_pts.resize(lla_pts.size());
        base_pts.resize(lla_pts.size());
        for (size_t i = 0; i < lla_pts.size(); ++i) {
            ego_pts[i]  = lla_to_enu(lla_pts[i]);
            base_pts[i] = ego_pts[i];
        }
    };

    // 转换 ObjectData 的质心（centroid_lla → centroid_ego + centroid）
    auto transform_centroid = [&](ObjectData &obj) {
        Point3d enu      = lla_to_enu(obj.centroid_lla);
        obj.centroid_ego = enu;
        obj.centroid     = enu;
    };

    // ── 1. 车道中心线 ──────────────────────────────────────────────────────
    for (auto &lane : ld_data->lanes) {
        transform_points(lane.center_points_lla,
                         lane.center_points_ego,
                         lane.center_points);
    }

    // ── 2. Section 中心线 ──────────────────────────────────────────────────
    for (auto &section : ld_data->sections) {
        transform_points(section.center_points_lla,
                         section.center_points_ego,
                         section.center_points);
    }

    // ── 3. Junction 边界 ───────────────────────────────────────────────────
    for (auto &junction : ld_data->junctions) {
        transform_points(junction.boundary_lla,
                         junction.boundary_ego,
                         junction.boundary);
    }

    // ── 4. 车道线（LineData -> LineSegmentData）────────────────────────────
    for (auto &line : ld_data->lines) {
        for (auto &seg : line.line_segments) {
            transform_points(seg.points_lla,
                             seg.points_ego,
                             seg.points);
        }
    }

    // ── 5. 停车线 ──────────────────────────────────────────────────────────
    for (auto &stop_line : ld_data->stop_lines) {
        transform_points(stop_line.points_lla,
                         stop_line.points_ego,
                         stop_line.points);
    }

    // ── 6. Area 边界 ───────────────────────────────────────────────────────
    for (auto &area : ld_data->areas) {
        transform_points(area.boundary_lla,
                         area.boundary_ego,
                         area.boundary);
    }

    // ── 7. 人行横道边界 ────────────────────────────────────────────────────
    for (auto &cw : ld_data->cross_walks) {
        transform_points(cw.boundary_lla,
                         cw.boundary_ego,
                         cw.boundary);
    }

    // ── 8. 减速带 ──────────────────────────────────────────────────────────
    for (auto &bump : ld_data->speed_bumps) {
        transform_points(bump.points_lla,
                         bump.points_ego,
                         bump.points);
    }

    // ── 9. 红绿灯 ──────────────────────────────────────────────────────────
    for (auto &tl : ld_data->traffic_lights) {
        transform_centroid(tl.object_data);
    }

    // ── 10. 交通标志 ───────────────────────────────────────────────────────
    for (auto &ts : ld_data->traffic_signs) {
        transform_centroid(ts.object_data);
    }

    // ── 11. 路面标记 ───────────────────────────────────────────────────────
    for (auto &rm : ld_data->road_markers) {
        transform_centroid(rm.object_data);
    }

    // ── 12. 交通锥 ─────────────────────────────────────────────────────────
    for (auto &cone : ld_data->traffic_cones) {
        transform_centroid(cone.object_data);
    }

    return true;
}

}  // namespace vision
}  // namespace vexus

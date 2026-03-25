/**
 * main.cpp – Demo for SaveLDDataToJson
 *
 * Builds a small synthetic LDData object (one of each major collection entry)
 * and writes it to the path supplied on the command line (default: output.json).
 *
 * Build:
 *   cmake -B build && cmake --build build
 *
 * Usage:
 *   ./build/save_lddata [output_path]
 */

#include <cstdint>
#include <iostream>
#include <string>

#include "lddata_types.h"
#include "save_lddata_to_json.h"

using namespace ldmap;

int main(int argc, char** argv) {
    const std::string output_path = (argc >= 2) ? argv[1] : "output.json";

    LDData ld;

    // ---- area ----
    {
        AreaData a;
        a.id         = 100001ULL;
        a.area_type  = 2;
        a.confidence = 98.5f;
        a.boundary.push_back({1.0f, 2.0f, 0.0f});
        a.boundary.push_back({3.0f, 2.0f, 0.0f});
        a.boundary.push_back({3.0f, 4.0f, 0.0f});
        a.boundary_lla.push_back({116.3f, 39.9f, 50.0f});
        a.lane_ids.push_back(200001ULL);
        ld.areas.push_back(std::move(a));
    }

    // ---- junction ----
    {
        JunctionData jd;
        jd.id            = 300001ULL;
        jd.junction_type = 3;
        jd.boundary.push_back({0.0f,  0.0f,  0.0f});
        jd.boundary.push_back({10.0f, 0.0f,  0.0f});
        jd.boundary.push_back({10.0f, 10.0f, 0.0f});
        jd.from_section_ids.push_back(400001ULL);
        jd.to_section_ids.push_back(400002ULL);
        ld.junctions.push_back(std::move(jd));
    }

    // ---- lane ----
    {
        LaneData lane;
        lane.id            = 200001ULL;
        lane.section_id    = 400001ULL;
        lane.lane_type     = 1;
        lane.turn_type     = 0x01;
        lane.length        = 35.6f;
        lane.max_speed     = 60.0f;
        lane.min_speed     = 0.0f;
        lane.left_line_id  = 500001ULL;
        lane.right_line_id = 500002ULL;
        lane.center_points.push_back({0.0f,  0.0f, 0.0f});
        lane.center_points.push_back({10.0f, 0.0f, 0.0f});
        lane.center_points.push_back({20.0f, 0.0f, 0.0f});
        lane.lane_widths          = {3.5f, 3.5f, 3.5f};
        lane.headings             = {0.0f, 0.01f, 0.02f};
        lane.longitudinal_slopes  = {0.0f, 0.0f, 0.0f};
        lane.lateral_slopes       = {0.0f, 0.0f, 0.0f};
        lane.curvature_radius     = {1000.0f, 1000.0f, 1000.0f};
        lane.successor_link_ids.push_back(600001ULL);
        lane.stop_line_ids.push_back(700001ULL);
        ld.lanes.push_back(std::move(lane));
    }

    // ---- section ----
    {
        SectionData sec;
        sec.id                 = 400001ULL;
        sec.section_type       = 1;
        sec.length             = 100.0f;
        sec.is_routing_section = true;
        sec.lane_ids.push_back(200001ULL);
        sec.center_points.push_back({0.0f,   0.0f, 0.0f});
        sec.center_points.push_back({50.0f,  0.0f, 0.0f});
        sec.center_points.push_back({100.0f, 0.0f, 0.0f});
        sec.from_junction_ids.push_back(300001ULL);
        ld.sections.push_back(std::move(sec));
    }

    // ---- line / line_segment ----
    {
        LineSegmentData seg;
        seg.id         = 550001ULL;
        seg.line_type  = 1;
        seg.line_style = 0;
        seg.color      = 1;
        seg.width      = 0.15f;
        seg.confidence = 95.0f;
        seg.points.push_back({0.0f,  1.75f, 0.0f});
        seg.points.push_back({10.0f, 1.75f, 0.0f});
        seg.points.push_back({20.0f, 1.75f, 0.0f});

        LineData line;
        line.id           = 500001ULL;
        line.left_lane_id = 200001ULL;
        line.line_segments.push_back(std::move(seg));
        ld.lines.push_back(std::move(line));
    }

    // ---- stop line ----
    {
        StopLineData sl;
        sl.id         = 700001ULL;
        sl.confidence = 99.0f;
        sl.points.push_back({-1.0f, -3.5f, 0.0f});
        sl.points.push_back({-1.0f,  3.5f, 0.0f});
        sl.lane_ids.push_back(200001ULL);
        ld.stop_lines.push_back(std::move(sl));
    }

    // ---- crosswalk ----
    {
        CrosswalkData cw;
        cw.id         = 800001ULL;
        cw.confidence = 97.0f;
        cw.boundary.push_back({-2.0f, -5.0f, 0.0f});
        cw.boundary.push_back({-2.0f,  5.0f, 0.0f});
        cw.boundary.push_back({ 2.0f,  5.0f, 0.0f});
        cw.boundary.push_back({ 2.0f, -5.0f, 0.0f});
        cw.lane_ids.push_back(200001ULL);
        ld.cross_walks.push_back(std::move(cw));
    }

    // ---- traffic light ----
    {
        TrafficLightData tl;
        tl.object_data.id         = 900001ULL;
        tl.object_data.centroid   = {5.0f, 0.0f, 5.0f};
        tl.object_data.confidence = 88.0f;
        tl.object_data.length     = 0.3f;
        tl.object_data.width      = 0.3f;
        tl.object_data.height     = 0.9f;
        tl.object_data.lane_ids.push_back(200001ULL);
        tl.distance    = 30.0;
        tl.countdown   = 15;
        tl.countdown_d = 14.7;
        tl.status      = 1;
        ld.traffic_lights.push_back(std::move(tl));
    }

    // ---- traffic sign ----
    {
        TrafficSignData ts;
        ts.object_data.id         = 910001ULL;
        ts.object_data.centroid   = {15.0f, 3.0f, 3.0f};
        ts.object_data.confidence = 90.0f;
        ts.type                   = 5;
        ld.traffic_signs.push_back(std::move(ts));
    }

    // ---- road marker ----
    {
        RoadMarkerData rm;
        rm.object_data.id         = 920001ULL;
        rm.object_data.centroid   = {8.0f, 0.0f, 0.0f};
        rm.object_data.confidence = 85.0f;
        rm.type                   = 2;
        rm.orientation_type       = 1;
        rm.color                  = 1;
        rm.semantic               = "arrow_straight";
        ld.road_markers.push_back(std::move(rm));
    }

    // ---- pole ----
    {
        PoleData p;
        p.id           = 930001ULL;
        p.confidence   = 92.0f;
        p.type         = 1;
        p.bottom_point = {3.0f, 5.0f, 0.0f};
        p.top_point    = {3.0f, 5.0f, 4.0f};
        p.lane_ids.push_back(200001ULL);
        ld.poles.push_back(std::move(p));
    }

    // ---- speed bump ----
    {
        SpeedBumpData sb;
        sb.id         = 940001ULL;
        sb.confidence = 91.0f;
        sb.type       = 1;
        sb.points.push_back({-0.5f, -3.5f, 0.0f});
        sb.points.push_back({-0.5f,  3.5f, 0.0f});
        sb.lane_ids.push_back(200001ULL);
        ld.speed_bumps.push_back(std::move(sb));
    }

    // ---- traffic cone ----
    {
        TrafficConeData tc;
        tc.object_data.id         = 950001ULL;
        tc.object_data.centroid   = {4.0f, 1.5f, 0.0f};
        tc.object_data.confidence = 80.0f;
        tc.type                   = 0;
        tc.obs_age                = 5;
        tc.loc_estimation         = 0;
        ld.traffic_cones.push_back(std::move(tc));
    }

    // ---- lane link ----
    {
        LaneLinkData ll;
        ll.id              = 600001ULL;
        ll.from_lane_id    = 200001ULL;
        ll.to_lane_id      = 200002ULL;
        ll.link_type       = 1;
        ll.section_link_id = 0ULL;
        ll.ref_points.push_back({20.0f, 0.0f, 0.0f});
        ld.lane_line.push_back(std::move(ll));
    }

    // ---- mpp_ids ----
    ld.mpp_ids.push_back(111111111111ULL);
    ld.mpp_ids.push_back(222222222222ULL);

    // ---- save ----
    std::cout << "Saving LDData to: " << output_path << "\n";
    const bool ok = SaveLDDataToJson(ld, output_path, 2);
    if (ok) {
        std::cout << "Success.\n";
        std::cout << "  areas          : " << ld.areas.size()         << "\n";
        std::cout << "  cross_walks    : " << ld.cross_walks.size()    << "\n";
        std::cout << "  sections       : " << ld.sections.size()       << "\n";
        std::cout << "  lanes          : " << ld.lanes.size()          << "\n";
        std::cout << "  lines          : " << ld.lines.size()          << "\n";
        std::cout << "  stop_lines     : " << ld.stop_lines.size()     << "\n";
        std::cout << "  junctions      : " << ld.junctions.size()      << "\n";
        std::cout << "  traffic_lights : " << ld.traffic_lights.size() << "\n";
        std::cout << "  traffic_signs  : " << ld.traffic_signs.size()  << "\n";
        std::cout << "  road_markers   : " << ld.road_markers.size()   << "\n";
        std::cout << "  poles          : " << ld.poles.size()          << "\n";
        std::cout << "  speed_bumps    : " << ld.speed_bumps.size()    << "\n";
        std::cout << "  traffic_cones  : " << ld.traffic_cones.size()  << "\n";
        std::cout << "  mpp_ids        : " << ld.mpp_ids.size()        << "\n";
        std::cout << "  lane_line      : " << ld.lane_line.size()      << "\n";
    } else {
        std::cerr << "Failed to write " << output_path << "\n";
        return 1;
    }
    return 0;
}

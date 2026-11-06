#include "raillab/analysis.hpp"

#include <iomanip>
#include <istream>
#include <ostream>
#include <sstream>
#include <stdexcept>

namespace rail {
void writeCsv(std::ostream &out, const Analysis &result) {
    out << "distance_m,time_s,x_m,height_m,speed_m_s,tangential_acceleration_m_s2,normal_acceleration_m_s2,"
           "total_acceleration_m_s2,normal_load_g,felt_load_g,rail_force_N,curvature_1_m,"
           "kinetic_energy_J,potential_energy_J,mechanical_energy_J,dissipated_energy_J,"
           "forward_force_N,propulsion_work_J,time_resolution_limited\n";
    out << std::setprecision(14);
    for (const auto &s : result.samples) {
        out << s.distance << ',' << s.time << ',' << s.track.position.x << ',' << s.track.position.y << ','
            << s.speed << ',' << s.tangentialAcceleration << ',' << s.normalAcceleration << ','
            << s.acceleration << ',' << s.normalG << ',' << s.feltG << ',' << s.railForce << ','
            << s.track.curvature << ',' << s.kineticEnergy << ',' << s.potentialEnergy << ','
            << s.mechanicalEnergy << ',' << s.dissipatedEnergy << ',' << s.forwardForce << ','
            << s.propulsionWork << ',' << (result.timeResolutionLimited ? 1 : 0) << '\n';
    }
    if (!out)
        throw std::runtime_error("Could not write the analysis CSV.");
}

std::vector<Vec2> readAnchorsCsv(std::istream &in) {
    std::vector<Vec2> points;
    std::string line;
    std::size_t number = 0;
    while (std::getline(in, line)) {
        ++number;
        if (line.empty() || line[0] == '#')
            continue;
        if (line == "x,y" || line == "x_m,y_m" || line == "x,y\r" || line == "x_m,y_m\r")
            continue;
        std::istringstream row(line);
        Vec2 point;
        char comma;
        if (!(row >> point.x >> comma >> point.y) || comma != ',' || (row >> std::ws && !row.eof()))
            throw std::invalid_argument("Invalid x,y CSV row at line " + std::to_string(number) + '.');
        points.push_back(point);
        if (points.size() > 128)
            throw std::invalid_argument("At most 128 points are supported.");
    }
    if (in.bad())
        throw std::runtime_error("Could not read the points CSV.");
    return points;
}
} // namespace rail

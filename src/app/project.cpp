#include "project.hpp"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

void Project::normalizeForces() {
    const std::size_t count = points.empty() ? 0 : points.size() - 1;
    rail::validateForwardForces(forwardForces, count);
    if (forwardForces.empty())
        forwardForces.resize(count, 0);
}

void Project::insertPoint(std::size_t index, rail::Vec2 point) {
    if (index > points.size())
        throw std::out_of_range("Invalid point insertion index.");
    normalizeForces();
    if (!points.empty()) {
        // A split inherits the old force on both halves. New end segments start at zero.
        const double force = index > 0 && index < points.size() ? forwardForces[index - 1] : 0;
        forwardForces.insert(forwardForces.begin() +
                                 static_cast<std::ptrdiff_t>(std::min(index, forwardForces.size())),
                             force);
    }
    points.insert(points.begin() + static_cast<std::ptrdiff_t>(index), point);
}

void Project::erasePoint(std::size_t index) {
    if (index >= points.size())
        throw std::out_of_range("Invalid point removal index.");
    normalizeForces();
    // Merging keeps the incoming segment's force and removes the outgoing force.
    if (!forwardForces.empty())
        forwardForces.erase(forwardForces.begin() +
                            static_cast<std::ptrdiff_t>(std::min(index, forwardForces.size() - 1)));
    points.erase(points.begin() + static_cast<std::ptrdiff_t>(index));
}

void Project::swapPoints(std::size_t first, std::size_t second) {
    if (first >= points.size() || second >= points.size())
        throw std::out_of_range("Invalid point reorder index.");
    normalizeForces();
    std::vector<std::size_t> order(points.size());
    std::iota(order.begin(), order.end(), 0);
    std::swap(order[first], order[second]);
    const auto oldForces = forwardForces;
    for (std::size_t i = 0; i < forwardForces.size(); ++i)
        // Keep only ordered endpoint pairs that still exist. New connections start at zero.
        forwardForces[i] = order[i + 1] == order[i] + 1 ? oldForces[order[i]] : 0;
    std::swap(points[first], points[second]);
}

Project preset(int index) {
    Project p;
    if (index == 0) {
        p.name = "Alpine descent";
        p.points = {{0, 42}, {18, 26}, {38, 4}, {58, 3}, {82, 23}, {104, 8}, {128, 5}, {150, 13}};
    } else if (index == 1) {
        p.name = "Vertical loop";
        p.points = {{-45, 48}, {-23, 18}, {0, 0}, {12, 12}, {0, 24}, {-12, 12}, {0, 0}, {25, 0}, {48, 9}};
        p.settings.initialSpeed = 5;
    } else if (index == 2) {
        p.name = "Energy barrier";
        p.points = {{0, 12}, {20, 2}, {40, 4}, {65, 30}, {90, 8}};
        p.settings.initialSpeed = 3;
    } else {
        p.name = "New track";
    }
    p.normalizeForces();
    return p;
}

namespace {
double required(const QJsonObject &object, const QString &key) {
    const auto v = object.value(key);
    if (!v.isDouble() || !std::isfinite(v.toDouble()))
        throw std::invalid_argument(("Missing or invalid setting: " + key).toStdString());
    return v.toDouble();
}
} // namespace
Project loadProject(const QString &path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        throw std::runtime_error(file.errorString().toStdString());
    if (file.size() > 1024 * 1024)
        throw std::invalid_argument("Project is too large (maximum 1 MB).");
    QJsonParseError error;
    const auto json = QJsonDocument::fromJson(file.readAll(), &error);
    if (error.error != QJsonParseError::NoError || !json.isObject())
        throw std::invalid_argument("This is not a valid RailLab JSON project.");
    const auto root = json.object();
    const double version = required(root, "version");
    if (root.value("format").toString() != "raillab" || (version != 1 && version != 2))
        throw std::invalid_argument("Unsupported project format or version.");
    Project p;
    p.name = root.value("name").toString("Untitled track").left(160);
    if (!root.value("points").isArray())
        throw std::invalid_argument("Project points must be an array.");
    for (const auto &entry : root.value("points").toArray()) {
        if (!entry.isArray())
            throw std::invalid_argument("Each point must be [x, y].");
        const auto a = entry.toArray();
        if (a.size() != 2 || !a[0].isDouble() || !a[1].isDouble())
            throw std::invalid_argument("Each point must have two numeric coordinates.");
        const rail::Vec2 point{a[0].toDouble(), a[1].toDouble()};
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || std::abs(point.x) > 10000 ||
            std::abs(point.y) > 10000)
            throw std::invalid_argument("Point coordinates are out of range.");
        p.points.push_back(point);
        if (p.points.size() > 128)
            throw std::invalid_argument("At most 128 points are supported.");
    }
    if (version == 2 || root.contains("segment_forces_N")) {
        const auto value = root.value("segment_forces_N");
        if (!value.isArray() ||
            value.toArray().size() != static_cast<qsizetype>(p.points.empty() ? 0 : p.points.size() - 1))
            throw std::invalid_argument("segment_forces_N must contain one force per segment.");
        for (const auto &force : value.toArray()) {
            if (!force.isDouble())
                throw std::invalid_argument("Each segment force must be a number in newtons.");
            p.forwardForces.push_back(force.toDouble());
        }
    }
    p.normalizeForces();
    const auto settings = root.value("settings").toObject();
    p.settings.initialSpeed = required(settings, "initial_speed_m_s");
    p.settings.mass = required(settings, "mass_kg");
    p.settings.gravity = required(settings, "gravity_m_s2");
    p.settings.resistance = required(settings, "resistance_m_s2");
    p.settings.dragArea = required(settings, "drag_area_m2");
    p.settings.airDensity = required(settings, "air_density_kg_m3");
    p.settings.step = required(settings, "distance_step_m");
    rail::validate(p.settings);
    if (p.points.size() >= 2) {
        const rail::Track check(p.points);
    }
    return p;
}
void saveProject(const QString &path, const Project &p) {
    rail::validate(p.settings);
    rail::validateForwardForces(p.forwardForces, p.points.empty() ? 0 : p.points.size() - 1);
    QJsonArray points;
    for (auto point : p.points)
        points.append(QJsonArray{point.x, point.y});
    QJsonArray forces;
    for (std::size_t i = 0; i + 1 < p.points.size(); ++i)
        forces.append(p.forwardForces.empty() ? 0 : p.forwardForces[i]);
    const QJsonObject settings{{"initial_speed_m_s", p.settings.initialSpeed},
                               {"mass_kg", p.settings.mass},
                               {"gravity_m_s2", p.settings.gravity},
                               {"resistance_m_s2", p.settings.resistance},
                               {"drag_area_m2", p.settings.dragArea},
                               {"air_density_kg_m3", p.settings.airDensity},
                               {"distance_step_m", p.settings.step}};
    const QJsonObject root{{"format", "raillab"}, {"version", 2},         {"name", p.name},
                           {"points", points},    {"settings", settings}, {"segment_forces_N", forces}};
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly))
        throw std::runtime_error(file.errorString().toStdString());
    const auto bytes = QJsonDocument(root).toJson(QJsonDocument::Indented);
    if (file.write(bytes) != bytes.size() || !file.commit())
        throw std::runtime_error(file.errorString().toStdString());
}

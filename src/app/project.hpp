#pragma once

#include "raillab/analysis.hpp"
#include <QString>

struct Project {
    QString name = "Untitled track";
    std::vector<rail::Vec2> points;
    rail::Settings settings;
    std::vector<double> forwardForces; // Signed N: positive propulsion, negative braking on segment i -> i+1.
    void normalizeForces();
    void insertPoint(std::size_t index, rail::Vec2 point);
    void erasePoint(std::size_t index);
    void swapPoints(std::size_t first, std::size_t second);
};
Project preset(int index);
Project loadProject(const QString &path);
void saveProject(const QString &path, const Project &project);

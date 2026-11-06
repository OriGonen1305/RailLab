#include "raillab/track.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace rail {
double dot(Vec2 a, Vec2 b) {
    return a.x * b.x + a.y * b.y;
}
double norm(Vec2 a) {
    return std::hypot(a.x, a.y);
}

Vec2 Bezier::position(double u) const {
    const double v = 1 - u;
    return p0 * (v * v * v) + p1 * (3 * v * v * u) + p2 * (3 * v * u * u) + p3 * (u * u * u);
}
Vec2 Bezier::derivative(double u) const {
    return (p1 - p0) * (3 * (1 - u) * (1 - u)) + (p2 - p1) * (6 * (1 - u) * u) + (p3 - p2) * (3 * u * u);
}
Vec2 Bezier::secondDerivative(double u) const {
    return (p2 - p1 * 2 + p0) * (6 * (1 - u)) + (p3 - p2 * 2 + p1) * (6 * u);
}

namespace {
double gauss(const Bezier &b, double a, double z) {
    constexpr std::array<double, 5> nodes{0, -0.5384693101056831, 0.5384693101056831, -0.9061798459386640,
                                          0.9061798459386640};
    constexpr std::array<double, 5> weights{0.5688888888888889, 0.4786286704993665, 0.4786286704993665,
                                            0.2369268850561891, 0.2369268850561891};
    const double mid = (a + z) / 2, half = (z - a) / 2;
    double result = 0;
    for (std::size_t i = 0; i < nodes.size(); ++i)
        result += weights[i] * norm(b.derivative(mid + half * nodes[i]));
    return half * result;
}
double integrate(const Bezier &b, double a, double z, double whole, int depth) {
    const double mid = (a + z) / 2;
    const double left = gauss(b, a, mid), right = gauss(b, mid, z);
    if (depth == 0 || std::abs(left + right - whole) < 1e-11 * (1 + std::abs(whole)))
        return left + right;
    return integrate(b, a, mid, left, depth - 1) + integrate(b, mid, z, right, depth - 1);
}
std::vector<double> quadraticRoots(double a, double b, double c) {
    const double scale = std::max({std::abs(a), std::abs(b), std::abs(c), 1e-30});
    a /= scale;
    b /= scale;
    c /= scale;
    if (std::abs(a) < 1e-14) {
        if (std::abs(b) < 1e-14)
            return {};
        return {-c / b};
    }
    const double discriminant = b * b - 4 * a * c;
    if (discriminant < 0)
        return {};
    const double q = -0.5 * (b + std::copysign(std::sqrt(discriminant), b));
    if (std::abs(q) < 1e-30)
        return {-b / (2 * a)};
    return {q / a, c / q};
}
std::vector<double> stationarySpeed(const Bezier &b) {
    // Speed extrema: B' dot B'' = 0.
    const Vec2 A = (b.p3 - b.p2 * 3 + b.p1 * 3 - b.p0) * 3;
    const Vec2 B = (b.p2 - b.p1 * 2 + b.p0) * 6;
    const Vec2 C = (b.p1 - b.p0) * 3;
    const double a = 2 * dot(A, A), d = dot(B, C);
    const double bb = 3 * dot(A, B), c = dot(B, B) + 2 * dot(A, C);
    auto polynomial = [&](double t) { return ((a * t + bb) * t + c) * t + d; };
    std::vector<double> cuts{0, 1}, roots{0, 1};
    for (double t : quadraticRoots(3 * a, 2 * bb, c))
        if (t > 0 && t < 1)
            cuts.push_back(t);
    std::sort(cuts.begin(), cuts.end());
    for (double t : cuts)
        roots.push_back(t);
    for (std::size_t i = 1; i < cuts.size(); ++i) {
        double lo = cuts[i - 1], hi = cuts[i];
        if (polynomial(lo) * polynomial(hi) >= 0)
            continue;
        const bool positive = polynomial(lo) > 0;
        for (int j = 0; j < 60; ++j) {
            const double mid = (lo + hi) / 2;
            if ((polynomial(mid) > 0) == positive)
                lo = mid;
            else
                hi = mid;
        }
        roots.push_back((lo + hi) / 2);
    }
    return roots;
}
} // namespace

double Bezier::arcLength(double begin, double end) const {
    return integrate(*this, begin, end, gauss(*this, begin, end), 12);
}

Track::Track(const std::vector<Vec2> &anchors) {
    if (anchors.size() < 2 || anchors.size() > 128)
        throw std::invalid_argument("A track needs 2 to 128 confirmed points.");
    for (Vec2 p : anchors)
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || std::abs(p.x) > 10000 || std::abs(p.y) > 10000)
            throw std::invalid_argument("Coordinates must be finite and within -10,000 to 10,000 m.");
    const std::size_t n = anchors.size();
    std::vector<double> h(n - 1), diagonal(n, 1), upper(n, 0);
    std::vector<Vec2> rhs(n), second(n);
    for (std::size_t i = 0; i + 1 < n; ++i) {
        h[i] = norm(anchors[i + 1] - anchors[i]);
        if (h[i] < 0.05)
            throw std::invalid_argument("Adjacent points must be at least 0.05 m apart.");
    }
    // Tridiagonal solve for the spline.
    for (std::size_t i = 1; i + 1 < n; ++i) {
        const double factor = h[i - 1] / diagonal[i - 1];
        diagonal[i] = 2 * (h[i - 1] + h[i]) - factor * upper[i - 1];
        upper[i] = h[i];
        rhs[i] = ((anchors[i + 1] - anchors[i]) / h[i] - (anchors[i] - anchors[i - 1]) / h[i - 1]) * 6 -
                 rhs[i - 1] * factor;
    }
    for (std::size_t i = n - 1; i-- > 0;)
        second[i] = (rhs[i] - second[i + 1] * upper[i]) / diagonal[i];
    for (std::size_t i = 0; i + 1 < n; ++i) {
        const Vec2 slope = (anchors[i + 1] - anchors[i]) / h[i];
        const Vec2 start = slope - (second[i] * 2 + second[i + 1]) * (h[i] / 6);
        const Vec2 end = slope + (second[i] + second[i + 1] * 2) * (h[i] / 6);
        Bezier b{anchors[i], anchors[i] + start * (h[i] / 3), anchors[i + 1] - end * (h[i] / 3),
                 anchors[i + 1]};
        const double scale = norm(b.p1 - b.p0) + norm(b.p2 - b.p1) + norm(b.p3 - b.p2);
        for (double u : stationarySpeed(b))
            if (norm(b.derivative(u)) < 1e-8 * std::max(1.0, scale))
                throw std::invalid_argument(
                    "These points create a cusp or reversal. Move a point to make a smooth turn.");
        segments_.push_back(b);
        constexpr int bins = 64;
        std::vector<double> table{0};
        for (int j = 1; j <= bins; ++j)
            table.push_back(table.back() + b.arcLength(double(j - 1) / bins, double(j) / bins));
        arcTables_.push_back(table);
        const double a = 3 * (-b.p0.y + 3 * b.p1.y - 3 * b.p2.y + b.p3.y);
        const double bb = 6 * (b.p0.y - 2 * b.p1.y + b.p2.y);
        const double c = 3 * (b.p1.y - b.p0.y);
        for (double u : quadraticRoots(a, bb, c)) {
            if (u > 0 && u < 1) {
                const auto j = static_cast<std::size_t>(u * bins);
                criticalDistances_.push_back(offsets_.back() + table[j] + b.arcLength(double(j) / bins, u));
            }
        }
        offsets_.push_back(offsets_.back() + table.back());
        criticalDistances_.push_back(offsets_.back());
    }
    if (length() > 100000)
        throw std::invalid_argument("Track length must not exceed 100 km.");
    std::sort(criticalDistances_.begin(), criticalDistances_.end());
}

std::vector<double> Track::tangentLevelDistances(std::size_t segment, double tangentY) const {
    std::vector<double> distances;
    if (!std::isfinite(tangentY) || std::abs(tangentY) >= 1)
        return distances;
    const auto &b = segments_.at(segment);
    const auto &table = arcTables_[segment];
    const Vec2 a = (b.p3 - b.p2 * 3 + b.p1 * 3 - b.p0) * 3;
    const Vec2 bb = (b.p2 - b.p1 * 2 + b.p0) * 6;
    const Vec2 c = (b.p1 - b.p0) * 3;
    const double tangentX = std::sqrt(1 - tangentY * tangentY);
    for (double x : {-tangentX, tangentX}) {
        // B'(u) parallel to (x, tangentY) is a quadratic equation.
        for (double u :
             quadraticRoots(a.y * x - a.x * tangentY, bb.y * x - bb.x * tangentY, c.y * x - c.x * tangentY)) {
            if (u > 0 && u < 1 && dot(b.derivative(u), {x, tangentY}) > 0) {
                const double bins = double(table.size() - 1);
                const auto cell = static_cast<std::size_t>(u * bins);
                distances.push_back(offsets_[segment] + table[cell] + b.arcLength(double(cell) / bins, u));
            }
        }
    }
    return distances;
}

// u is not meters. Convert the distance here.
TrackSample Track::sample(double distance) const {
    if (!std::isfinite(distance))
        throw std::invalid_argument("Distance must be finite.");
    distance = std::clamp(distance, 0.0, length());
    const auto upper = std::upper_bound(offsets_.begin(), offsets_.end(), distance);
    const auto segment =
        std::min(static_cast<std::size_t>(upper - offsets_.begin() - 1), segments_.size() - 1);
    const auto &table = arcTables_[segment];
    const auto &b = segments_[segment];
    const double local = distance - offsets_[segment];
    auto cell =
        static_cast<std::size_t>(std::upper_bound(table.begin(), table.end(), local) - table.begin() - 1);
    cell = std::min(cell, table.size() - 2);
    const double bins = double(table.size() - 1), begin = double(cell) / bins;
    double lo = begin, hi = double(cell + 1) / bins;
    double u = lo + (hi - lo) * (local - table[cell]) / (table[cell + 1] - table[cell]);
    for (int i = 0; i < 12; ++i) {
        const double error = table[cell] + b.arcLength(begin, u) - local;
        if (std::abs(error) < 1e-10)
            break;
        if (error > 0)
            hi = u;
        else
            lo = u;
        const double next = u - error / norm(b.derivative(u));
        u = next > lo && next < hi ? next : (lo + hi) / 2;
    }
    const Vec2 d = b.derivative(u), dd = b.secondDerivative(u);
    const double speed = norm(d);
    return {b.position(u), d / speed, (d.x * dd.y - d.y * dd.x) / (speed * speed * speed), segment, u};
}
} // namespace rail

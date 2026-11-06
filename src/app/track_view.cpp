#include "track_view.hpp"

#include <QLineF>
#include <QMouseEvent>
#include <QPainterPath>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>

namespace {
double tickStep(double span) {
    const double base = std::pow(10.0, std::floor(std::log10(std::max(span / 9, 1e-8))));
    const double ratio = span / 9 / base;
    return base * (ratio <= 1 ? 1 : ratio <= 2 ? 2 : ratio <= 5 ? 5 : 10);
}
} // namespace
TrackView::TrackView(QWidget *parent) : QWidget(parent) {
    setMinimumSize(500, 280);
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
    setToolTip("Type X/Y to preview a point, or click the plot. Click an anchor to edit it. Wheel: zoom. "
               "Right drag: pan.");
}
QRectF TrackView::graphArea(const QRectF &bounds) const {
    return bounds.adjusted(62, 46, -30, -43);
}
double TrackView::scale(const QRectF &bounds) const {
    const auto r = graphArea(bounds);
    return std::min(r.width() / worldWidth_, r.height() / worldHeight_);
}
QPointF TrackView::toScreen(rail::Vec2 point, const QRectF &bounds) const {
    const auto r = graphArea(bounds);
    const double k = scale(bounds);
    return {r.center().x() + (point.x - center_.x) * k, r.center().y() - (point.y - center_.y) * k};
}
rail::Vec2 TrackView::toWorld(QPointF point) const {
    const auto r = graphArea(rect());
    const double k = scale(rect());
    return {center_.x + (point.x() - r.center().x()) / k, center_.y - (point.y() - r.center().y()) / k};
}
void TrackView::setTrack(const std::vector<rail::Vec2> &points, const rail::Track *track,
                         const rail::Analysis *analysis) {
    points_ = points;
    curves_.clear();
    reached_.clear();
    hoverPoints_.clear();
    stop_.reset();
    probe_.reset();
    if (track) {
        curves_ = track->segments();
        for (int i = 0; i <= 600; ++i) {
            const double s = track->length() * i / 600;
            hoverPoints_.push_back({s, track->sample(s).position});
        }
    }
    if (analysis && !analysis->samples.empty()) {
        const std::size_t stride = std::max<std::size_t>(1, analysis->samples.size() / 2500);
        for (std::size_t i = 0; i < analysis->samples.size(); i += stride)
            reached_.push_back(analysis->samples[i].track.position);
        reached_.push_back(analysis->samples.back().track.position);
        if (analysis->status != rail::RunStatus::Complete)
            stop_ = analysis->samples.back().track.position;
    }
    update();
}
void TrackView::setPreview(const std::vector<rail::Vec2> &points, const rail::Track *track,
                           std::optional<rail::Vec2> candidate) {
    previewPoints_ = points;
    previewCurves_ = track ? track->segments() : std::vector<rail::Bezier>{};
    candidate_ = candidate;
    if (candidate_ && !graphArea(rect()).adjusted(15, 15, -15, -15).contains(toScreen(*candidate_, rect())))
        fit();
    update();
}
void TrackView::setProbe(std::optional<rail::Vec2> point) {
    probe_ = point;
    update();
}
void TrackView::setHandles(bool enabled) {
    handles_ = enabled;
    update();
}
void TrackView::fit() {
    double xmin = 0, xmax = 100, ymin = 0, ymax = 40;
    bool first = true;
    auto include = [&](rail::Vec2 p) {
        if (first) {
            xmin = xmax = p.x;
            ymin = ymax = p.y;
            first = false;
        }
        xmin = std::min(xmin, p.x);
        xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y);
        ymax = std::max(ymax, p.y);
    };
    for (auto p : points_)
        include(p);
    for (const auto &list : {curves_, previewCurves_})
        for (const auto &b : list)
            for (int j = 0; j <= 40; ++j)
                include(b.position(j / 40.0));
    if (candidate_)
        include(*candidate_);
    center_ = {(xmin + xmax) / 2, (ymin + ymax) / 2};
    worldWidth_ = std::max(20.0, (xmax - xmin) * 1.18);
    worldHeight_ = std::max(12.0, (ymax - ymin) * 1.45);
    update();
}
void TrackView::paintEvent(QPaintEvent *) {
    QPainter p(this);
    draw(p, rect());
}
void TrackView::draw(QPainter &p, const QRectF &bounds, bool report) const {
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(QPen(QColor("#aaaaaa"), 1));
    p.setBrush(QColor("#ffffff"));
    p.drawRect(bounds.adjusted(0.5, 0.5, -0.5, -0.5));
    QFont font("Arial");
    font.setPixelSize(13);
    font.setBold(true);
    p.setFont(font);
    p.setPen(QColor("#222222"));
    p.drawText(bounds.adjusted(18, 14, -18, 0), Qt::AlignLeft | Qt::AlignTop, "Track (meters)");
    font.setPixelSize(11);
    font.setBold(false);
    p.setFont(font);
    p.setPen(QColor("#555555"));
    p.drawText(bounds.adjusted(18, 16, -18, 0), Qt::AlignRight | Qt::AlignTop,
               report ? "Cubic Bezier interpolation  /  meters"
                      : "Wheel to zoom   ·   Right-drag to pan   ·   Click to preview");
    const QRectF graph = graphArea(bounds);
    const double k = scale(bounds);
    const double minX = center_.x - graph.width() / (2 * k), maxX = center_.x + graph.width() / (2 * k);
    const double minY = center_.y - graph.height() / (2 * k), maxY = center_.y + graph.height() / (2 * k);
    const double stepX = tickStep(maxX - minX), stepY = tickStep(maxY - minY);
    for (double x = std::ceil(minX / stepX) * stepX; x <= maxX; x += stepX) {
        const double sx = toScreen({x, 0}, bounds).x();
        p.setPen(QPen(QColor("#dddddd"), 0.8, Qt::DotLine));
        p.drawLine(QPointF(sx, graph.top()), QPointF(sx, graph.bottom()));
        p.setPen(QColor("#555555"));
        p.drawText(QRectF(sx - 27, graph.bottom() + 5, 54, 16), Qt::AlignHCenter, QString::number(x, 'g', 4));
    }
    for (double y = std::ceil(minY / stepY) * stepY; y <= maxY; y += stepY) {
        const double sy = toScreen({0, y}, bounds).y();
        p.setPen(QPen(QColor("#dddddd"), 0.8, Qt::DotLine));
        p.drawLine(QPointF(graph.left(), sy), QPointF(graph.right(), sy));
        p.setPen(QColor("#555555"));
        p.drawText(QRectF(bounds.left() + 8, sy - 7, 45, 14), Qt::AlignRight, QString::number(y, 'g', 4));
    }
    p.drawText(QRectF(graph.left(), bounds.bottom() - 19, graph.width(), 14), Qt::AlignHCenter,
               "Horizontal position · x (m)");
    p.save();
    p.translate(bounds.left() + 12, graph.center().y());
    p.rotate(-90);
    p.drawText(QRectF(-80, -8, 160, 14), Qt::AlignHCenter, "Height · y (m)");
    p.restore();
    p.save();
    p.setClipRect(graph);
    if (minY <= 0 && maxY >= 0) {
        p.setPen(QPen(QColor("#999999"), 1));
        p.drawLine(toScreen({minX, 0}, bounds), toScreen({maxX, 0}, bounds));
    }
    auto drawCurves = [&](const auto &curves, QColor color, double width, Qt::PenStyle style) {
        QPainterPath path;
        for (const auto &b : curves) {
            if (path.isEmpty())
                path.moveTo(toScreen(b.p0, bounds));
            path.cubicTo(toScreen(b.p1, bounds), toScreen(b.p2, bounds), toScreen(b.p3, bounds));
        }
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(color, width, style, Qt::RoundCap, Qt::RoundJoin));
        p.drawPath(path);
    };
    if (handles_)
        for (const auto &b : curves_) {
            p.setPen(QPen(QColor("#999999"), 1, Qt::DashLine));
            p.drawLine(toScreen(b.p0, bounds), toScreen(b.p1, bounds));
            p.drawLine(toScreen(b.p2, bounds), toScreen(b.p3, bounds));
            p.setBrush(QColor("#999999"));
            p.drawEllipse(toScreen(b.p1, bounds), 2.5, 2.5);
            p.drawEllipse(toScreen(b.p2, bounds), 2.5, 2.5);
        }

    drawCurves(curves_, QColor(stop_ ? "#999999" : "#245ca6"), 2.8, Qt::SolidLine);
    if (stop_ && reached_.size() > 1) {
        QPainterPath reached;
        reached.moveTo(toScreen(reached_.front(), bounds));
        for (const auto &point : reached_)
            reached.lineTo(toScreen(point, bounds));
        p.setPen(QPen(QColor("#245ca6"), 3));
        p.drawPath(reached);
    }
    if (!report)
        drawCurves(previewCurves_, QColor("#b75c00"), 2, Qt::DashLine);
    for (std::size_t i = 0; i < points_.size(); ++i) {
        const QPointF screen = toScreen(points_[i], bounds);
        p.setPen(QPen(QColor("#245ca6"), 1.5));
        p.setBrush(QColor("#ffffff"));
        p.drawEllipse(screen, 4.5, 4.5);
        p.setPen(QColor("#333333"));
        p.drawText(screen + QPointF(7, -9), QString::number(i + 1));
    }
    if (candidate_ && !report) {
        const auto pos = toScreen(*candidate_, bounds);
        p.setBrush(QColor("#b75c00"));
        p.setPen(QPen(QColor("#b75c00"), 1.5));
        p.drawRect(QRectF(pos.x() - 5, pos.y() - 5, 10, 10));
        p.setBrush(Qt::NoBrush);
        p.drawEllipse(pos, 11, 11);
        p.drawText(pos + QPointF(14, 20), "PREVIEW");
    }
    if (stop_) {
        const auto pos = toScreen(*stop_, bounds);
        p.setPen(QPen(QColor("#b75c00"), 2));
        p.setBrush(QColor("#ffffff"));
        p.drawEllipse(pos, 7, 7);
        p.drawText(pos + QPointF(11, -12), "STOP");
    }
    if (probe_ && !report) {
        p.setPen(QPen(QColor("#ffffff"), 1.5));
        p.setBrush(QColor("#245ca6"));
        p.drawEllipse(toScreen(*probe_, bounds), 6, 6);
    }
    if (points_.empty() && !candidate_) {
        p.setPen(QColor("#9eafc6"));
        font.setPixelSize(16);
        p.setFont(font);
        p.drawText(graph, Qt::AlignCenter, "Enter X and Y to preview your first point");
    }
    p.restore();
    p.restore();
}
void TrackView::wheelEvent(QWheelEvent *e) {
    const auto before = toWorld(e->position());
    const double factor = std::pow(1.0015, -e->angleDelta().y());
    worldWidth_ = std::clamp(worldWidth_ * factor, 0.2, 80000.0);
    worldHeight_ = std::clamp(worldHeight_ * factor, 0.2, 80000.0);
    const auto after = toWorld(e->position());
    center_ = center_ + before - after;
    update();
    e->accept();
}
void TrackView::mousePressEvent(QMouseEvent *e) {
    if (e->button() == Qt::RightButton || e->button() == Qt::MiddleButton) {
        panning_ = true;
        lastMouse_ = e->position();
        setCursor(Qt::ClosedHandCursor);
        return;
    }
    if (e->button() != Qt::LeftButton || !graphArea(rect()).contains(e->position()))
        return;
    for (std::size_t i = 0; i < points_.size(); ++i) {
        if (QLineF(toScreen(points_[i], rect()), e->position()).length() < 10) {
            emit anchorSelected(static_cast<int>(i));
            return;
        }
    }
    const auto point = toWorld(e->position());
    emit coordinateProposed(point.x, point.y);
}
void TrackView::mouseMoveEvent(QMouseEvent *e) {
    if (panning_) {
        const auto delta = e->position() - lastMouse_;
        const double k = scale(rect());
        center_ = center_ + rail::Vec2{-delta.x() / k, delta.y() / k};
        lastMouse_ = e->position();
        update();
        return;
    }
    double best = 20, distance = -1;
    for (const auto &[s, point] : hoverPoints_) {
        const double d = QLineF(toScreen(point, rect()), e->position()).length();
        if (d < best) {
            best = d;
            distance = s;
        }
    }
    if (distance >= 0)
        emit distanceHovered(distance);
}
void TrackView::mouseReleaseEvent(QMouseEvent *) {
    panning_ = false;
    unsetCursor();
}

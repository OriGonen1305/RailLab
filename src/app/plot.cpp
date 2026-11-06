#include "plot.hpp"

#include <QMouseEvent>
#include <QPainterPath>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
QString number(double x) {
    if (std::abs(x) < 1e-10)
        return "0";
    if (std::abs(x) >= 10000 || std::abs(x) < 0.001)
        return QString::number(x, 'g', 2);
    return QString::number(x, 'g', 3);
}
QRectF area(QRectF r) {
    return r.adjusted(62, 64, -22, -45);
}
} // namespace

Plot::Plot(QWidget *parent) : QWidget(parent) {
    setMinimumSize(290, 210);
    setMouseTracking(true);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}
void Plot::setData(QString title, QString unit, QString xLabel, QVector<Series> series) {
    title_ = std::move(title);
    unit_ = std::move(unit);
    xLabel_ = std::move(xLabel);
    series_ = std::move(series);
    minX_ = minY_ = std::numeric_limits<double>::infinity();
    maxX_ = maxY_ = -std::numeric_limits<double>::infinity();
    for (const auto &line : series_)
        for (const auto &p : line.points) {
            if (!std::isfinite(p.x()) || !std::isfinite(p.y()))
                continue;
            minX_ = std::min(minX_, p.x());
            maxX_ = std::max(maxX_, p.x());
            minY_ = std::min(minY_, p.y());
            maxY_ = std::max(maxY_, p.y());
        }
    if (!std::isfinite(minX_)) {
        minX_ = minY_ = 0;
        maxX_ = maxY_ = 1;
    }
    if (maxX_ - minX_ < 1e-8)
        maxX_ = minX_ + 1;
    minY_ = std::min(0.0, minY_);
    maxY_ = std::max(0.0, maxY_);
    const double pad = std::max(0.01, (maxY_ - minY_) * 0.1);
    if (minY_ < 0)
        minY_ -= pad;
    maxY_ += pad;
    update();
}
void Plot::setCursor(std::optional<double> value) {
    cursor_ = value;
    update();
}
void Plot::paintEvent(QPaintEvent *) {
    QPainter p(this);
    draw(p, rect());
}
void Plot::draw(QPainter &p, const QRectF &bounds, bool cursor) const {
    p.save();
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(QPen(QColor("#aaaaaa"), 1));
    p.setBrush(QColor("#ffffff"));
    p.drawRect(bounds.adjusted(0.5, 0.5, -0.5, -0.5));
    QFont font("Arial");
    font.setPixelSize(14);
    font.setBold(true);
    p.setFont(font);
    p.setPen(QColor("#222222"));
    p.drawText(bounds.adjusted(17, 12, -17, 0), Qt::AlignLeft | Qt::AlignTop, title_);
    font.setPixelSize(10);
    font.setBold(false);
    p.setFont(font);
    p.setPen(QColor("#555555"));
    p.drawText(bounds.adjusted(17, 15, -17, 0), Qt::AlignRight | Qt::AlignTop, unit_);
    double legendX = bounds.left() + 17;
    for (const auto &line : series_) {
        p.setPen(QPen(line.color, 1.5));
        p.drawLine(QPointF(legendX, bounds.top() + 43), QPointF(legendX + 10, bounds.top() + 43));
        p.setPen(QColor("#444444"));
        p.drawText(QPointF(legendX + 15, bounds.top() + 47), line.name);
        legendX += 29 + p.fontMetrics().horizontalAdvance(line.name);
    }
    const QRectF graph = area(bounds);
    auto map = [&](QPointF point) {
        return QPointF(graph.left() + (point.x() - minX_) / (maxX_ - minX_) * graph.width(),
                       graph.bottom() - (point.y() - minY_) / (maxY_ - minY_) * graph.height());
    };
    for (int i = 0; i <= 4; ++i) {
        const double fraction = i / 4.0;
        const double x = graph.left() + fraction * graph.width();
        const double y = graph.bottom() - fraction * graph.height();
        p.setPen(QPen(QColor("#dddddd"), 0.8, Qt::DotLine));
        p.drawLine(QPointF(x, graph.top()), QPointF(x, graph.bottom()));
        p.drawLine(QPointF(graph.left(), y), QPointF(graph.right(), y));
        p.setPen(QColor("#555555"));
        p.drawText(QRectF(bounds.left() + 4, y - 7, 50, 14), Qt::AlignRight | Qt::AlignVCenter,
                   number(minY_ + fraction * (maxY_ - minY_)));
        p.drawText(QRectF(x - 35, graph.bottom() + 6, 70, 14), Qt::AlignHCenter,
                   number(minX_ + fraction * (maxX_ - minX_)));
    }
    p.drawText(QRectF(graph.left(), bounds.bottom() - 19, graph.width(), 14), Qt::AlignHCenter, xLabel_);
    p.save();
    p.setClipRect(graph.adjusted(-1, -1, 1, 1));
    for (const auto &line : series_) {
        QPainterPath path;
        bool first = true;
        for (const auto &point : line.points) {
            if (!std::isfinite(point.x()) || !std::isfinite(point.y())) {
                first = true;
                continue;
            }
            const QPointF screen = map(point);
            if (first)
                path.moveTo(screen);
            else
                path.lineTo(screen);
            first = false;
        }
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(line.color, 1.5));
        p.drawPath(path);
        if (line.points.size() == 1) {
            p.setBrush(line.color);
            p.drawEllipse(map(line.points.front()), 3, 3);
        }
    }
    if (cursor && cursor_ && *cursor_ >= minX_ && *cursor_ <= maxX_) {
        const double x = map({*cursor_, 0}).x();
        p.setPen(QPen(QColor("#777777"), 1, Qt::DashLine));
        p.drawLine(QPointF(x, graph.top()), QPointF(x, graph.bottom()));
    }
    p.restore();
    p.restore();
}
void Plot::mouseMoveEvent(QMouseEvent *event) {
    const auto graph = area(rect());
    if (graph.contains(event->position()))
        emit hovered(minX_ + (event->position().x() - graph.left()) / graph.width() * (maxX_ - minX_));
}

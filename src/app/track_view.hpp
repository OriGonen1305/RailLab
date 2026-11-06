#pragma once

#include "raillab/analysis.hpp"
#include <QPainter>
#include <QPointF>
#include <QWidget>
#include <optional>

class TrackView final : public QWidget {
    Q_OBJECT
  public:
    explicit TrackView(QWidget *parent = nullptr);
    void setTrack(const std::vector<rail::Vec2> &points, const rail::Track *track,
                  const rail::Analysis *analysis);
    void setPreview(const std::vector<rail::Vec2> &points, const rail::Track *track,
                    std::optional<rail::Vec2> candidate);
    void setProbe(std::optional<rail::Vec2> point);
    void setHandles(bool enabled);
    void fit();
    void draw(QPainter &painter, const QRectF &bounds, bool report = false) const;
  signals:
    void coordinateProposed(double x, double y);
    void anchorSelected(int index);
    void distanceHovered(double distance);

  protected:
    void paintEvent(QPaintEvent *) override;
    void wheelEvent(QWheelEvent *) override;
    void mousePressEvent(QMouseEvent *) override;
    void mouseMoveEvent(QMouseEvent *) override;
    void mouseReleaseEvent(QMouseEvent *) override;

  private:
    std::vector<rail::Vec2> points_, previewPoints_, reached_;
    std::vector<rail::Bezier> curves_, previewCurves_;
    std::vector<std::pair<double, rail::Vec2>> hoverPoints_;
    std::optional<rail::Vec2> candidate_, probe_, stop_;
    rail::Vec2 center_{50, 20};
    double worldWidth_ = 160, worldHeight_ = 70;
    bool handles_ = false, panning_ = false;
    QPointF lastMouse_;
    QRectF graphArea(const QRectF &bounds) const;
    double scale(const QRectF &bounds) const;
    QPointF toScreen(rail::Vec2 point, const QRectF &bounds) const;
    rail::Vec2 toWorld(QPointF point) const;
};

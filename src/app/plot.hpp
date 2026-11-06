#pragma once

#include <QColor>
#include <QPainter>
#include <QPointF>
#include <QString>
#include <QVector>
#include <QWidget>
#include <optional>

struct Series {
    QString name;
    QColor color;
    QVector<QPointF> points;
};

class Plot final : public QWidget {
    Q_OBJECT
  public:
    explicit Plot(QWidget *parent = nullptr);
    void setData(QString title, QString unit, QString xLabel, QVector<Series> series);
    void setCursor(std::optional<double> value);
    void draw(QPainter &painter, const QRectF &bounds, bool cursor = true) const;
  signals:
    void hovered(double value);

  protected:
    void paintEvent(QPaintEvent *) override;
    void mouseMoveEvent(QMouseEvent *) override;

  private:
    QString title_, unit_, xLabel_;
    QVector<Series> series_;
    std::optional<double> cursor_;
    double minX_ = 0, maxX_ = 1, minY_ = 0, maxY_ = 1;
};

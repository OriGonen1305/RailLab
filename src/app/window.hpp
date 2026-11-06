#pragma once

#include "plot.hpp"
#include "project.hpp"
#include "track_view.hpp"
#include <QMainWindow>
#include <array>
#include <exception>
#include <optional>

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QTabWidget;
class QTableView;
class SampleModel;

class Window final : public QMainWindow {
    Q_OBJECT
  public:
    explicit Window(QWidget *parent = nullptr);
    const Project &document() const { return project_; }
    const std::optional<rail::Analysis> &analysis() const { return analysis_; }
    void setDocument(Project project);
    void openFrom(const QString &path);
    void saveTo(const QString &path);
    void exportCsv(const QString &path) const;
    void exportSvg(const QString &path) const;

  protected:
    void closeEvent(QCloseEvent *event) override;

  private:
    Project project_;
    std::vector<Project> undo_, redo_;
    std::optional<rail::Track> track_;
    std::optional<rail::Analysis> analysis_;
    QString filePath_;
    bool dirty_ = false, loading_ = false, draft_ = false;
    int editIndex_ = -1;
    int insertIndex_ = -1;
    rail::Vec2 draftPoint_{};
    TrackView *trackView_{};
    QListWidget *points_{};
    QDoubleSpinBox *x_{}, *y_{};
    std::array<QDoubleSpinBox *, 7> physics_{};
    QLabel *draftHint_{}, *status_{}, *inspector_{}, *projectTitle_{}, *pointCount_{};
    QLabel *forceHint_{};
    QLineEdit *forwardForce_{};
    std::array<QLabel *, 4> metrics_{};
    QPushButton *confirm_{}, *cancel_{}, *undoButton_{}, *redoButton_{}, *remove_{};
    QPushButton *applyForce_{}, *insert_{};
    QComboBox *presets_{}, *axis_{}, *segments_{};
    QTabWidget *tabs_{};
    std::array<Plot *, 9> plots_{};
    SampleModel *sampleModel_{};
    void buildUi();
    void refresh(bool fit = false);
    void updatePreview();
    void startNewPoint();
    void startInsertPoint();
    void selectPoint(int index);
    void clearDraft();
    void commitPoint();
    void removePoint();
    void movePoint(int direction);
    void pushUndo();
    void history(bool redo);
    void physicsChanged(int settingIndex);
    void refreshPreservingDraft();
    void selectSegment();
    void updateForceInput();
    void applyForce();
    void updatePlots();
    void inspect(double value, bool byDistance = false);
    bool confirmDiscard();
    bool saveDialog();
    void showError(const std::exception &error);
};

#include "window.hpp"

#include <QAbstractTableModel>
#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenu>
#include <QMessageBox>
#include <QPushButton>
#include <QSaveFile>
#include <QScrollArea>
#include <QShortcut>
#include <QSignalBlocker>
#include <QSplitter>
#include <QStatusBar>
#include <QSvgGenerator>
#include <QTabWidget>
#include <QTableView>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace {
const QColor teal("#245ca6"), blue("#287a38"), amber("#b75c00"), pink("#a52c37"), muted("#555555");
QLabel *label(const QString &text, const char *objectName, QWidget *parent = nullptr) {
    auto *result = new QLabel(text, parent);
    result->setObjectName(objectName);
    return result;
}
QPushButton *button(const QString &text, const char *objectName) {
    auto *result = new QPushButton(text);
    result->setObjectName(objectName);
    result->setCursor(Qt::PointingHandCursor);
    return result;
}
QDoubleSpinBox *numberBox(const char *name, double low, double high, int digits, double step = 1) {
    auto *b = new QDoubleSpinBox;
    b->setObjectName(name);
    b->setRange(low, high);
    b->setDecimals(digits);
    b->setSingleStep(step);
    b->setKeyboardTracking(false);
    b->setMinimumHeight(32);
    b->setLocale(QLocale::c());
    return b;
}
} // namespace

class SampleModel final : public QAbstractTableModel {
  public:
    explicit SampleModel(QObject *parent) : QAbstractTableModel(parent) {}
    void setAnalysis(const rail::Analysis *value) {
        beginResetModel();
        analysis_ = value;
        endResetModel();
    }
    int rowCount(const QModelIndex &parent = {}) const override {
        return !parent.isValid() && analysis_ ? static_cast<int>(analysis_->samples.size()) : 0;
    }
    int columnCount(const QModelIndex &parent = {}) const override { return parent.isValid() ? 0 : 15; }
    QVariant headerData(int section, Qt::Orientation orientation, int role) const override {
        if (role != Qt::DisplayRole)
            return {};
        if (orientation == Qt::Vertical)
            return section + 1;
        const QStringList names{"s (m)",      "t (s)",      "x (m)",    "y (m)",       "v (m/s)",
                                "a_t (m/s²)", "a_n (m/s²)", "Normal G", "Felt G",      "Rail (N)",
                                "K (J)",      "U (J)",      "Loss (J)", "Applied (N)", "Applied work (J)"};
        return names.value(section);
    }
    QVariant data(const QModelIndex &index, int role) const override {
        if (!analysis_ || !index.isValid())
            return {};
        if (role == Qt::TextAlignmentRole)
            return int(Qt::AlignRight | Qt::AlignVCenter);
        if (role != Qt::DisplayRole)
            return {};
        const auto &p = analysis_->samples.at(static_cast<std::size_t>(index.row()));
        const double values[]{p.distance,           p.time,          p.track.position.x,
                              p.track.position.y,   p.speed,         p.tangentialAcceleration,
                              p.normalAcceleration, p.normalG,       p.feltG,
                              p.railForce,          p.kineticEnergy, p.potentialEnergy,
                              p.dissipatedEnergy,   p.forwardForce,  p.propulsionWork};
        return QString::number(values[index.column()], 'f', 3);
    }

  private:
    const rail::Analysis *analysis_ = nullptr;
};

Window::Window(QWidget *parent) : QMainWindow(parent) {
    setWindowTitle("RailLab — Bezier track analysis");
    resize(1450, 1040);
    setMinimumSize(1100, 850);
    QPalette colors;
    colors.setColor(QPalette::Window, QColor("#eeeeee"));
    colors.setColor(QPalette::WindowText, Qt::black);
    colors.setColor(QPalette::Base, Qt::white);
    colors.setColor(QPalette::AlternateBase, QColor("#f2f2f2"));
    colors.setColor(QPalette::Text, Qt::black);
    colors.setColor(QPalette::Button, QColor("#e5e5e5"));
    colors.setColor(QPalette::ButtonText, Qt::black);
    colors.setColor(QPalette::Highlight, QColor("#b8d2eb"));
    colors.setColor(QPalette::HighlightedText, Qt::black);
    colors.setColor(QPalette::Disabled, QPalette::ButtonText, QColor("#888888"));
    qApp->setPalette(colors);
    setStyleSheet(R"(
        QWidget { font-size: 12px; }
        QLabel#brand { font-size: 18px; font-weight: bold; }
        QLabel#projectTitle { font-size: 14px; }
        QLabel#metricValue { font-size: 18px; }
        QLabel#metricLabel, QLabel#section { font-size: 11px; }
        QFrame#sidebar { border: 1px solid #bbbbbb; }
        QPushButton { padding: 5px 8px; }
        QListWidget, QTableView, QDoubleSpinBox, QComboBox { background: white; color: black; }
    )");
    buildUi();
    setDocument(preset(0));
}

void Window::buildUi() {
    auto *root = new QWidget;
    setCentralWidget(root);
    auto *layout = new QVBoxLayout(root);
    layout->setContentsMargins(20, 14, 20, 10);
    layout->setSpacing(12);
    auto *header = new QHBoxLayout;
    header->addWidget(label("Track simulator", "brand"));
    auto *subtitle = label("2D Bezier track", "eyebrow");
    subtitle->setStyleSheet("margin-left: 12px;");
    header->addWidget(subtitle);
    header->addStretch();
    auto *newProject = button("New", "newProject");
    auto *open = button("Open", "openProject");
    auto *save = button("Save project", "saveProject");
    auto *exports = button("Export", "exportMenu");
    for (auto *b : {newProject, open, save, exports})
        header->addWidget(b);
    auto *menu = new QMenu(exports);
    auto *csvAction = menu->addAction("Analysis data (.csv)");
    auto *svgAction = menu->addAction("All graphs (.svg)");
    exports->setMenu(menu);
    layout->addLayout(header);

    auto *body = new QHBoxLayout;
    body->setSpacing(16);
    layout->addLayout(body, 1);
    auto *sidebarScroll = new QScrollArea;
    sidebarScroll->setWidgetResizable(true);
    sidebarScroll->setFixedWidth(284);
    sidebarScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    auto *sidebar = new QFrame;
    sidebar->setObjectName("sidebar");
    sidebarScroll->setWidget(sidebar);
    body->addWidget(sidebarScroll);
    auto *side = new QVBoxLayout(sidebar);
    side->setContentsMargins(16, 18, 16, 18);
    side->setSpacing(10);
    side->addWidget(label("Examples", "section"));
    presets_ = new QComboBox;
    presets_->setObjectName("presets");
    presets_->addItems({"Alpine descent", "Vertical loop", "Energy barrier"});
    side->addWidget(presets_);
    auto *loadPreset = button("Load example", "loadPreset");
    side->addWidget(loadPreset);
    side->addSpacing(8);
    side->addWidget(label("Segment force", "section"));
    segments_ = new QComboBox;
    segments_->setObjectName("segmentSelector");
    segments_->setAccessibleName("Track segment endpoints");
    side->addWidget(segments_);
    auto *forceLabel = label("Applied force (N)", "forwardForceLabel");
    side->addWidget(forceLabel);
    auto *forceRow = new QHBoxLayout;
    forwardForce_ = new QLineEdit;
    forwardForce_->setObjectName("forwardForce");
    forwardForce_->setAccessibleName("Applied force (N)");
    forwardForce_->setMinimumHeight(32);
    forwardForce_->setToolTip(
        "Positive = propulsion along the local forward tangent; negative = braking "
        "opposite travel; 0 = off. Use a finite number; scientific notation is accepted. "
        "The cart stops without reversing.");
    forceLabel->setBuddy(forwardForce_);
    applyForce_ = button("Apply force", "applyForce");
    forceRow->addWidget(forwardForce_, 1);
    forceRow->addWidget(applyForce_);
    side->addLayout(forceRow);
    forceHint_ = label("", "forceHint");
    forceHint_->setWordWrap(true);
    side->addWidget(forceHint_);
    connect(segments_, &QComboBox::currentIndexChanged, this, &Window::selectSegment);
    connect(forwardForce_, &QLineEdit::textChanged, this, &Window::updateForceInput);
    connect(applyForce_, &QPushButton::clicked, this, &Window::applyForce);
    side->addSpacing(10);
    side->addWidget(label("Add or edit a point", "section"));
    auto *intro = label("Enter coordinates, inspect the amber preview, then confirm the point.", "muted");
    intro->setWordWrap(true);
    side->addWidget(intro);
    auto *coordinates = new QGridLayout;
    coordinates->setHorizontalSpacing(10);
    x_ = numberBox("pointX", -10000, 10000, 2);
    y_ = numberBox("pointY", -10000, 10000, 2);
    // Typing only previews it. Click to keep it.
    x_->setKeyboardTracking(true);
    y_->setKeyboardTracking(true);
    coordinates->addWidget(label("X · meters", "muted"), 0, 0);
    coordinates->addWidget(label("Y · meters", "muted"), 0, 1);
    coordinates->addWidget(x_, 1, 0);
    coordinates->addWidget(y_, 1, 1);
    side->addLayout(coordinates);
    confirm_ = button("Add point", "confirmPoint");
    side->addWidget(confirm_);
    auto *draftButtons = new QHBoxLayout;
    auto *addNew = button("New point", "newPoint");
    cancel_ = button("Cancel", "cancelPreview");
    draftButtons->addWidget(addNew);
    draftButtons->addWidget(cancel_);
    side->addLayout(draftButtons);
    draftHint_ = label("No unconfirmed changes.", "draftHint");
    draftHint_->setWordWrap(true);
    draftHint_->setMinimumHeight(32);
    side->addWidget(draftHint_);
    auto *pointHeader = new QHBoxLayout;
    pointHeader->addWidget(label("Points", "section"));
    pointCount_ = label("0 / 128", "muted");
    pointHeader->addStretch();
    pointHeader->addWidget(pointCount_);
    side->addLayout(pointHeader);
    points_ = new QListWidget;
    points_->setObjectName("pointsList");
    points_->setFixedHeight(163);
    side->addWidget(points_);
    auto *row = new QHBoxLayout;
    auto *up = button("Up", "moveUp");
    auto *down = button("Down", "moveDown");
    remove_ = button("Remove", "removePoint");
    row->addWidget(up);
    row->addWidget(down);
    row->addWidget(remove_);
    side->addLayout(row);
    insert_ = button("Insert after selected point", "insertPoint");
    insert_->setToolTip("Split the next segment; both halves inherit its applied force.");
    side->addWidget(insert_);
    auto *historyRow = new QHBoxLayout;
    undoButton_ = button("Undo", "undo");
    redoButton_ = button("Redo", "redo");
    historyRow->addWidget(undoButton_);
    historyRow->addWidget(redoButton_);
    side->addLayout(historyRow);
    side->addSpacing(8);
    side->addWidget(label("Physics", "section"));
    auto *form = new QFormLayout;
    form->setSpacing(7);
    physics_ = {numberBox("initialSpeed", 0, 200, 2, 0.5),  numberBox("mass", 1, 100000, 1, 10),
                numberBox("gravity", 0.1, 30, 5, 0.1),      numberBox("resistance", 0, 10, 3, 0.01),
                numberBox("dragArea", 0, 20, 3, 0.05),      numberBox("airDensity", 0, 20, 3, 0.05),
                numberBox("distanceStep", 0.01, 2, 2, 0.05)};
    const QStringList names{"Start speed · m/s", "Cart mass · kg",      "Gravity · m/s²", "Resistance · m/s²",
                            "Drag CdA · m²",     "Air density · kg/m³", "Step · m"};
    const QStringList tips{
        "Forward initial speed. Zero needs positive net acceleration from gravity or propulsion.",
        "Point mass constrained to the track.",
        "Gravitational acceleration, downward.",
        "Constant opposing acceleration; an approximate energy loss model, not wheel-contact friction.",
        "Drag coefficient times frontal area. Drag = 0.5 × density × CdA × speed².",
        "Air density used in quadratic aerodynamic drag.",
        "Maximum distance integration step. Smaller values improve numerical resolution."};
    for (int i = 0; i < 7; ++i) {
        form->addRow(names[i], physics_[i]);
        physics_[i]->setToolTip(tips[i]);
        connect(physics_[i], &QDoubleSpinBox::valueChanged, this, [this, i] { physicsChanged(i); });
    }
    side->addLayout(form);
    auto *modelNote =
        label("Held-to-rail point mass · one forward pass\nSI units · static analysis", "muted");
    modelNote->setWordWrap(true);
    side->addWidget(modelNote);
    side->addStretch();

    auto *right = new QVBoxLayout;
    right->setSpacing(10);
    body->addLayout(right, 1);
    auto *titleRow = new QHBoxLayout;
    projectTitle_ = label("", "projectTitle");
    titleRow->addWidget(projectTitle_);
    titleRow->addStretch();
    auto *controls = new QCheckBox("Bezier handles");
    controls->setObjectName("showHandles");
    titleRow->addWidget(controls);
    auto *fit = button("Fit view", "fitView");
    titleRow->addWidget(fit);
    right->addLayout(titleRow);
    auto *stats = new QHBoxLayout;
    stats->setSpacing(10);
    const QStringList metricNames{"Length", "Max speed", "Max felt load", "Time estimate"};
    for (int i = 0; i < 4; ++i) {
        auto *frame = new QFrame;
        frame->setObjectName("metric");
        auto *inner = new QVBoxLayout(frame);
        inner->setContentsMargins(14, 10, 14, 10);
        inner->setSpacing(3);
        inner->addWidget(label(metricNames[i], "metricLabel"));
        metrics_[i] = label("—", "metricValue");
        inner->addWidget(metrics_[i]);
        stats->addWidget(frame);
    }
    right->addLayout(stats);
    auto *split = new QSplitter(Qt::Vertical);
    split->setChildrenCollapsible(false);
    split->setHandleWidth(8);
    trackView_ = new TrackView;
    trackView_->setObjectName("trackView");
    split->addWidget(trackView_);
    auto *graphPanel = new QWidget;
    auto *graphLayout = new QVBoxLayout(graphPanel);
    graphLayout->setContentsMargins(0, 0, 0, 0);
    graphLayout->setSpacing(6);
    status_ = label("", "analysisStatus");
    status_->setWordWrap(true);
    status_->setMinimumHeight(29);
    graphLayout->addWidget(status_);
    auto *graphTop = new QHBoxLayout;
    graphTop->addWidget(label("Graphs", "section"));
    graphTop->addStretch();
    graphTop->addWidget(label("Horizontal axis", "muted"));
    axis_ = new QComboBox;
    axis_->setObjectName("horizontalAxis");
    axis_->addItems({"Distance (m)", "Elapsed time (s)"});
    graphTop->addWidget(axis_);
    graphLayout->addLayout(graphTop);
    tabs_ = new QTabWidget;
    tabs_->setObjectName("analysisTabs");
    graphLayout->addWidget(tabs_, 1);
    for (auto &plot : plots_) {
        plot = new Plot;
        connect(plot, &Plot::hovered, this, [this](double value) { inspect(value); });
    }
    const auto page = [&](QString name, std::initializer_list<int> indices) {
        auto *w = new QWidget;
        auto *grid = new QGridLayout(w);
        grid->setContentsMargins(0, 7, 0, 0);
        grid->setSpacing(10);
        int i = 0;
        for (int index : indices) {
            grid->addWidget(plots_[index], i / 2, i % 2);
            ++i;
        }
        const int rows = (i + 1) / 2;
        w->setMinimumHeight(rows * 210 + (rows - 1) * 10 + 7);
        auto *scroll = new QScrollArea;
        scroll->setWidgetResizable(true);
        scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        scroll->setWidget(w);
        tabs_->addTab(scroll, name);
    };
    page("Overview", {0, 1, 2, 3});
    page("Geometry / time", {4, 5, 7, 8});
    page("Rail force", {6});
    sampleModel_ = new SampleModel(this);
    auto *table = new QTableView;
    table->setObjectName("sampleTable");
    table->setModel(sampleModel_);
    table->setAlternatingRowColors(true);
    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
    table->horizontalHeader()->setDefaultSectionSize(107);
    table->verticalHeader()->setDefaultSectionSize(25);
    tabs_->addTab(table, "Sample data");
    split->addWidget(graphPanel);
    split->setSizes({300, 480});
    right->addWidget(split, 1);
    inspector_ = label("Hover over the track or any graph to inspect a sample.", "inspector");
    inspector_->setWordWrap(true);
    inspector_->setStyleSheet("color: #444444; font-size: 11px; padding: 5px 0;");
    right->addWidget(inspector_);

    for (auto *coordinate : {x_, y_}) {
        const auto editCoordinate = [this, coordinate] {
            if (!loading_) {
                if (!draft_)
                    draftPoint_ = {x_->value(), y_->value()};
                // The other coordinate may have more precision than its control displays.
                if (coordinate == x_)
                    draftPoint_.x = x_->value();
                else
                    draftPoint_.y = y_->value();
                draft_ = true;
                updatePreview();
            }
        };
        connect(coordinate, &QDoubleSpinBox::valueChanged, this, editCoordinate);
        connect(coordinate->findChild<QLineEdit *>(), &QLineEdit::textEdited, this, editCoordinate);
    }
    connect(confirm_, &QPushButton::clicked, this, &Window::commitPoint);
    connect(cancel_, &QPushButton::clicked, this, &Window::clearDraft);
    connect(addNew, &QPushButton::clicked, this, &Window::startNewPoint);
    connect(insert_, &QPushButton::clicked, this, &Window::startInsertPoint);
    connect(points_, &QListWidget::currentRowChanged, this, &Window::selectPoint);
    connect(remove_, &QPushButton::clicked, this, &Window::removePoint);
    connect(up, &QPushButton::clicked, this, [this] { movePoint(-1); });
    connect(down, &QPushButton::clicked, this, [this] { movePoint(1); });
    connect(undoButton_, &QPushButton::clicked, this, [this] { history(false); });
    connect(redoButton_, &QPushButton::clicked, this, [this] { history(true); });
    connect(fit, &QPushButton::clicked, trackView_, &TrackView::fit);
    connect(controls, &QCheckBox::toggled, trackView_, &TrackView::setHandles);
    connect(axis_, &QComboBox::currentIndexChanged, this, [this] { updatePlots(); });
    connect(trackView_, &TrackView::anchorSelected, this,
            [this](int index) { points_->setCurrentRow(index); });
    connect(trackView_, &TrackView::coordinateProposed, this, [this](double x, double y) {
        editIndex_ = -1;
        insertIndex_ = -1;
        {
            QSignalBlocker block(points_);
            points_->setCurrentRow(-1);
        }
        loading_ = true;
        x_->setValue(x);
        y_->setValue(y);
        loading_ = false;
        draftPoint_ = {x_->value(), y_->value()};
        draft_ = true;
        updatePreview();
    });
    connect(trackView_, &TrackView::distanceHovered, this, [this](double s) { inspect(s, true); });
    connect(newProject, &QPushButton::clicked, this, [this] {
        if (confirmDiscard())
            setDocument(preset(3));
    });
    connect(loadPreset, &QPushButton::clicked, this, [this] {
        if (confirmDiscard())
            setDocument(preset(presets_->currentIndex()));
    });
    connect(open, &QPushButton::clicked, this, [this] {
        if (!confirmDiscard())
            return;
        const auto path =
            QFileDialog::getOpenFileName(this, "Open RailLab project", {}, "RailLab project (*.json)");
        if (!path.isEmpty())
            try {
                openFrom(path);
            } catch (const std::exception &e) {
                showError(e);
            }
    });
    connect(save, &QPushButton::clicked, this, [this] { saveDialog(); });
    connect(csvAction, &QAction::triggered, this, [this] {
        const auto path =
            QFileDialog::getSaveFileName(this, "Export analysis", "analysis.csv", "CSV (*.csv)");
        if (!path.isEmpty())
            try {
                exportCsv(path);
                statusBar()->showMessage("Analysis exported to " + path, 6000);
            } catch (const std::exception &e) {
                showError(e);
            }
    });
    connect(svgAction, &QAction::triggered, this, [this] {
        const auto path =
            QFileDialog::getSaveFileName(this, "Export all graphs", "raillab-report.svg", "SVG (*.svg)");
        if (!path.isEmpty())
            try {
                exportSvg(path);
                statusBar()->showMessage("Graph report exported to " + path, 6000);
            } catch (const std::exception &e) {
                showError(e);
            }
    });
    auto *saveShortcut = new QShortcut(QKeySequence::Save, this);
    connect(saveShortcut, &QShortcut::activated, this, [this] { saveDialog(); });
    auto *undoShortcut = new QShortcut(QKeySequence::Undo, this);
    connect(undoShortcut, &QShortcut::activated, this, [this] { history(false); });
    auto *redoShortcut = new QShortcut(QKeySequence::Redo, this);
    connect(redoShortcut, &QShortcut::activated, this, [this] { history(true); });
    auto *cancelShortcut = new QShortcut(QKeySequence(Qt::Key_Escape), this);
    connect(cancelShortcut, &QShortcut::activated, this, &Window::clearDraft);
}

void Window::setDocument(Project project) {
    rail::validate(project.settings);
    project.normalizeForces();
    if (project.points.size() >= 2) {
        const rail::Track valid(project.points);
    }
    project_ = std::move(project);
    filePath_.clear();
    undo_.clear();
    redo_.clear();
    dirty_ = false;
    draft_ = false;
    editIndex_ = -1;
    insertIndex_ = -1;
    {
        QSignalBlocker block(segments_);
        segments_->setCurrentIndex(-1);
    }
    refresh(true);
}
void Window::refresh(bool fit) {
    loading_ = true;
    sampleModel_->setAnalysis(nullptr);
    track_.reset();
    analysis_.reset();
    points_->clear();
    for (std::size_t i = 0; i < project_.points.size(); ++i) {
        const auto p = project_.points[i];
        points_->addItem(QString("%1     (%2, %3)").arg(i + 1, 2).arg(p.x, 0, 'f', 2).arg(p.y, 0, 'f', 2));
    }
    const int selectedSegment = segments_->currentIndex();
    segments_->clear();
    for (std::size_t i = 0; i < project_.forwardForces.size(); ++i)
        segments_->addItem(QString("Point %1 → Point %2").arg(i + 1).arg(i + 2));
    segments_->setCurrentIndex(segments_->count() ? std::clamp(selectedSegment, 0, segments_->count() - 1)
                                                  : -1);
    pointCount_->setText(QString("%1 / 128").arg(project_.points.size()));
    const auto &p = project_.settings;
    const double values[]{p.initialSpeed, p.mass, p.gravity, p.resistance, p.dragArea, p.airDensity, p.step};
    for (int i = 0; i < 7; ++i)
        physics_[i]->setValue(values[i]);
    projectTitle_->setText(project_.name + (dirty_ ? "  ·  unsaved" : ""));
    setWindowTitle(QString("%1%2 — RailLab").arg(project_.name, dirty_ ? " *" : ""));
    undoButton_->setEnabled(!undo_.empty());
    redoButton_->setEnabled(!redo_.empty());
    remove_->setEnabled(false);
    insert_->setEnabled(false);
    for (auto *metric : metrics_)
        metric->setText("—");
    if (project_.points.size() >= 2) {
        try {
            track_.emplace(project_.points);
            analysis_ = rail::analyze(*track_, p, project_.forwardForces);
            const auto &a = *analysis_;
            metrics_[0]->setText(QString::number(a.trackLength, 'f', 1) + " m");
            metrics_[1]->setText(QString::number(a.maxSpeed, 'f', 1) + " m/s");
            metrics_[2]->setText(QString::number(a.maxFeltG, 'f', 2) + " g");
            metrics_[2]->setToolTip("Maximum sampled resultant non-gravitational force divided by mass × "
                                    "gravity. This is an idealized point-mass result.");
            metrics_[3]->setText(a.timeResolutionLimited
                                     ? "Limit / see note"
                                     : QString::number(a.samples.back().time, 'f', 2) + " s");
            const bool complete = a.status == rail::RunStatus::Complete;
            status_->setStyleSheet(
                QString("color: %1; background: %2; border-radius: 5px; padding: 7px 10px; font-size: 11px;")
                    .arg(complete ? "#235c2c" : "#815300", complete ? "#e4eee4" : "#fff0d6"));
            status_->setText(
                complete
                    ? QString(
                          "Reached end   ·   %1 samples   ·   Distance step ≤ %2 m   ·   Hover to inspect")
                          .arg(a.samples.size())
                          .arg(p.step)
                    : QString("%1 at %2 m   ·   %3")
                          .arg(QString::fromStdString(rail::statusName(a.status)).toUpper())
                          .arg(a.samples.back().distance, 0, 'f', 2)
                          .arg(QString::fromStdString(a.message)));
            sampleModel_->setAnalysis(&*analysis_);
        } catch (const std::exception &e) {
            status_->setText(QString::fromUtf8(e.what()));
            status_->setStyleSheet("color: #b75c00; padding: 8px;");
        }
    } else {
        status_->setText("Confirm at least two points to build a track and calculate its physics.");
        status_->setStyleSheet("color: #444444; padding: 8px;");
    }
    loading_ = false;
    selectSegment();
    trackView_->setTrack(project_.points, track_ ? &*track_ : nullptr, analysis_ ? &*analysis_ : nullptr);
    clearDraft();
    if (fit)
        trackView_->fit();
    updatePlots();
}
void Window::clearDraft() {
    draft_ = false;
    editIndex_ = -1;
    insertIndex_ = -1;
    {
        QSignalBlocker block(points_);
        points_->setCurrentRow(-1);
    }
    remove_->setEnabled(false);
    insert_->setEnabled(false);
    updatePreview();
}
void Window::startNewPoint() {
    editIndex_ = -1;
    insertIndex_ = -1;
    {
        QSignalBlocker block(points_);
        points_->setCurrentRow(-1);
    }
    loading_ = true;
    if (!project_.points.empty()) {
        x_->setValue(project_.points.back().x + 20);
        y_->setValue(project_.points.back().y);
    } else {
        x_->setValue(0);
        y_->setValue(20);
    }
    loading_ = false;
    draftPoint_ = {x_->value(), y_->value()};
    draft_ = true;
    updatePreview();
}
void Window::startInsertPoint() {
    const int index = points_->currentRow();
    if (index < 0 || index + 1 >= static_cast<int>(project_.points.size()))
        return;
    insertIndex_ = index + 1;
    editIndex_ = -1;
    draftPoint_ = (project_.points[static_cast<std::size_t>(index)] +
                   project_.points[static_cast<std::size_t>(index + 1)]) /
                  2;
    loading_ = true;
    x_->setValue(draftPoint_.x);
    y_->setValue(draftPoint_.y);
    loading_ = false;
    draft_ = true;
    updatePreview();
}
void Window::selectPoint(int index) {
    if (loading_ || index < 0 || index >= static_cast<int>(project_.points.size()))
        return;
    editIndex_ = index;
    insertIndex_ = -1;
    draftPoint_ = project_.points[static_cast<std::size_t>(index)];
    loading_ = true;
    x_->setValue(draftPoint_.x);
    y_->setValue(draftPoint_.y);
    loading_ = false;
    draft_ = true;
    remove_->setEnabled(true);
    insert_->setEnabled(index + 1 < static_cast<int>(project_.points.size()));
    if (segments_->count())
        segments_->setCurrentIndex(std::min(index, segments_->count() - 1));
    updatePreview();
}
void Window::updatePreview() {
    confirm_->setText(editIndex_ >= 0     ? "Apply point edit"
                      : insertIndex_ >= 0 ? "Insert point"
                                          : "Add point");
    cancel_->setEnabled(draft_);
    confirm_->setEnabled(false);
    if (!draft_) {
        trackView_->setPreview({}, nullptr, std::nullopt);
        draftHint_->setText("No unconfirmed changes.");
        return;
    }
    if (!x_->hasAcceptableInput() || !y_->hasAcceptableInput()) {
        trackView_->setPreview({}, nullptr, std::nullopt);
        draftHint_->setText("Enter a complete number in both coordinate fields.");
        return;
    }
    const rail::Vec2 candidate = draftPoint_;
    auto trial = project_.points;
    if (editIndex_ >= 0)
        trial[static_cast<std::size_t>(editIndex_)] = candidate;
    else if (insertIndex_ >= 0)
        trial.insert(trial.begin() + insertIndex_, candidate);
    else
        trial.push_back(candidate);
    try {
        std::optional<rail::Track> preview;
        if (trial.size() >= 2)
            preview.emplace(trial);
        trackView_->setPreview(trial, preview ? &*preview : nullptr, candidate);
        const bool unchanged =
            editIndex_ >= 0 &&
            rail::norm(candidate - project_.points[static_cast<std::size_t>(editIndex_)]) < 1e-9;
        confirm_->setEnabled(!unchanged);
        draftHint_->setText(unchanged           ? "Change X or Y to preview an edit."
                            : editIndex_ >= 0   ? "Preview only. Apply to update this point."
                            : insertIndex_ >= 0 ? "Preview only. Insert to split the selected segment."
                                                : "Preview only. Add point to include it in the analysis.");
    } catch (const std::exception &e) {
        trackView_->setPreview({}, nullptr, candidate);
        draftHint_->setText(QString::fromUtf8(e.what()));
    }
}
void Window::pushUndo() {
    undo_.push_back(project_);
    if (undo_.size() > 100)
        undo_.erase(undo_.begin());
    redo_.clear();
    dirty_ = true;
}
void Window::commitPoint() {
    if (!draft_ || !confirm_->isEnabled())
        return;
    pushUndo();
    const rail::Vec2 p = draftPoint_;
    if (editIndex_ >= 0)
        project_.points[static_cast<std::size_t>(editIndex_)] = p;
    else
        project_.insertPoint(
            insertIndex_ >= 0 ? static_cast<std::size_t>(insertIndex_) : project_.points.size(), p);
    refresh(true);
}
void Window::removePoint() {
    const int index = points_->currentRow();
    if (index < 0)
        return;
    auto trial = project_;
    trial.erasePoint(static_cast<std::size_t>(index));
    try {
        if (trial.points.size() >= 2) {
            const rail::Track valid(trial.points);
        }
        pushUndo();
        project_ = std::move(trial);
        refresh(true);
    } catch (const std::exception &e) {
        showError(e);
    }
}
void Window::movePoint(int direction) {
    const int index = points_->currentRow(), next = index + direction;
    if (index < 0 || next < 0 || next >= static_cast<int>(project_.points.size()))
        return;
    auto trial = project_;
    trial.swapPoints(static_cast<std::size_t>(index), static_cast<std::size_t>(next));
    try {
        if (trial.points.size() >= 2) {
            const rail::Track valid(trial.points);
        }
        pushUndo();
        project_ = std::move(trial);
        refresh(true);
        points_->setCurrentRow(next);
    } catch (const std::exception &e) {
        showError(e);
    }
}
void Window::history(bool redo) {
    auto &source = redo ? redo_ : undo_;
    auto &destination = redo ? undo_ : redo_;
    if (source.empty()) {
        clearDraft();
        return;
    }
    destination.push_back(project_);
    project_ = source.back();
    source.pop_back();
    dirty_ = true;
    refresh(true);
}
void Window::physicsChanged(int settingIndex) {
    if (loading_)
        return;
    pushUndo();
    auto &p = project_.settings;
    double *fields[]{&p.initialSpeed, &p.mass,       &p.gravity, &p.resistance,
                     &p.dragArea,     &p.airDensity, &p.step};
    // Other fields may contain more precision than their controls display.
    *fields[settingIndex] = physics_[settingIndex]->value();
    refreshPreservingDraft();
}
void Window::selectSegment() {
    if (loading_)
        return;
    const int index = segments_->currentIndex();
    segments_->setEnabled(index >= 0);
    forwardForce_->setEnabled(index >= 0);
    forwardForce_->setText(
        index >= 0 ? QString::number(project_.forwardForces[static_cast<std::size_t>(index)], 'g', 17)
                   : QString());
    updateForceInput();
}
void Window::updateForceInput() {
    if (loading_)
        return;
    const int index = segments_->currentIndex();
    bool valid = false;
    const double force = forwardForce_->text().toDouble(&valid);
    valid = valid && std::isfinite(force);
    const bool changed =
        index >= 0 && valid && force != project_.forwardForces[static_cast<std::size_t>(index)];
    applyForce_->setEnabled(changed);
    forceHint_->setText(index < 0 ? "Add two points to set a segment force."
                        : !valid ? "Enter a finite force: positive = propulsion, negative = braking, 0 = off."
                        : changed ? "Positive = propulsion; negative = braking; 0 = off. Click Apply force."
                                  : "Positive = propulsion; negative = braking; 0 = off.");
}
void Window::applyForce() {
    updateForceInput();
    if (!applyForce_->isEnabled())
        return;
    pushUndo();
    project_.forwardForces[static_cast<std::size_t>(segments_->currentIndex())] =
        forwardForce_->text().toDouble();
    refreshPreservingDraft();
}
void Window::refreshPreservingDraft() {
    // Settings changed, but keep my preview.
    const bool draft = draft_;
    const int index = editIndex_;
    const int insertion = insertIndex_;
    const int selectedPoint = points_->currentRow();
    const rail::Vec2 point = draftPoint_;
    const double x = x_->value(), y = y_->value();
    refresh();
    if (draft) {
        editIndex_ = index;
        insertIndex_ = insertion;
        draftPoint_ = point;
        draft_ = true;
        loading_ = true;
        x_->setValue(x);
        y_->setValue(y);
        loading_ = false;
        {
            QSignalBlocker block(points_);
            points_->setCurrentRow(selectedPoint);
        }
        remove_->setEnabled(selectedPoint >= 0);
        insert_->setEnabled(selectedPoint >= 0 &&
                            selectedPoint + 1 < static_cast<int>(project_.points.size()));
        updatePreview();
    }
}
void Window::updatePlots() {
    const bool time = axis_->currentIndex() == 1;
    const QString xLabel = time ? "Elapsed time (s)" : "Distance along track (m)";
    auto series = [&](QString name, QColor color, auto value) {
        Series line{std::move(name), std::move(color), {}};
        if (analysis_) {
            line.points.reserve(static_cast<qsizetype>(analysis_->samples.size()));
            const rail::Sample *previous = nullptr;
            for (const auto &p : analysis_->samples) {
                if (previous && previous->forwardForce != p.forwardForce) {
                    // Draw the jump at the boundary, not a ramp across the preceding interval.
                    auto incoming = p;
                    incoming.forwardForce = previous->forwardForce;
                    incoming.tangentialAcceleration +=
                        (incoming.forwardForce - p.forwardForce) / project_.settings.mass;
                    incoming.acceleration =
                        std::hypot(incoming.tangentialAcceleration, incoming.normalAcceleration);
                    incoming.feltG = std::hypot(incoming.normalG,
                                                incoming.tangentialAcceleration / project_.settings.gravity +
                                                    incoming.track.tangent.y);
                    line.points.append({time ? p.time : p.distance, value(incoming)});
                }
                line.points.append({time ? p.time : p.distance, value(p)});
                previous = &p;
            }
        }
        return line;
    };
    plots_[0]->setData("Speed", "m/s", xLabel,
                       {series("Speed", teal, [](const auto &p) { return p.speed; })});
    plots_[1]->setData("G-load", "g", xLabel,
                       {series("Signed normal", amber, [](const auto &p) { return p.normalG; }),
                        series("Resultant felt", pink, [](const auto &p) { return p.feltG; })});
    plots_[2]->setData("Acceleration", "m/s²", xLabel,
                       {series("Tangential", blue, [](const auto &p) { return p.tangentialAcceleration; }),
                        series("Normal", teal, [](const auto &p) { return p.normalAcceleration; }),
                        series("Total", muted, [](const auto &p) { return p.acceleration; })});
    plots_[3]->setData(
        "Energy", "kJ", xLabel,
        {series("Kinetic", teal, [](const auto &p) { return p.kineticEnergy / 1000; }),
         series("Potential", blue, [](const auto &p) { return p.potentialEnergy / 1000; }),
         series("Total", pink, [](const auto &p) { return p.mechanicalEnergy / 1000; }),
         series("Lost", amber, [](const auto &p) { return p.dissipatedEnergy / 1000; }),
         series("Applied work", QColor("#7541a0"), [](const auto &p) { return p.propulsionWork / 1000; })});
    plots_[4]->setData("Height profile", "m", xLabel,
                       {series("Height", teal, [](const auto &p) { return p.track.position.y; })});
    plots_[5]->setData("Signed curvature", "1/m", xLabel,
                       {series("Curvature", pink, [](const auto &p) { return p.track.curvature; })});
    plots_[6]->setData(
        "Signed normal rail force", "kN", xLabel,
        {series("Normal rail force", amber, [](const auto &p) { return p.railForce / 1000; })});
    plots_[7]->setData("Track inclination", "degrees", xLabel,
                       {series("Tangent angle (wrapped)", blue, [](const auto &p) {
                           return std::atan2(p.track.tangent.y, p.track.tangent.x) * 180 / std::acos(-1.0);
                       })});
    plots_[8]->setData(time ? "Distance travelled" : "Elapsed time estimate", time ? "m" : "s", xLabel,
                       {series(time ? "Distance" : "Time", teal,
                               [time](const auto &p) { return time ? p.distance : p.time; })});
    for (auto *plot : plots_)
        plot->setCursor(std::nullopt);
    inspector_->setText("Hover over the track or any graph to inspect a sample.  ·  G-load is a model "
                        "output, not a design rating.");
}
void Window::inspect(double value, bool byDistance) {
    if (!analysis_ || analysis_->samples.empty())
        return;
    const bool time = axis_->currentIndex() == 1 && !byDistance;
    const auto &samples = analysis_->samples;
    auto it =
        std::lower_bound(samples.begin(), samples.end(), value, [time](const rail::Sample &p, double target) {
            return (time ? p.time : p.distance) < target;
        });
    if (it == samples.end())
        --it;
    const auto &p = *it;
    trackView_->setProbe(p.track.position);
    for (auto *plot : plots_)
        plot->setCursor(axis_->currentIndex() == 1 ? p.time : p.distance);
    inspector_->setText(QString("s  %1 m     t  %2 s     height  %3 m     speed  %4 m/s     normal load  %5 "
                                "g     rail force  %6 kN     applied force  %7 N")
                            .arg(p.distance, 0, 'f', 2)
                            .arg(p.time, 0, 'f', 2)
                            .arg(p.track.position.y, 0, 'f', 2)
                            .arg(p.speed, 0, 'f', 2)
                            .arg(p.normalG, 0, 'f', 2)
                            .arg(p.railForce / 1000, 0, 'f', 2)
                            .arg(p.forwardForce, 0, 'g', 6));
}
void Window::openFrom(const QString &path) {
    auto loaded = loadProject(path);
    setDocument(std::move(loaded));
    filePath_ = path;
}
void Window::saveTo(const QString &path) {
    saveProject(path, project_);
    filePath_ = path;
    dirty_ = false;
    projectTitle_->setText(project_.name);
    setWindowTitle(project_.name + " — RailLab");
    statusBar()->showMessage("Saved confirmed points and segment forces to " + path +
                                 (draft_ ? "  ·  Preview is not saved." : ""),
                             7000);
}
void Window::exportCsv(const QString &path) const {
    if (!analysis_)
        throw std::runtime_error("Confirm at least two valid points before exporting.");
    std::ostringstream output;
    rail::writeCsv(output, *analysis_);
    const auto bytes = QByteArray::fromStdString(output.str());
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
        throw std::runtime_error("Could not save the CSV export.");
}
void Window::exportSvg(const QString &path) const {
    if (!analysis_)
        throw std::runtime_error("Confirm at least two valid points before exporting.");
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly))
        throw std::runtime_error(file.errorString().toStdString());
    QSvgGenerator svg;
    svg.setOutputDevice(&file);
    svg.setResolution(96);
    svg.setSize({1440, 2220});
    svg.setViewBox(QRect(0, 0, 1440, 2220));
    svg.setTitle("RailLab — " + project_.name);
    svg.setDescription("Static Bezier track analysis. Educational held-to-rail point mass. All values in SI "
                       "units. Unconfirmed points excluded.");
    QPainter p;
    if (!p.begin(&svg))
        throw std::runtime_error("Could not render the SVG report.");
    p.fillRect(QRect(0, 0, 1440, 2220), QColor("#ffffff"));
    QFont font("Arial");
    font.setPixelSize(18);
    font.setBold(true);
    p.setFont(font);
    p.setPen(QColor("#222222"));
    p.drawText(QPointF(32, 47), "RailLab  /  " + project_.name);
    font.setPixelSize(13);
    font.setBold(false);
    p.setFont(font);
    p.setPen(QColor("#444444"));
    const auto &a = *analysis_;
    p.drawText(
        QPointF(32, 78),
        QString(
            "%1  ·  %2 points  ·  Track %3 m  ·  Traversed %4 m  ·  Max speed %5 m/s  ·  Peak felt load %6 g")
            .arg(QString::fromStdString(rail::statusName(a.status)))
            .arg(project_.points.size())
            .arg(a.trackLength, 0, 'f', 2)
            .arg(a.samples.back().distance, 0, 'f', 2)
            .arg(a.maxSpeed, 0, 'f', 2)
            .arg(a.maxFeltG, 0, 'f', 2));
    const auto &s = project_.settings;
    p.drawText(QPointF(32, 103), QString("v0 %1 m/s  ·  Mass %2 kg  ·  g %3 m/s²  ·  Resistance %4 m/s²  ·  "
                                         "CdA %5 m²  ·  Air %6 kg/m³  ·  Step %7 m")
                                     .arg(s.initialSpeed)
                                     .arg(s.mass)
                                     .arg(s.gravity)
                                     .arg(s.resistance)
                                     .arg(s.dragArea)
                                     .arg(s.airDensity)
                                     .arg(s.step));
    p.drawText(QRectF(32, 116, 1376, 50), Qt::TextWordWrap, QString::fromStdString(a.message));
    // Only export points we actually kept.
    TrackView reportView;
    reportView.resize(1376, 360);
    reportView.setTrack(project_.points, &*track_, &*analysis_);
    reportView.fit();
    reportView.draw(p, QRectF(32, 176, 1376, 360), true);
    for (std::size_t i = 0; i < plots_.size(); ++i)
        plots_[i]->draw(p, QRectF(32 + double(i % 2) * 696, 558 + double(i / 2) * 306, 680, 288), false);
    font.setPixelSize(12);
    p.setFont(font);
    p.setPen(QColor("#444444"));
    p.drawText(QRectF(32, 2105, 1376, 85), Qt::TextWordWrap,
               "Model: open 2D track; point mass constrained to the rails; gravity, quadratic air drag and "
               "constant resistance, and per-segment propulsion or braking. "
               "Normal load is signed relative to the left normal. Felt load is the resultant "
               "non-gravitational force / (m g). "
               "Time and peak values are numerical estimates. No free flight, backwards pass, train "
               "geometry, banking or structural model. "
               "Educational analysis, not engineering certification.");
    p.end();
    if (!file.commit())
        throw std::runtime_error("Could not save the SVG report.");
}
bool Window::saveDialog() {
    QString path = filePath_;
    if (path.isEmpty())
        path = QFileDialog::getSaveFileName(this, "Save RailLab project", "track.json",
                                            "RailLab project (*.json)");
    if (path.isEmpty())
        return false;
    try {
        saveTo(path);
        return true;
    } catch (const std::exception &e) {
        showError(e);
        return false;
    }
}
bool Window::confirmDiscard() {
    if (!dirty_ && !draft_)
        return true;
    const auto answer = QMessageBox::question(
        this, "Unsaved changes",
        "Save the confirmed project before continuing? Any unconfirmed preview will be discarded.",
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel, QMessageBox::Save);
    if (answer == QMessageBox::Cancel)
        return false;
    return answer == QMessageBox::Discard || saveDialog();
}
void Window::showError(const std::exception &error) {
    QMessageBox::warning(this, "RailLab", QString::fromUtf8(error.what()));
}
void Window::closeEvent(QCloseEvent *event) {
    if (confirmDiscard())
        event->accept();
    else
        event->ignore();
}

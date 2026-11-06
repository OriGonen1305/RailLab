#include "window.hpp"

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QScrollBar>
#include <QSvgRenderer>
#include <QTabWidget>
#include <QTableView>
#include <QTemporaryDir>
#include <QTest>
#include <cmath>
#include <limits>

namespace {
Project poweredProject() {
    Project p;
    p.name = "Segment propulsion check";
    p.points = {{0, 0}, {5, 0}, {10, 0}, {15, 0}, {20, 0}};
    p.settings.initialSpeed = 0;
    p.settings.mass = 100;
    p.settings.dragArea = p.settings.resistance = 0;
    p.forwardForces = {100, 0, 250, 42.1234567890123};
    return p;
}
} // namespace

class UiTests : public QObject {
    Q_OBJECT
  private slots:
    void propulsionStartsCartThroughUi() {
        Window w;
        w.setDocument(preset(3));
        w.show();
        for (double x : {0.0, 5.0, 10.0}) {
            QTest::mouseClick(w.findChild<QPushButton *>("newPoint"), Qt::LeftButton);
            w.findChild<QDoubleSpinBox *>("pointX")->setValue(x);
            w.findChild<QDoubleSpinBox *>("pointY")->setValue(0);
            QTest::mouseClick(w.findChild<QPushButton *>("confirmPoint"), Qt::LeftButton);
        }
        w.findChild<QDoubleSpinBox *>("initialSpeed")->setValue(0);
        w.findChild<QDoubleSpinBox *>("mass")->setValue(100);
        w.findChild<QDoubleSpinBox *>("resistance")->setValue(0);
        w.findChild<QDoubleSpinBox *>("dragArea")->setValue(0);
        QCOMPARE(w.document().forwardForces, std::vector<double>({0, 0}));
        QCOMPARE(w.analysis()->status, rail::RunStatus::CannotStart);
        auto *force = w.findChild<QLineEdit *>("forwardForce");
        force->setFocus();
        force->selectAll();
        QTest::keyClicks(force, "100");
        QTest::mouseClick(w.findChild<QPushButton *>("applyForce"), Qt::LeftButton);
        QCOMPARE(w.document().forwardForces, std::vector<double>({100, 0}));
        QCOMPARE(w.analysis()->status, rail::RunStatus::Complete);
        QVERIFY(std::abs(w.analysis()->samples.back().speed - std::sqrt(10)) < 1e-10);
        QVERIFY(std::abs(w.analysis()->samples.back().propulsionWork - 500) < 1e-8);
        w.findChild<QComboBox *>("segmentSelector")->setCurrentIndex(1);
        QCOMPARE(force->text(), QString("0"));
        QTest::mouseClick(w.findChild<QPushButton *>("undo"), Qt::LeftButton);
        QCOMPARE(w.analysis()->status, rail::RunStatus::CannotStart);
        QTest::mouseClick(w.findChild<QPushButton *>("redo"), Qt::LeftButton);
        QCOMPARE(w.analysis()->status, rail::RunStatus::Complete);
    }
    void segmentForceEditorUndoRedoAndIsolation() {
        Window w;
        const auto p = poweredProject();
        w.setDocument(p);
        w.show();
        auto *segments = w.findChild<QComboBox *>("segmentSelector");
        auto *force = w.findChild<QLineEdit *>("forwardForce");
        auto *apply = w.findChild<QPushButton *>("applyForce");
        QCOMPARE(segments->count(), 4);
        segments->setCurrentIndex(1);
        QCOMPARE(segments->currentText(), QString("Point 2 → Point 3"));
        QCOMPARE(force->text().toDouble(), 0.0);
        QVERIFY(!apply->isEnabled());
        force->setFocus();
        force->selectAll();
        QTest::keyClicks(force, "75.5");
        QCOMPARE(w.document().forwardForces, p.forwardForces);
        QTest::mouseClick(apply, Qt::LeftButton);
        auto expected = p.forwardForces;
        expected[1] = 75.5;
        QCOMPARE(w.document().forwardForces, expected);
        QCOMPARE(segments->currentIndex(), 1);
        QVERIFY(w.windowTitle().contains('*'));
        QVERIFY(w.analysis());
        QVERIFY(std::abs(w.analysis()->samples.back().propulsionWork -
                         5 * (100 + 75.5 + 250 + p.forwardForces[3])) < 1e-7);
        QTest::mouseClick(w.findChild<QPushButton *>("undo"), Qt::LeftButton);
        QCOMPARE(w.document().forwardForces, p.forwardForces);
        QCOMPARE(force->text().toDouble(), 0.0);
        QTest::mouseClick(w.findChild<QPushButton *>("redo"), Qt::LeftButton);
        QCOMPARE(w.document().forwardForces, expected);
        segments->setCurrentIndex(3);
        QCOMPARE(force->text().toDouble(), p.forwardForces[3]);
        QVERIFY(!apply->isEnabled());
        QCOMPARE(w.document().forwardForces, expected); // Merely selecting does not round or edit.
        auto *table = w.findChild<QTableView *>("sampleTable");
        QCOMPARE(table->model()->headerData(13, Qt::Horizontal).toString(), QString("Applied (N)"));
        QCOMPARE(table->model()->headerData(14, Qt::Horizontal).toString(), QString("Applied work (J)"));
    }
    void brakingEditorUndoSaveReopenAndExport() {
        QTemporaryDir dir;
        QVERIFY(dir.isValid());
        Project p;
        p.name = "Braking check";
        p.points = {{0, 0}, {5, 0}, {10, 0}};
        p.settings.initialSpeed = 2;
        p.settings.mass = 100;
        p.settings.dragArea = p.settings.resistance = 0;
        p.forwardForces = {0, 250};
        Window w;
        w.setDocument(p);
        w.show();
        auto *segments = w.findChild<QComboBox *>("segmentSelector");
        segments->setCurrentIndex(0);
        QCOMPARE(segments->currentText(), QString("Point 1 → Point 2"));
        auto *force = w.findChild<QLineEdit *>("forwardForce");
        QCOMPARE(w.findChild<QLabel *>("forwardForceLabel")->text(), QString("Applied force (N)"));
        QVERIFY(force->toolTip().contains("negative = braking"));
        force->setFocus();
        force->selectAll();
        QTest::keyClicks(force, "-100");
        auto *apply = w.findChild<QPushButton *>("applyForce");
        QVERIFY(apply->isEnabled());
        QTest::mouseClick(apply, Qt::LeftButton);
        QCOMPARE(w.document().forwardForces, std::vector<double>({-100, 250}));
        QCOMPARE(w.analysis()->status, rail::RunStatus::Stalled);
        QVERIFY(std::abs(w.analysis()->samples.back().distance - 2) < 1e-10);
        QVERIFY(std::abs(w.analysis()->samples.back().propulsionWork + 200) < 1e-8);
        QVERIFY(w.findChild<QLabel *>("forceHint")->text().contains("negative = braking"));
        w.findChild<QPushButton *>("undo")->click();
        QCOMPARE(w.document().forwardForces, p.forwardForces);
        QCOMPARE(w.analysis()->status, rail::RunStatus::Complete);
        w.findChild<QPushButton *>("redo")->click();
        QCOMPARE(w.document().forwardForces, std::vector<double>({-100, 250}));
        const QString path = dir.filePath("braking.json");
        w.saveTo(path);
        Window reopened;
        reopened.openFrom(path);
        QCOMPARE(reopened.document().forwardForces, w.document().forwardForces);
        QCOMPARE(reopened.analysis()->status, rail::RunStatus::Stalled);
        QVERIFY(std::abs(reopened.analysis()->samples.back().propulsionWork + 200) < 1e-8);
        reopened.findChild<QComboBox *>("segmentSelector")->setCurrentIndex(0);
        QCOMPARE(reopened.findChild<QLineEdit *>("forwardForce")->text(), QString("-100"));
        const QString csvPath = dir.filePath("braking.csv");
        reopened.exportCsv(csvPath);
        QFile csv(csvPath);
        QVERIFY(csv.open(QIODevice::ReadOnly));
        const auto fields = csv.readAll().trimmed().split('\n').back().split(',');
        QCOMPARE(fields[16].toDouble(), -100.0);
        QVERIFY(std::abs(fields[17].toDouble() + 200) < 1e-8);
        const QString artifactDirectory = qEnvironmentVariable("RAILLAB_SCREENSHOT_DIR");
        if (!artifactDirectory.isEmpty()) {
            QDir().mkpath(artifactDirectory);
            QApplication::processEvents();
            QVERIFY(w.grab().save(artifactDirectory + "/braking.png"));
            w.saveTo(artifactDirectory + "/braking.json");
            w.exportCsv(artifactDirectory + "/braking.csv");
        }
    }
    void signedForcesSurvivePointSplitsAndMerges() {
        auto p = poweredProject();
        p.forwardForces = {100, -50, -125.5, 42};
        p.insertPoint(2, {7.5, 0});
        QCOMPARE(p.forwardForces, std::vector<double>({100, -50, -50, -125.5, 42}));
        p.erasePoint(2);
        QCOMPARE(p.forwardForces, std::vector<double>({100, -50, -125.5, 42}));
        p.erasePoint(2);
        QCOMPARE(p.forwardForces, std::vector<double>({100, -50, 42}));
    }
    void forceInputValidationAndEmptyTracks() {
        Window w;
        w.setDocument(poweredProject());
        auto *force = w.findChild<QLineEdit *>("forwardForce");
        auto *apply = w.findChild<QPushButton *>("applyForce");
        for (const QString text : {QString(), QString("nan"), QString("inf"), QString("-inf"),
                                   QString("1e309"), QString("-1e309"), QString("100 N")}) {
            force->setText(text);
            QVERIFY2(!apply->isEnabled(), qPrintable(text));
            QCOMPARE(w.document().forwardForces[0], 100.0);
        }
        force->setText("2.5e2");
        QVERIFY(apply->isEnabled());
        apply->click();
        QCOMPARE(w.document().forwardForces[0], 250.0);
        w.setDocument(preset(3));
        QVERIFY(!force->isEnabled());
        QVERIFY(!apply->isEnabled());
        Project one;
        one.points = {{0, 0}};
        w.setDocument(one);
        QVERIFY(!force->isEnabled());
        one.points.push_back({5, 0});
        w.setDocument(one);
        QVERIFY(force->isEnabled());
        QCOMPARE(force->text(), QString("0"));
        QCOMPARE(w.document().forwardForces, std::vector<double>({0}));
    }
    void forceSaveLoadAndLegacyProjects() {
        QTemporaryDir dir;
        QVERIFY(dir.isValid());
        Window w;
        w.setDocument(poweredProject());
        const QString path = dir.filePath("powered.json");
        w.saveTo(path);
        Window reopened;
        reopened.openFrom(path);
        QCOMPARE(reopened.document().forwardForces, w.document().forwardForces);
        QCOMPARE(reopened.analysis()->samples.back().speed, w.analysis()->samples.back().speed);
        QFile file(path);
        QVERIFY(file.open(QIODevice::ReadOnly));
        auto object = QJsonDocument::fromJson(file.readAll()).object();
        QCOMPARE(object["version"].toInt(), 2);
        QCOMPARE(object["segment_forces_N"].toArray().size(), 4);
        file.close();
        object["version"] = 1;
        object.remove("segment_forces_N");
        QVERIFY(file.open(QIODevice::WriteOnly));
        file.write(QJsonDocument(object).toJson());
        file.close();
        reopened.openFrom(path);
        QCOMPARE(reopened.document().forwardForces, std::vector<double>({0, 0, 0, 0}));
        QCOMPARE(reopened.analysis()->status, rail::RunStatus::CannotStart);
        // Existing version-1 example files also load with zero forces.
        for (const char *example :
             {"../examples/alpine.json", "../examples/loop.json", "../examples/energy-barrier.json"}) {
            const auto old = loadProject(QFINDTESTDATA(example));
            QCOMPARE(old.forwardForces, std::vector<double>(old.points.size() - 1, 0));
        }
    }
    void malformedForcesCannotReplaceTheDocument() {
        QTemporaryDir dir;
        Window w;
        const auto original = poweredProject();
        w.setDocument(original);
        const QString path = dir.filePath("invalid.json");
        saveProject(path, original);
        QFile file(path);
        QVERIFY(file.open(QIODevice::ReadOnly));
        const auto saved = QJsonDocument::fromJson(file.readAll()).object();
        file.close();
        for (const QJsonValue value :
             {QJsonValue(), QJsonValue("100"), QJsonValue(QJsonArray{}), QJsonValue(QJsonArray{100, 0, 250}),
              QJsonValue(QJsonArray{100, "bad", 250, 0}),
              QJsonValue(QJsonArray{100, QJsonValue(), 250, 0})}) {
            auto object = saved;
            object["segment_forces_N"] = value;
            QVERIFY(file.open(QIODevice::WriteOnly));
            file.write(QJsonDocument(object).toJson());
            file.close();
            QVERIFY_THROWS_EXCEPTION(std::invalid_argument, w.openFrom(path));
            QCOMPARE(w.document().forwardForces, original.forwardForces);
        }
        auto missing = saved;
        missing.remove("segment_forces_N");
        QVERIFY(file.open(QIODevice::WriteOnly));
        file.write(QJsonDocument(missing).toJson());
        file.close();
        QVERIFY_THROWS_EXCEPTION(std::invalid_argument, loadProject(path));
        for (double value : {-std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity(), std::nan("")}) {
            auto invalid = original;
            invalid.forwardForces[0] = value;
            QVERIFY_THROWS_EXCEPTION(std::invalid_argument, saveProject(path, invalid));
        }
    }
    void coordinateInsertDeleteAndForceHistory() {
        Window w;
        const auto p = poweredProject();
        w.setDocument(p);
        w.show();
        auto *points = w.findChild<QListWidget *>("pointsList");
        points->setCurrentRow(2);
        w.findChild<QDoubleSpinBox *>("pointY")->setValue(1);
        w.findChild<QPushButton *>("confirmPoint")->click();
        QCOMPARE(w.document().forwardForces, p.forwardForces);
        w.findChild<QPushButton *>("undo")->click();
        points->setCurrentRow(2);
        w.findChild<QPushButton *>("insertPoint")->click();
        QCOMPARE(w.document().points.size(), p.points.size()); // Insertion is still a preview.
        w.findChild<QPushButton *>("confirmPoint")->click();
        QCOMPARE(w.document().points.size(), p.points.size() + 1);
        QCOMPARE(w.document().points[3].x, 12.5);
        QCOMPARE(w.document().forwardForces, std::vector<double>({100, 0, 250, 250, p.forwardForces[3]}));
        w.findChild<QPushButton *>("undo")->click();
        QCOMPARE(w.document().forwardForces, p.forwardForces);
        w.findChild<QPushButton *>("redo")->click();
        QCOMPARE(w.document().forwardForces, std::vector<double>({100, 0, 250, 250, p.forwardForces[3]}));
        w.findChild<QPushButton *>("undo")->click();
        points->setCurrentRow(2);
        w.findChild<QPushButton *>("removePoint")->click();
        QCOMPARE(w.document().forwardForces, std::vector<double>({100, 0, p.forwardForces[3]}));
        QCOMPARE(w.document().points[2].x, 15.0); // Merged 5 -> 15 m keeps the incoming zero force.
        w.findChild<QPushButton *>("undo")->click();
        QCOMPARE(w.document().forwardForces, p.forwardForces);
    }
    void endpointEditsAndReorderingPreserveUnrelatedForces() {
        auto p = poweredProject();
        p.insertPoint(0, {-5, 0});
        QCOMPARE(p.forwardForces, std::vector<double>({0, 100, 0, 250, 42.1234567890123}));
        p.insertPoint(p.points.size(), {25, 0});
        QCOMPARE(p.forwardForces.back(), 0.0);
        p.erasePoint(0);
        p.erasePoint(p.points.size() - 1);
        QCOMPARE(p.forwardForces, poweredProject().forwardForces);
        while (!p.points.empty())
            p.erasePoint(0);
        QVERIFY(p.forwardForces.empty());
        p.insertPoint(0, {0, 0});
        p.insertPoint(1, {5, 0});
        QCOMPARE(p.forwardForces, std::vector<double>({0}));

        Window w;
        p = preset(0);
        p.forwardForces = {11, 22, 33, 44, 55, 66, 77};
        w.setDocument(p);
        w.findChild<QListWidget *>("pointsList")->setCurrentRow(2);
        w.findChild<QPushButton *>("moveDown")->click();
        QCOMPARE(w.document().forwardForces, std::vector<double>({11, 0, 0, 0, 55, 66, 77}));
        w.findChild<QPushButton *>("undo")->click();
        QCOMPARE(w.document().forwardForces, p.forwardForces);
        w.findChild<QPushButton *>("redo")->click();
        QCOMPARE(w.document().forwardForces, std::vector<double>({11, 0, 0, 0, 55, 66, 77}));
    }
    void forceEditPreservesPrecisePointAndInsertionPreviews() {
        Window w;
        auto p = poweredProject();
        p.points[1] = {5.123456, 0};
        w.setDocument(p);
        auto *points = w.findChild<QListWidget *>("pointsList");
        points->setCurrentRow(1);
        w.findChild<QDoubleSpinBox *>("pointY")->setValue(1);
        w.findChild<QLineEdit *>("forwardForce")->setText("60");
        w.findChild<QPushButton *>("applyForce")->click();
        QCOMPARE(w.document().points[1].y, 0.0);
        QVERIFY(w.findChild<QPushButton *>("confirmPoint")->isEnabled());
        w.findChild<QPushButton *>("confirmPoint")->click();
        QCOMPARE(w.document().points[1].x, p.points[1].x);
        QCOMPARE(w.document().points[1].y, 1.0);
        QCOMPARE(w.document().forwardForces[1], 60.0);
        points->setCurrentRow(1);
        w.findChild<QPushButton *>("insertPoint")->click();
        w.findChild<QLineEdit *>("forwardForce")->setText("70");
        w.findChild<QPushButton *>("applyForce")->click();
        w.findChild<QDoubleSpinBox *>("initialSpeed")->setValue(1);
        w.findChild<QPushButton *>("confirmPoint")->click();
        QCOMPARE(w.document().points.size(), p.points.size() + 1);
        QCOMPARE(w.document().forwardForces, std::vector<double>({100, 70, 70, 250, p.forwardForces[3]}));
    }
    void previewRequiresExplicitConfirmation() {
        Window w;
        w.setDocument(preset(3));
        w.show();
        auto *x = w.findChild<QDoubleSpinBox *>("pointX");
        auto *y = w.findChild<QDoubleSpinBox *>("pointY");
        auto *add = w.findChild<QPushButton *>("confirmPoint");
        x->setValue(10);
        y->setValue(30);
        QCOMPARE(w.document().points.size(), std::size_t(0));
        QVERIFY(!w.analysis());
        QVERIFY(add->isEnabled());
        x->setFocus();
        QTest::keyClick(x, Qt::Key_Return);
        QCOMPARE(w.document().points.size(), std::size_t(0));
        QTest::mouseClick(add, Qt::LeftButton);
        QCOMPARE(w.document().points.size(), std::size_t(1));
        QCOMPARE(w.document().points[0].x, 10.0);
        x->setValue(40);
        y->setValue(4);
        QCOMPARE(w.document().points.size(), std::size_t(1));
        QTest::mouseClick(add, Qt::LeftButton);
        QCOMPARE(w.document().points.size(), std::size_t(2));
        QVERIFY(w.analysis());
        QVERIFY(w.analysis()->samples.size() > 10);
        const double length = w.analysis()->trackLength;
        x->setValue(80);
        y->setValue(20);
        QCOMPARE(w.analysis()->trackLength, length);
        QTest::mouseClick(w.findChild<QPushButton *>("cancelPreview"), Qt::LeftButton);
        QVERIFY(!add->isEnabled());
        QCOMPARE(w.document().points.size(), std::size_t(2));
    }
    void keyboardTypingShowsPreviewWithoutCommit() {
        Window w;
        w.setDocument(preset(3));
        w.show();
        auto *x = w.findChild<QDoubleSpinBox *>("pointX");
        x->setFocus();
        x->selectAll();
        QTest::keyClicks(x, "25.5");
        QCOMPARE(x->value(), 25.5);
        QCOMPARE(w.document().points.size(), std::size_t(0));
        QVERIFY(w.findChild<QLabel *>("draftHint")->text().contains("Preview"));
    }
    void originCanBeConfirmedEvenWhenValueWasAlreadyZero() {
        Window w;
        w.setDocument(preset(3));
        w.show();
        auto *x = w.findChild<QDoubleSpinBox *>("pointX");
        x->setFocus();
        x->selectAll();
        QTest::keyClicks(x, "0");
        auto *add = w.findChild<QPushButton *>("confirmPoint");
        QVERIFY(add->isEnabled());
        QCOMPARE(w.document().points.size(), std::size_t(0));
        QTest::mouseClick(add, Qt::LeftButton);
        QCOMPARE(w.document().points.size(), std::size_t(1));
        QCOMPARE(w.document().points[0].x, 0.0);
        QCOMPARE(w.document().points[0].y, 0.0);
    }
    void editCancelUndoRedoAndRemove() {
        Window w;
        w.show();
        const auto original = w.document().points;
        auto *list = w.findChild<QListWidget *>("pointsList");
        list->setCurrentRow(1);
        w.findChild<QDoubleSpinBox *>("pointY")->setValue(22);
        QCOMPARE(w.document().points[1].y, original[1].y);
        QTest::mouseClick(w.findChild<QPushButton *>("confirmPoint"), Qt::LeftButton);
        QCOMPARE(w.document().points[1].y, 22.0);
        QTest::mouseClick(w.findChild<QPushButton *>("undo"), Qt::LeftButton);
        QCOMPARE(w.document().points[1].y, original[1].y);
        QTest::mouseClick(w.findChild<QPushButton *>("redo"), Qt::LeftButton);
        QCOMPARE(w.document().points[1].y, 22.0);
        list->setCurrentRow(1);
        QTest::mouseClick(w.findChild<QPushButton *>("removePoint"), Qt::LeftButton);
        QCOMPARE(w.document().points.size(), original.size() - 1);
        QTest::mouseClick(w.findChild<QPushButton *>("undo"), Qt::LeftButton);
        QCOMPARE(w.document().points.size(), original.size());
    }
    void duplicateRejectedAndPhysicsPreservesDraft() {
        Window w;
        w.show();
        const auto last = w.document().points.back();
        auto *x = w.findChild<QDoubleSpinBox *>("pointX");
        auto *y = w.findChild<QDoubleSpinBox *>("pointY");
        x->setValue(last.x);
        y->setValue(last.y);
        QVERIFY(!w.findChild<QPushButton *>("confirmPoint")->isEnabled());
        x->setValue(last.x + 20);
        y->setValue(10);
        const auto count = w.document().points.size();
        w.findChild<QDoubleSpinBox *>("initialSpeed")->setValue(9);
        QCOMPARE(w.document().points.size(), count);
        QCOMPARE(x->value(), last.x + 20);
        QVERIFY(w.findChild<QPushButton *>("confirmPoint")->isEnabled());
        QTest::mouseClick(w.findChild<QPushButton *>("confirmPoint"), Qt::LeftButton);
        QCOMPARE(w.document().points.size(), count + 1);
    }
    void coordinateEditPreservesUntouchedPrecision_data() {
        QTest::addColumn<bool>("editX");
        QTest::addColumn<bool>("changePhysics");
        QTest::newRow("Y only") << false << false;
        QTest::newRow("X only") << true << false;
        QTest::newRow("Y then physics") << false << true;
        QTest::newRow("X then physics") << true << true;
    }
    void coordinateEditPreservesUntouchedPrecision() {
        QFETCH(bool, editX);
        QFETCH(bool, changePhysics);
        QTemporaryDir dir;
        QVERIFY(dir.isValid());
        Project project = preset(0);
        project.points[1] = {18.123456, 26.234567};
        const auto original = project.points[1];
        const QString path = dir.filePath("precise-point.json");
        saveProject(path, project);

        Window w;
        w.openFrom(path);
        w.show();
        w.findChild<QListWidget *>("pointsList")->setCurrentRow(1);
        auto *apply = w.findChild<QPushButton *>("confirmPoint");
        QVERIFY(!apply->isEnabled()); // Selecting a point is not an edit.
        auto *coordinate = w.findChild<QDoubleSpinBox *>(editX ? "pointX" : "pointY");
        coordinate->setFocus();
        coordinate->selectAll();
        QTest::keyClicks(coordinate, "22");
        if (changePhysics)
            w.findChild<QDoubleSpinBox *>("initialSpeed")->setValue(9.5);
        QVERIFY(apply->isEnabled());
        QTest::mouseClick(apply, Qt::LeftButton);
        const rail::Vec2 expected = editX ? rail::Vec2{22, original.y} : rail::Vec2{original.x, 22};
        QVERIFY(w.document().points[1].x == expected.x);
        QVERIFY(w.document().points[1].y == expected.y);
        w.saveTo(path);

        Window reopened;
        reopened.openFrom(path);
        QVERIFY(reopened.document().points[1].x == expected.x);
        QVERIFY(reopened.document().points[1].y == expected.y);
    }
    void speedEditPreservesOtherSettingsPrecision() {
        QTemporaryDir dir;
        QVERIFY(dir.isValid());
        Project project = preset(0);
        project.settings = {4.123456, 300.123456,  9.806651234, 0.025123456,
                            0.123456, 1.225123456, 0.15123456};
        const QString path = dir.filePath("precise-settings.json");
        saveProject(path, project);

        Window w;
        w.openFrom(path);
        auto *speed = w.findChild<QDoubleSpinBox *>("initialSpeed");
        QVERIFY(speed);
        speed->setValue(9.5);
        const auto &actual = w.document().settings;
        const auto &original = project.settings;
        QCOMPARE(actual.initialSpeed, 9.5);
        // Unedited values must survive exactly, even beyond the displayed precision.
        QVERIFY(actual.mass == original.mass);
        QVERIFY(actual.gravity == original.gravity);
        QVERIFY(actual.resistance == original.resistance);
        QVERIFY(actual.dragArea == original.dragArea);
        QVERIFY(actual.airDensity == original.airDensity);
        QVERIFY(actual.step == original.step);
    }
    void saveLoadExportsAndMalformedInput() {
        QTemporaryDir dir;
        QVERIFY(dir.isValid());
        Window w;
        const QString path = dir.filePath("track.json");
        w.findChild<QDoubleSpinBox *>("pointX")->setValue(200);
        w.saveTo(path);
        const Project loaded = loadProject(path);
        QCOMPARE(loaded.points.size(), w.document().points.size()); // Preview stays out.
        QCOMPARE(loaded.settings.gravity, w.document().settings.gravity);
        Window reopened;
        reopened.openFrom(path);
        QCOMPARE(reopened.analysis()->trackLength, w.analysis()->trackLength);
        const QString csv = dir.filePath("analysis.csv"), svg = dir.filePath("report.svg");
        w.exportCsv(csv);
        w.exportSvg(svg);
        QFile data(csv);
        QVERIFY(data.open(QIODevice::ReadOnly));
        const QByteArray content = data.readAll();
        QCOMPARE(content.count('\n'), static_cast<qsizetype>(w.analysis()->samples.size() + 1));
        QVERIFY(content.startsWith("distance_m,time_s,"));
        QSvgRenderer rendered(svg);
        QVERIFY(rendered.isValid());
        QCOMPARE(rendered.viewBox(), QRect(0, 0, 1440, 2220));
        QImage report(720, 1110, QImage::Format_ARGB32);
        report.fill(Qt::transparent);
        QPainter painter(&report);
        rendered.render(&painter);
        painter.end();
        QVERIFY(report.save(dir.filePath("report.png")));
        QFile bad(dir.filePath("bad.json"));
        QVERIFY(bad.open(QIODevice::WriteOnly));
        bad.write("{\"format\":\"raillab\",\"version\":99}");
        bad.close();
        QVERIFY_THROWS_EXCEPTION(std::invalid_argument, reopened.openFrom(bad.fileName()));
        QCOMPARE(reopened.document().points.size(), loaded.points.size());
        QFile wrong(dir.filePath("wrong.json"));
        QVERIFY(wrong.open(QIODevice::WriteOnly));
        QFile saved(path);
        QVERIFY(saved.open(QIODevice::ReadOnly));
        auto object = QJsonDocument::fromJson(saved.readAll()).object();
        auto settings = object["settings"].toObject();
        settings["mass_kg"] = -2;
        object["settings"] = settings;
        wrong.write(QJsonDocument(object).toJson());
        wrong.close();
        QVERIFY_THROWS_EXCEPTION(std::invalid_argument, loadProject(wrong.fileName()));
    }
    void presetsAxisAndDataTable() {
        Window w;
        w.show();
        for (int i = 0; i < 3; ++i) {
            w.setDocument(preset(i));
            QVERIFY(w.analysis());
            if (i == 2)
                QCOMPARE(w.analysis()->status, rail::RunStatus::Stalled);
            else
                QCOMPARE(w.analysis()->status, rail::RunStatus::Complete);
            auto *table = w.findChild<QTableView *>("sampleTable");
            QCOMPARE(table->model()->rowCount(), static_cast<int>(w.analysis()->samples.size()));
            const auto count = w.analysis()->samples.size();
            w.findChild<QComboBox *>("horizontalAxis")->setCurrentIndex(1);
            QCOMPARE(w.analysis()->samples.size(), count);
            auto *tabs = w.findChild<QTabWidget *>("analysisTabs");
            for (int page = 0; page < tabs->count(); ++page) {
                tabs->setCurrentIndex(page);
                QApplication::processEvents();
                QVERIFY(!w.grab().isNull());
            }
            w.findChild<QComboBox *>("horizontalAxis")->setCurrentIndex(0);
        }
    }
    void screenshotsForVisualReview() {
        const QString directory = qEnvironmentVariable("RAILLAB_SCREENSHOT_DIR");
        if (directory.isEmpty())
            QSKIP("Set RAILLAB_SCREENSHOT_DIR to capture review artifacts.");
        QDir().mkpath(directory);
        Window w;
        w.show();
        QApplication::processEvents();
        QVERIFY(w.grab().save(directory + "/overview.png"));
        w.setDocument(poweredProject());
        QApplication::processEvents();
        QVERIFY(w.grab().save(directory + "/propulsion.png"));
        w.saveTo(directory + "/propulsion.json");
        w.exportCsv(directory + "/propulsion.csv");
        w.exportSvg(directory + "/propulsion.svg");
        QSvgRenderer poweredReport(directory + "/propulsion.svg");
        QVERIFY(poweredReport.isValid());
        QImage poweredImage(QSize(1440, 2220), QImage::Format_ARGB32);
        poweredImage.fill(Qt::transparent);
        QPainter poweredPainter(&poweredImage);
        poweredReport.render(&poweredPainter);
        poweredPainter.end();
        QVERIFY(poweredImage.save(directory + "/propulsion-report.png"));
        w.findChild<QTabWidget *>("analysisTabs")->setCurrentIndex(3);
        QApplication::processEvents();
        auto *dataScroll = w.findChild<QTableView *>("sampleTable")->horizontalScrollBar();
        dataScroll->setValue(dataScroll->maximum());
        QApplication::processEvents();
        QVERIFY(w.grab().save(directory + "/propulsion-data.png"));
        w.setDocument(preset(0));
        w.findChild<QTabWidget *>("analysisTabs")->setCurrentIndex(0);
        w.findChild<QPushButton *>("newPoint")->click();
        w.findChild<QDoubleSpinBox *>("pointX")->setValue(174);
        w.findChild<QDoubleSpinBox *>("pointY")->setValue(30);
        QApplication::processEvents();
        QVERIFY(w.grab().save(directory + "/preview.png"));
        w.setDocument(preset(1));
        w.findChild<QCheckBox *>("showHandles")->setChecked(true);
        w.findChild<QTabWidget *>("analysisTabs")->setCurrentIndex(1);
        QApplication::processEvents();
        QVERIFY(w.grab().save(directory + "/loop.png"));
        w.setDocument(preset(2));
        w.findChild<QTabWidget *>("analysisTabs")->setCurrentIndex(0);
        QApplication::processEvents();
        QVERIFY(w.grab().save(directory + "/stalled.png"));
        w.setDocument(preset(3));
        QApplication::processEvents();
        QVERIFY(w.grab().save(directory + "/empty.png"));
        w.setDocument(preset(0));
        w.resize(1100, 850);
        QApplication::processEvents();
        QVERIFY(w.grab().save(directory + "/compact.png"));
        w.exportSvg(directory + "/report.svg");
        QSvgRenderer svg(directory + "/report.svg");
        QImage image(QSize(1440, 2220), QImage::Format_ARGB32);
        image.fill(Qt::transparent);
        QPainter p(&image);
        svg.render(&p);
        p.end();
        QVERIFY(image.save(directory + "/report.png"));
    }
};

QTEST_MAIN(UiTests)
#include "ui_tests.moc"

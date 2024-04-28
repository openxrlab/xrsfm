#include <iostream>
#include <QApplication>
#include <QMainWindow>
#include <QtCore>
#include <QtGui>
#include <QtWidgets>
#include "src/ui/project_widget.h"
#include "src/feature/feature_processor.h"
#include "src/ui/model_viewer_widget.h"

namespace xrsfm {
class MainWindow : public QMainWindow {
  public:
    MainWindow() {
        model_viewer_widget_ = new ModelViewerWidget(this);
        setCentralWidget(model_viewer_widget_);
        model_viewer_widget_->map = new Map();
        // ReadColMapDataBinary(
        //     "/path/to/KITTI/00/results/",
        //     *model_viewer_widget_->map);

        std::setlocale(LC_NUMERIC, "C");
        resize(1024, 600);
        setWindowTitle(QString::fromStdString("XRSfM"));

        project_widget_ = new ProjectWidget(this);

        action_set_project_ = new QAction("Set Project", this);
        connect(action_set_project_, &QAction::triggered, this,
                &MainWindow::showMenu);
        action_start_reconstruction_ =
            new QAction("Start Reconstruction", this);
        connect(action_start_reconstruction_, &QAction::triggered, this,
                &MainWindow::mapping);

        file_toolbar_ = addToolBar(tr("File"));
        file_toolbar_->addAction(action_set_project_);
        file_toolbar_->addAction(action_start_reconstruction_);
        file_toolbar_->setIconSize(QSize(16, 16));
    }

    void showMenu() {
        project_widget_->Reset();
        project_widget_->show();
        project_widget_->raise();
    }

    void mapping() {
        const std::string image_path = project_widget_->GetImagePath();
        const std::string work_path = project_widget_->GetWorkspacePath();

        FeatureProcessor feature_processor(image_path, work_path);
        feature_processor.Run();

        // Map map;
        // PreProcess(work_path, camera_path, map);

        // IncrementalMapper imapper;
        // imapper.options.init_id1 = -1;
        // imapper.options.init_id2 = -1;
        // imapper.options.correct_pose = false;
        // imapper.options.stop_when_register_fail = true;
        // imapper.Reconstruct(map);
    }

    QToolBar *file_toolbar_;
    QAction *action_set_project_;
    QAction *action_start_reconstruction_;
    ProjectWidget *project_widget_;
    ModelViewerWidget *model_viewer_widget_;
};
} // namespace xrsfm

int main(int argc, char **argv) {
    using namespace xrsfm;
#if (QT_VERSION >= QT_VERSION_CHECK(5, 6, 0))
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
#endif

    QApplication app(argc, argv);
    QMainWindow window;
    MainWindow main_window;
    main_window.show();

    return app.exec();
}

#include <iostream>
#include <QApplication>
#include <QMainWindow>
#include <QtGui>
#include <QtWidgets>
#include "feature/feature_processor.h"
#include "ui/model_viewer_widget.h"
#include "ui/project_widget.h"

namespace xrsfm {
class MainWindow : public QMainWindow {
  public:
    MainWindow() {
        model_viewer_widget_ = new ModelViewerWidget(this);
        setCentralWidget(model_viewer_widget_);
        model_viewer_widget_->map = new Map();

        std::setlocale(LC_NUMERIC, "C");
        resize(1024, 600);
        setWindowTitle(QString::fromStdString("XRSfM"));

        project_widget_ = new ProjectWidget(this);

        action_view_model_ = new QAction("Open Sparse Model", this);
        connect(action_view_model_, &QAction::triggered, this,
                &MainWindow::selectModelPath);
        action_set_project_ = new QAction("Set Project", this);
        connect(action_set_project_, &QAction::triggered, this,
                &MainWindow::showMenu);
        action_start_reconstruction_ =
            new QAction("Start Reconstruction", this);
        connect(action_start_reconstruction_, &QAction::triggered, this,
                &MainWindow::mapping);

        file_toolbar_ = addToolBar(tr("File"));
        file_toolbar_->addAction(action_view_model_);
        // TODO the reconstruction part
        // file_toolbar_->addAction(action_set_project_);
        // file_toolbar_->addAction(action_start_reconstruction_);
        file_toolbar_->setIconSize(QSize(16, 16));
    }

    void selectModelPath() {
        const auto model_path = QFileDialog::getExistingDirectory(
            this, tr("Select model path..."), "", QFileDialog::ShowDirsOnly);
        std::string model_path_str = model_path.toUtf8().constData();
        model_path_str += "/";
        if (ReadColMapDataBinary(model_path_str, *model_viewer_widget_->map)) {
            model_viewer_widget_->Upload();
            std::cout << model_path_str << " load success\n";
        } else {
            std::cout << model_path_str << " load fail\n";
        }
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
    QAction *action_view_model_;
    QAction *action_set_project_;
    QAction *action_start_reconstruction_;
    ProjectWidget *project_widget_;
    ModelViewerWidget *model_viewer_widget_;
};
} // namespace xrsfm

int main(int argc, char **argv) {
#if (QT_VERSION >= QT_VERSION_CHECK(5, 6, 0))
    Q_INIT_RESOURCE(resources);
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
#endif

    QApplication app(argc, argv);
    QMainWindow window;
    xrsfm::MainWindow main_window;
    main_window.show();

    return app.exec();
}

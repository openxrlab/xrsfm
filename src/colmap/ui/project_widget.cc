// Copyright (c) 2023, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#include "project_widget.h"

ProjectWidget::ProjectWidget(QWidget *parent)
    : QWidget(parent), prev_selected_(false) {
    setWindowFlags(Qt::Dialog);
    setWindowModality(Qt::ApplicationModal);
    setWindowTitle("Project");

    // Image path.
    QPushButton *image_path_select = new QPushButton(tr("Select"), this);
    connect(image_path_select, &QPushButton::released, this,
            &ProjectWidget::SelectImagePath);
    image_path_text_ = new QLineEdit(this);
    image_path_text_->setText(QString::fromStdString(image_path_));

    // Workspace path.
    QPushButton *databse_path_open = new QPushButton(tr("Open"), this);
    connect(databse_path_open, &QPushButton::released, this,
            &ProjectWidget::SelectWorkspacePath);
    workspace_path_text_ = new QLineEdit(this);
    workspace_path_text_->setText(QString::fromStdString(workspace_path_));

    QGridLayout *grid = new QGridLayout(this);
    grid->addWidget(new QLabel(tr("Workspace"), this), 0, 0);
    grid->addWidget(workspace_path_text_, 0, 1);
    grid->addWidget(databse_path_open, 0, 2);
    grid->addWidget(new QLabel(tr("Images"), this), 1, 0);
    grid->addWidget(image_path_text_, 1, 1);
    grid->addWidget(image_path_select, 1, 2);
}

void ProjectWidget::Reset() {
    workspace_path_text_->clear();
    image_path_text_->clear();
}

std::string ProjectWidget::GetWorkspacePath() const {
    std::string path = workspace_path_text_->text().toUtf8().constData();
    if (path.back() != "/")
        path = path + "/";
    return path;
}

std::string ProjectWidget::GetImagePath() const {
    std::string path = image_path_text_->text().toUtf8().constData();
    if (path.back() != "/")
        path = path + "/";
    return path;
}

void ProjectWidget::SelectImagePath() {
    const auto image_path = QFileDialog::getExistingDirectory(
        this, tr("Select image path..."), DefaultDirectory(),
        QFileDialog::ShowDirsOnly);
    if (image_path != "") {
        image_path_text_->setText(image_path);
    }
}

void ProjectWidget::SelectWorkspacePath() {
    const auto workspace_path = QFileDialog::getExistingDirectory(
        this, tr("Select image path..."), DefaultDirectory(),
        QFileDialog::ShowDirsOnly);
    if (workspace_path != "") {
        workspace_path_text_->setText(workspace_path);
    }
}

QString ProjectWidget::DefaultDirectory() {
    // if (prev_selected_) {
    //     return "";
    // }

    // prev_selected_ = true;

    // if (!options_->project_path->empty()) {
    //     const auto parent_path = GetParentDir(*options_->project_path);
    //     if (ExistsDir(parent_path)) {
    //         return QString::fromStdString(parent_path);
    //     }
    // }

    // if (!workspace_path_text_->text().isEmpty()) {
    //     const auto parent_path =
    //         GetParentDir(workspace_path_text_->text().toUtf8().constData());
    //     if (ExistsDir(parent_path)) {
    //         return QString::fromStdString(parent_path);
    //     }
    // }

    // if (!image_path_text_->text().isEmpty()) {
    //     return image_path_text_->text();
    // }

    return "";
}

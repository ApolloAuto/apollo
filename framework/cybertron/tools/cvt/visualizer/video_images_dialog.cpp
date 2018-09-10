#include "video_images_dialog.h"
#include "ui_video_images_dialog.h"

VideoImagesDialog::VideoImagesDialog(QWidget *parent)
    : QDialog(parent), ui(new Ui::VideoImagesDialog) {
  ui->setupUi(this);
}

VideoImagesDialog::~VideoImagesDialog() { delete ui; }

int VideoImagesDialog::count(void) const { return ui->spinBox->value(); }

#ifndef VIDEO_IMAGES_DIALOG_H
#define VIDEO_IMAGES_DIALOG_H

#include <QDialog>

namespace Ui {
class VideoImagesDialog;
}

class VideoImagesDialog : public QDialog {
  Q_OBJECT

 public:
  explicit VideoImagesDialog(QWidget *parent = nullptr);
  ~VideoImagesDialog();

  int count(void) const;

 private:
  Ui::VideoImagesDialog *ui;
};

#endif  // VIDEO_IMAGES_DIALOG_H

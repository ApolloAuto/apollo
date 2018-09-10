#ifndef SCENE_CAMERA_DIALOG_H
#define SCENE_CAMERA_DIALOG_H

#include <QDialog>

namespace Ui {
class SceneCameraDialog;
}

class QVector3D;

class SceneCameraDialog : public QDialog {
  Q_OBJECT

 public:
  explicit SceneCameraDialog(QWidget *parent = nullptr);
  ~SceneCameraDialog();

 signals:
  void resetcamera();

  void sensitivityChanged(float);
  void cameraTypeChanged(int);

  void xValueChanged(double);
  void yValueChanged(double);
  void zValueChanged(double);
  void yawValueChanged(double);
  void pitchValueChanged(double);
  void rollValueChanged(double);

 public slots:
  void updateCameraAttitude(const QVector3D &);
  void updateCameraPos(const QVector3D &);

 private slots:
  void OnStepSlideChanged(int v);
  void onCameraTypeChanged(int);

 private:
  Ui::SceneCameraDialog *ui;
};

#endif  // SCENE_CAMERA_DIALOG_H

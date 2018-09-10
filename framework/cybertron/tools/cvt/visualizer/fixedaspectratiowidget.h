#ifndef FIXEDASPECTRATIOWIDGET_H
#define FIXEDASPECTRATIOWIDGET_H

#include <QTimer>
#include <QWidget>
#include "video_image_viewer.h"

class FixedAspectRatioWidget : public QWidget {
  Q_OBJECT
 public:
  explicit FixedAspectRatioWidget(QWidget *parent = nullptr, int index = 0);

  bool is_init(void) const { return viewer_.is_init_; }
  void SetupDynamicTexture(std::shared_ptr<Texture> &textureObj);

  void set_index(int i) { index_ = i; }
  int index(void) const { return index_; }

  void StartOrStopUpdate(bool b);
  int innerHeight(void) { return viewer_.plane_.texHeight(); }

 signals:
  void focusOnThis(FixedAspectRatioWidget *);

 protected:
  void mouseDoubleClickEvent(QMouseEvent *event) override;
  void contextMenuEvent(QContextMenuEvent *) override;
  void resizeEvent(QResizeEvent *) override;
  void paintEvent(QPaintEvent *event) override;

 private:
  int index_;
  QTimer refresh_timer_;
  VideoImgViewer viewer_;
};

#endif  // FIXEDASPECTRATIOWIDGET_H

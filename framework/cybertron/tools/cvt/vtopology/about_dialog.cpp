#include "about_dialog.h"
#include "ui_about_dialog.h"

AboutDialog::AboutDialog(QWidget *parent)
    : QDialog(parent), ui_(new Ui::AboutDialog) {
  ui_->setupUi(this);
}

AboutDialog::~AboutDialog() { delete ui_; }

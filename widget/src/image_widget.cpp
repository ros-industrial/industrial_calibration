#include "widget/image_widget.h"
#include <QPainter>

ImageWidget::ImageWidget(QWidget* parent) : QWidget(parent)
{
  image = QPixmap(size());
  image.fill(Qt::white);
}
  
void ImageWidget::setImage(const QString& filepath)
{
  image.load(filepath);
}
  
void ImageWidget::paintEvent(QPaintEvent *event)
{
  QWidget::paintEvent(event);
  QPainter painter(this);
  painter.drawPixmap(rect(), image);
}

void ImageWidget::resizeEvent(QResizeEvent *event)
{
  image = image.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
  update();  // Trigger a repaint
}
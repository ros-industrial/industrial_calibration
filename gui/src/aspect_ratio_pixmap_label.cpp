#include <industrial_calibration/gui/aspect_ratio_pixmap_label.h>
#include <QPainter>

namespace industrial_calibration
{
AspectRatioPixmapLabel::AspectRatioPixmapLabel(QWidget* parent) : QLabel(parent)
{
  this->setMinimumSize(1, 1);
  setScaledContents(false);
}

void AspectRatioPixmapLabel::setPixmap(const QPixmap& p)
{
  pix = p;
  auto scaled_pm = scaledPixmap();
  QLabel::setPixmap(scaled_pm);
}

int AspectRatioPixmapLabel::heightForWidth(int width) const
{
  return pix.isNull() ? this->height() : ((qreal)pix.height() * width) / pix.width();
}

QSize AspectRatioPixmapLabel::sizeHint() const
{
  int w = this->width();
  return QSize(w, heightForWidth(w));
}

QPixmap AspectRatioPixmapLabel::scaledPixmap() const
{
  return pix.scaled(this->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
}

void AspectRatioPixmapLabel::resizeEvent(QResizeEvent* e)
{
  if (!pix.isNull())
  {
    auto scaled_pm = scaledPixmap();
    QLabel::setPixmap(scaled_pm);
  }
}

}  // namespace industrial_calibration

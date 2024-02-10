#include "widget/image_widget.h"
#include <QPainter>

ImageWidget::ImageWidget(QWidget* parent) : QWidget(parent)
{
  image_original_ = QPixmap(size());
  image_original_.fill(Qt::white);
  image_scaled_ = image_original_;
}
  
void ImageWidget::setImage(const QString& filepath)
{
  image_original_.load(filepath);
  image_scaled_ = image_original_;
}
  
void ImageWidget::paintEvent(QPaintEvent *event)
{
  QWidget::paintEvent(event);
  QPainter painter(this);
  painter.drawPixmap(rect(), image_scaled_);
}

void ImageWidget::resizeEvent(QResizeEvent *event)
{
  image_scaled_ = image_original_.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
  update();  // Trigger a repaint
}

// AspectRatioPixmapLabel

AspectRatioPixmapLabel::AspectRatioPixmapLabel(QWidget *parent) :
    QLabel(parent)
{
    this->setMinimumSize(1,1);
    setScaledContents(false);
}

void AspectRatioPixmapLabel::setPixmap ( const QPixmap & p)
{
    pix = p;
    auto scaled_pm = scaledPixmap();
    QLabel::setPixmap(scaled_pm);
}

int AspectRatioPixmapLabel::heightForWidth( int width ) const
{
    return pix.isNull() ? this->height() : ((qreal)pix.height()*width)/pix.width();
}

QSize AspectRatioPixmapLabel::sizeHint() const
{
    int w = this->width();
    return QSize( w, heightForWidth(w) );
}

QPixmap AspectRatioPixmapLabel::scaledPixmap() const
{
    return pix.scaled(this->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
}

void AspectRatioPixmapLabel::resizeEvent(QResizeEvent * e)
{
    if(!pix.isNull())
    {
        auto scaled_pm = scaledPixmap();
        QLabel::setPixmap(scaled_pm);
    }
}

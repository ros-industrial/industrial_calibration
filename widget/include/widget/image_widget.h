#ifndef IMAGE_WIDGET_H
#define IMAGE_WIDGET_H

#include <QWidget>
#include <QPixmap>

class ImageWidget : public QWidget
{
  Q_OBJECT

public:
  ImageWidget(QWidget *parent = nullptr);

  void setImage(const QString& filepath);

protected:
  void paintEvent(QPaintEvent *event) override;
  void resizeEvent(QResizeEvent *event) override;
  
private:
  QPixmap image_original_;  // original image to load and be scaled
  QPixmap image_scaled_;  // scaled image to be drawn
};

#endif // IMAGE_WIDGET_H

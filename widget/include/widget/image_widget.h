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

#include <QLabel>

/**
 * @brief The AspectRatioPixmapLabel class
 * @link https://stackoverflow.com/questions/8211982/qt-resizing-a-qlabel-containing-a-qpixmap-while-keeping-its-aspect-ratio
 */
class AspectRatioPixmapLabel : public QLabel
{
    Q_OBJECT
public:
    explicit AspectRatioPixmapLabel(QWidget *parent = 0);
    virtual int heightForWidth( int width ) const;
    virtual QSize sizeHint() const;
    QPixmap scaledPixmap() const;
public slots:
    void setPixmap ( const QPixmap & );
    void resizeEvent(QResizeEvent *);
private:
    QPixmap pix;
};

#endif // IMAGE_WIDGET_H

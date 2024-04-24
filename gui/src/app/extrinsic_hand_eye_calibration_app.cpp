#include <industrial_calibration/gui/extrinsic_hand_eye_calibration_widget.h>
#include <QApplication>
#include <QMainWindow>
#include <signal.h>

void handleSignal(int /*sig*/) { QApplication::instance()->quit(); }

int main(int argc, char** argv)
{
  QApplication app(argc, argv);

  signal(SIGINT, handleSignal);
  signal(SIGTERM, handleSignal);

  // Create the main window
  QMainWindow w;

  // Create the IC widget
  auto* widget = new industrial_calibration::ExtrinsicHandEyeCalibrationWidget(&w);
  w.setWindowTitle("");

  // Set the IC widget as the central widget and show
  w.setCentralWidget(widget);
  w.showMaximized();

  return app.exec();
}
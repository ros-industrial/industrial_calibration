#include <industrial_calibration/gui/camera_intrinsic_calibration_widget.h>
#include <QApplication>
#include <signal.h>

void handleSignal(int /*sig*/) { QApplication::instance()->quit(); }

int main(int argc, char** argv)
{
  QApplication app(argc, argv);

  signal(SIGINT, handleSignal);
  signal(SIGTERM, handleSignal);

  // Create the calibration main widget
  industrial_calibration::CameraIntrinsicCalibrationWidget w;
  w.setWindowTitle("Camera Intrinsic Calibration");
  w.setWindowIcon(QIcon(":/icons/icon.jpg"));
  w.showMaximized();

  // Attempt to load configuration and observations files if available
  if (argc > 1) w.loadConfig(argv[1]);
  if (argc > 2) w.loadObservations(argv[2]);

  return app.exec();
}

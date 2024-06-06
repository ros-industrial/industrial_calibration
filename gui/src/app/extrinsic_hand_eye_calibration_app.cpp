#include <industrial_calibration/gui/extrinsic_hand_eye_calibration_widget.h>
#include <QApplication>
#include <signal.h>

void handleSignal(int /*sig*/) { QApplication::instance()->quit(); }

int main(int argc, char** argv)
{
  QApplication app(argc, argv);

  signal(SIGINT, handleSignal);
  signal(SIGTERM, handleSignal);

  // Create the calibration main widget
  industrial_calibration::ExtrinsicHandEyeCalibrationWidget w;
  w.setWindowTitle("Extrinsic Hand Eye Calibration");
  w.setWindowIcon(QIcon(":/icons/icon.jpg"));
  w.showMaximized();

  return app.exec();
}

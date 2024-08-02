#include <industrial_calibration/gui/camera_intrinsic_calibration_widget.h>
#include <QApplication>
#include <QMessageBox>
#include <QTextStream>
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

  /* Attempt to run headless if all files are specified:
   *   argv[1]: configuration file
   *   argv[2]: observation file
   *   argv[3]: results file (industrial_calibration)
   *   argv[4]: results file (ROS) - optional
   */
  if (argc > 3)
  {
    try
    {
      w.loadConfig(argv[1]);
      w.loadObservations(argv[2]);
      w.calibrate();
      w.saveResults(argv[3]);
      if (argc > 4)
        w.saveROSFormat(argv[4]);

      QMessageBox::StandardButton ret = QMessageBox::question(nullptr, "Calibration",
                                                              "Successfully completed calibration and saved results. "
                                                              "View results in the GUI?");
      if (ret == QMessageBox::StandardButton::Yes)
      {
        w.showMaximized();
        return app.exec();
      }
    }
    catch (const std::exception& ex)
    {
      QString question;
      QTextStream ss(&question);
      ss << "Error: " << ex.what() << "\n\nOpen GUI to fix?";

      QMessageBox::StandardButton ret = QMessageBox::question(nullptr, "Error", question);
      if (ret == QMessageBox::StandardButton::Yes)
      {
        w.showMaximized();
        return app.exec();
      }
    }
  }
  else
  {
    w.showMaximized();

    // Attempt to load configuration and observations files if available
    try
    {
      if (argc > 1)
      {
        w.loadConfig(argv[1]);
        QMessageBox::information(nullptr, "Configuration", "Successfully loaded calibration configuration");
      }
      if (argc > 2)
        w.loadObservations(argv[2]);
    }
    catch (const std::exception& ex)
    {
      QMessageBox::warning(nullptr, "Error", ex.what());
    }

    return app.exec();
  }

  return 0;
}

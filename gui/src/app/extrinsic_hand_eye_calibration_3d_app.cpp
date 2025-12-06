#include <industrial_calibration/gui/extrinsic_hand_eye_calibration_3d_widget.h>
#include <filesystem>
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
  industrial_calibration::ExtrinsicHandEyeCalibration3DWidget w;
  w.setWindowTitle("Extrinsic Hand Eye Calibration 3D");
  w.setWindowIcon(QIcon(":/icons/icon.jpg"));

  // Attempt to run headless if the configuration file (argv[1]), observation file (argv[2]), and results file (argv[3])
  // are specified
  if (argc > 3)
  {
    try
    {
      w.loadConfig(argv[1]);
      w.loadObservations(argv[2]);
      w.calibrate();
      w.saveResults(argv[3]);
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
        w.show();
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
      if (argc > 1 && std::filesystem::is_regular_file(argv[1]))
      {
        w.loadConfig(argv[1]);
        QMessageBox::information(nullptr, "Configuration", "Successfully loaded calibration configuration");
      }
      if (argc > 2 && std::filesystem::is_regular_file(argv[2]))
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

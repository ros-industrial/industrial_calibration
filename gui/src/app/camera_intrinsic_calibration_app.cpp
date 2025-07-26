#include <industrial_calibration/gui/camera_intrinsic_calibration_widget.h>
#include <industrial_calibration/gui/aspect_ratio_pixmap_label.h>

#include <filesystem>
#include <QApplication>
#include <QDockWidget>
#include <QIcon>
#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QTextStream>
#include <QToolBar>
#include <signal.h>

void handleSignal(int /*sig*/) { QApplication::instance()->quit(); }

int main(int argc, char** argv)
{
  QApplication app(argc, argv);

  signal(SIGINT, handleSignal);
  signal(SIGTERM, handleSignal);

  // Create the main window
  auto window = new QMainWindow();
  window->setWindowTitle("Camera Intrinsic Calibration");
  window->setWindowIcon(QIcon(":/icons/icon.jpg"));

  // Create an image label as the main display
  auto image = new industrial_calibration::AspectRatioPixmapLabel(window);
  const std::string instructions = R"(
<html>
<head/>
<body>
<h3>Camera Intrinsic Calibration</h3>
<p>
  Configure the calibration, either from a YAML file (<b><i>Load Configuration</i></b> button) or manually using the icons
</p>
<p>
  Load the calibration observations from a YAML file using the <b><i>Load Observations</i></b> button
</p>
<p>
  Click on an observation in the list to view the image and detected target
</p>
<p>
Click the <b><i>Calibrate</i></b> button to perform the calibration
</p>
<p>
  Click the <b><i>Save</i></b> button to save the calibration results to file
</p>
</body>
</html>
)";
  image->setText(QString::fromStdString(instructions));
  image->setAlignment(Qt::AlignCenter);
  window->setCentralWidget(image);
  window->setStyleSheet(QString("QMainWindow::separator { background-color: %1; width: 3px; height: 3px; }")
                            .arg(app.palette().color(QPalette::Midlight).name()));

  // Create the calibration widget as a docked widget
  auto cal = new industrial_calibration::CameraIntrinsicCalibrationWidget(window);
  cal->action_use_extrinsic_guesses->setChecked(true);
  auto dock = new QDockWidget("Calibration", window);
  dock->setStyleSheet(
      QString("QDockWidget::title { background-color: %1; }").arg(app.palette().color(QPalette::Midlight).name()));
  dock->setFeatures(QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);
  dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  dock->setWidget(cal);
  window->addDockWidget(Qt::LeftDockWidgetArea, dock);

  QApplication::connect(cal, &industrial_calibration::CameraIntrinsicCalibrationWidget::imageSelected, image,
                        &industrial_calibration::AspectRatioPixmapLabel::setPixmap);

  // Add an instructions action to the tool bar
  auto action_instructions = new QAction("Instructions", window);
  action_instructions->setToolTip("Click for instructions on running the calibration");
  action_instructions->setIcon(QIcon::fromTheme("dialog-information"));
  QApplication::connect(action_instructions, &QAction::triggered, [instructions]() {
    QMessageBox::information(nullptr, "Instructions", QString::fromStdString(instructions));
  });

  // Set up the tool bar
  {
    QToolBar* tool_bar = window->addToolBar("Tools");
    tool_bar->addAction(action_instructions);
    tool_bar->addSeparator();
    tool_bar->addAction(cal->action_load_observations);
    tool_bar->addAction(cal->action_load_configuration);
    tool_bar->addSeparator();
    tool_bar->addAction(cal->action_edit_target_finder);
    tool_bar->addAction(cal->action_edit_camera_intrinsics);
    tool_bar->addAction(cal->action_use_extrinsic_guesses);
    tool_bar->addAction(cal->action_use_opencv);
    tool_bar->addSeparator();
    tool_bar->addAction(cal->action_calibrate);
    tool_bar->addAction(cal->action_save);
    tool_bar->addAction(cal->action_save_ros_format);
  }

  // Set up the menu bar
  {
    QMenu* menu_file = window->menuBar()->addMenu("File");
    menu_file->addAction(cal->action_load_observations);
    menu_file->addAction(cal->action_load_configuration);
    menu_file->addSeparator();
    menu_file->addAction(cal->action_save);
    menu_file->addAction(cal->action_save_ros_format);
  }
  {
    QMenu* menu_edit = window->menuBar()->addMenu("Edit");
    menu_edit->addAction(cal->action_edit_target_finder);
    menu_edit->addAction(cal->action_edit_camera_intrinsics);
    menu_edit->addAction(cal->action_use_extrinsic_guesses);
    menu_edit->addAction(cal->action_use_opencv);
  }
  {
    QMenu* menu_calibrate = window->menuBar()->addMenu("Calibrate");
    menu_calibrate->addAction(cal->action_calibrate);
  }
  {
    QMenu* menu_help = window->menuBar()->addMenu("Help");
    menu_help->addAction(action_instructions);
  }

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
      cal->loadConfig(argv[1]);
      cal->loadObservations(argv[2]);
      cal->calibrate();
      cal->saveResults(argv[3]);
      if (argc > 4)
        cal->saveROSFormat(argv[4]);

      QMessageBox::StandardButton ret = QMessageBox::question(nullptr, "Calibration",
                                                              "Successfully completed calibration and saved results. "
                                                              "View results in the GUI?");
      if (ret == QMessageBox::StandardButton::Yes)
      {
        window->showMaximized();
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
        window->showMaximized();
        return app.exec();
      }
    }
  }
  else
  {
    window->showMaximized();

    // Attempt to load configuration and observations files if available
    try
    {
      if (argc > 1 && std::filesystem::is_regular_file(argv[1]))
      {
        cal->loadConfig(argv[1]);
        QMessageBox::information(nullptr, "Configuration", "Successfully loaded calibration configuration");
      }
      if (argc > 2 && std::filesystem::is_regular_file(argv[2]))
        cal->loadObservations(argv[2]);
    }
    catch (const std::exception& ex)
    {
      QMessageBox::warning(nullptr, "Error", ex.what());
    }

    return app.exec();
  }

  return 0;
}

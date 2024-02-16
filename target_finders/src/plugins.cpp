#include <industrial_calibration/target_finders/aruco_grid_target_finder.h>
#include <industrial_calibration/target_finders/charuco_grid_target_finder.h>
#include <industrial_calibration/target_finders/modified_circle_grid_target_finder.h>

#include <boost_plugin_loader/macros.h>
#define EXPORT_TARGET_FINDER_PLUGIN(DERIVED_CLASS, ALIAS)                                                              \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, TARGET_FINDER_SECTION)

EXPORT_TARGET_FINDER_PLUGIN(industrial_calibration::ArucoGridTargetFinderFactory, ArucoGridTargetFinder)
EXPORT_TARGET_FINDER_PLUGIN(industrial_calibration::CharucoGridTargetFinderFactory, CharucoGridTargetFinder)
EXPORT_TARGET_FINDER_PLUGIN(industrial_calibration::ModifiedCircleGridTargetFinderFactory,
                            ModifiedCircleGridTargetFinder)

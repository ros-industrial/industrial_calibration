"""
   Copyright 2014 Southwest Research Institute

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
"""

import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from std_srvs.srv import Empty

class CalibrationControl(Plugin):

    def __init__(self, context):
        super(CalibrationControl, self).__init__(context)
        self.setObjectName('CalibrationControl')
        
        
        # Argument parsing
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('industrial_calibration_gui'), 'resource', 'calibration_control.ui')
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('calibration_control_Ui')
        
        # Custom code begins here
        self._widget.cal_button.clicked[bool].connect(self.__handle_cal_clicked)
        
        self.cal_service = rospy.ServiceProxy('calibration_service', Empty)
        
        # Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
        
        
    def __handle_cal_clicked(self, checked):
        self.cal_service()
        
        

/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef ROBOT_ICAL_WIDGET_H
#define ROBOT_ICAL_WIDGET_H

#include <QWidget>

namespace intrinsic_cal
{

class RobotIcalWidget: public QWidget
{
Q_OBJECT
public:
  // This class is not instantiated by pluginlib::ClassLoader, so the
  // constructor has no restrictions.
  RobotIcalWidget( QWidget* parent = 0 );

  // add overrides to whatever is necessary
  // e.g.  virtual void mouseMoveEvent( QMouseEvent* event );

  // Override sizeHint() to give the layout managers some idea of a good size for the widget
  virtual QSize sizeHint() const { return QSize( 150, 150 ); }

  private Q_SLOTS:
    void on_CalibrateButton_Clicked();
    void on_AcceptButton_Clicked();
    
Q_SIGNALS:
  // public Emitted signals 
  // e.g.  void outputVelocity( float linear, float angular );

protected:
  // privat Emitted signals 
  // e.g. void sendVelocitiesFromMouse( int x, int y, int width, int height );


  // Finally the member variables:
  float linear_velocity_; // In m/s
  float angular_velocity_; // In radians/s
  float linear_scale_; // In m/s
  float angular_scale_; // In radians/s
};
// END_TUTORIAL

} // end namespace intrinsic_cal


#endif // ROBOT_ICAL_WIDGET_H

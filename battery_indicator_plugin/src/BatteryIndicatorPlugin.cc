/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/


#include "BatteryIndicatorPlugin.hh"
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(BatteryIndicatorPlugin)

BatteryIndicatorPlugin::BatteryIndicatorPlugin()
{
        gzwarn << "Created BatteryIndicatorPlugin\n";

}

/////////////////////////////////////////////////
BatteryIndicatorPlugin::~BatteryIndicatorPlugin()
{
}

void BatteryIndicatorPlugin::Load(rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    gzwarn << "Battery Indicator Loading\n";
    this->modelviz = _parent;
    this->updateConnection = event::Events::ConnectPreRender(boost::bind(&BatteryIndicatorPlugin::OnUpdate, this));
}

// Called by the world update start event
void BatteryIndicatorPlugin::OnUpdate()
{
    static float r=0.0, increment=0.01;
    if (r > 1.0) {
        increment = -0.01;
        r = 1.0;
    }
    if (r < 0.0) {
        r = 0.0;
        increment = 0.01;
    }
    //
    common::Color c(r, 0.0, 0.0);
    this->modelviz->SetAmbient(c);
    this->modelviz->SetDiffuse(c);
    //
    r += increment;
    gzmsg << r << std::endl;
}

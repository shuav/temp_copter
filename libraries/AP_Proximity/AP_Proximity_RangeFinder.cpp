/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_RangeFinder.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>
#include <AP_RangeFinder/RangeFinder_Backend.h>

extern const AP_HAL::HAL& hal;

AP_Proximity_RangeFinder::AP_Proximity_RangeFinder(AP_Proximity &_frontend,
                                   AP_Proximity::Proximity_State &_state) :
    AP_Proximity_Backend(_frontend, _state),
    _distance_upward(-1)
{
}


/**************************************************************************************************************
*函数原型：void AP_Proximity::update(void)
*函数功能：更新近距离传感器
*修改日期：2019-2-18
*修改作者：cihang_uav
*备注信息：update the state of the sensor
****************************************************************************************************************/

void AP_Proximity_RangeFinder::update(void)
{
    //立即退出，如果没有测距目标------ exit immediately if no rangefinder object
    const RangeFinder *rngfnd = frontend.get_rangefinder();
    if (rngfnd == nullptr)
    {
        set_status(AP_Proximity::Proximity_NoData);
        return;
    }

    uint32_t now = AP_HAL::millis();

    //寻找所有的测距仪数据------ look through all rangefinders
    for (uint8_t i=0; i < rngfnd->num_sensors(); i++)
    {
        AP_RangeFinder_Backend *sensor = rngfnd->get_backend(i);
        if (sensor == nullptr)
        {
            continue;
        }
        if (sensor->has_data())
        {
            // check for horizontal range finders
            if (sensor->orientation() <= ROTATION_YAW_315)
            {
                uint8_t sector = (uint8_t)sensor->orientation();    //区域
                _angle[sector] = sector * 45;                       //角度
                _distance[sector] = sensor->distance_cm() / 100.0f; //距离
                _distance_min = sensor->min_distance_cm() / 100.0f; //最小距离
                _distance_max = sensor->max_distance_cm() / 100.0f; //最大距离
                _distance_valid[sector] = (_distance[sector] >= _distance_min) && (_distance[sector] <= _distance_max); //判断距离是有效的吗？
                _last_update_ms = now;
                update_boundary_for_sector(sector);   //更新边界区域
            }
            // check upward facing range finder
            if (sensor->orientation() == ROTATION_PITCH_90)
            {
                int16_t distance_upward = sensor->distance_cm();
                int16_t up_distance_min = sensor->min_distance_cm();
                int16_t up_distance_max = sensor->max_distance_cm();
                if ((distance_upward >= up_distance_min) && (distance_upward <= up_distance_max))
                {
                    _distance_upward = distance_upward * 1e2;
                } else
                {
                    _distance_upward = -1.0; // mark an valid reading
                }
                _last_upward_update_ms = now;
            }
        }
    }

    // check for timeout and set health status
    if ((_last_update_ms == 0) || (now - _last_update_ms > PROXIMITY_RANGEFIDER_TIMEOUT_MS))
    {
        set_status(AP_Proximity::Proximity_NoData);
    } else
    {
        set_status(AP_Proximity::Proximity_Good);
    }
}

/**************************************************************************************************************
*函数原型：bool AP_Proximity_RangeFinder::get_upward_distance(float &distance) const
*函数功能：更新近距离传感器
*修改日期：2019-2-18
*修改作者：cihang_uav
*备注信息：get distance upwards in meters. returns true on success
****************************************************************************************************************/

bool AP_Proximity_RangeFinder::get_upward_distance(float &distance) const
{
    if ((AP_HAL::millis() - _last_upward_update_ms <= PROXIMITY_RANGEFIDER_TIMEOUT_MS) &&
        is_positive(_distance_upward)) {
        distance = _distance_upward;
        return true;
    }
    return false;
}

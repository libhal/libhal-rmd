// Copyright 2024 Khalil Estell
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>

#include <libhal-rmd/drc.hpp>

namespace hal::rmd {
drc_servo::drc_servo(drc& p_drc, hal::rpm p_max_speed)
  : m_drc(&p_drc)
  , m_max_speed(p_max_speed)
{
}

void drc_servo::driver_position(hal::degrees p_position)
{
  m_drc->position_control(p_position, m_max_speed);
}

drc_temperature_sensor::drc_temperature_sensor(drc& p_drc)
  : m_drc(&p_drc)
{
}

hal::celsius drc_temperature_sensor::driver_read()
{
  m_drc->feedback_request(hal::rmd::drc::read::status_2);
  return m_drc->feedback().temperature();
}

drc_rotation_sensor::drc_rotation_sensor(drc& p_drc)
  : m_drc(&p_drc)
{
}

hal::rotation_sensor::read_t drc_rotation_sensor::driver_read()
{
  m_drc->feedback_request(hal::rmd::drc::read::multi_turns_angle);
  return { .angle = m_drc->feedback().angle() };
}

rmd::drc_rotation_sensor make_rotation_sensor(rmd::drc& p_drc)
{
  return { p_drc };
}

rmd::drc_servo make_servo(rmd::drc& p_drc, hal::rpm p_max_speed)
{
  return { p_drc, std::abs(p_max_speed) };
}

rmd::drc_temperature_sensor make_temperature_sensor(rmd::drc& p_drc)
{
  return { p_drc };
}

drc_motor::drc_motor(rmd::drc& p_drc, hal::rpm p_max_speed)
  : m_drc(&p_drc)
  , m_max_speed(p_max_speed)
{
}

void drc_motor::driver_power(float p_power)
{
  m_drc->velocity_control(m_max_speed * p_power);
}

drc_motor make_motor(rmd::drc& p_drc, hal::rpm p_max_speed)
{
  return { p_drc, std::abs(p_max_speed) };
}

int make_servo(hal::rpm p_max_speed)
{
  return static_cast<int>(5 * p_max_speed);
}

drc_angular_velocity_sensor::drc_angular_velocity_sensor(drc& p_drc)
  : m_drc(&p_drc)
{
}

hal::rpm drc_angular_velocity_sensor::driver_read()
{
  m_drc->feedback_request(drc::read::status_2);
  return m_drc->feedback().speed();
}

drc_angular_velocity_sensor make_angular_velocity_sensor(drc& p_drc)
{
  return { p_drc };
}
}  // namespace hal::rmd
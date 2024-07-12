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

#include <libhal-exceptions/control.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>

#include "hardware_map.hpp"

hardware_map_t hardware_map{};

[[noreturn]] void terminate_handler() noexcept
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  while (true) {
    hardware_map.status_led->level(false);
    hal::delay(*hardware_map.clock, 100ms);
    hardware_map.status_led->level(true);
    hal::delay(*hardware_map.clock, 100ms);
    hardware_map.status_led->level(false);
    hal::delay(*hardware_map.clock, 100ms);
    hardware_map.status_led->level(true);
    hal::delay(*hardware_map.clock, 1000ms);
  }
}

int main()
{
  try {
    hardware_map = initialize_platform();
  } catch (...) {
    hal::halt();
  }

  hal::set_terminate(terminate_handler);

  application(hardware_map);
  hardware_map.reset();
  return 0;
}

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

#include <libhal-rmd/drc.hpp>

class do_nothing_can : public hal::can
{
public:
  void driver_configure(const settings&) override
  {
  }

  void driver_bus_on() override
  {
  }

  void driver_send(const message_t&) override
  {
  }

  void driver_on_receive(hal::callback<handler>) override
  {
  }
};

class do_nothing_steady_clock : public hal::steady_clock
{
public:
  hal::hertz driver_frequency() override
  {
    return 10'000'000.0f;
  }
  std::uint64_t driver_uptime() override
  {
    return m_counter++;
  }
  std::uint64_t m_counter = 0;
};

volatile bool run = false;
int main()
{
  if (run) {
    do_nothing_can can;
    do_nothing_steady_clock steady_clock;
    hal::can_router router(can);
    [[maybe_unused]] hal::rmd::drc servo(router, steady_clock, 6.0f, 0x140);
  }
  return 0;
}

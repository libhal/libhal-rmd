# Copyright 2024 Khalil Estell
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from conan import ConanFile

required_conan_version = ">=2.0.14"


class libhal_rmd_conan(ConanFile):
    name = "libhal-rmd"
    license = "Apache-2.0"
    homepage = "https://libhal.github.io/libhal-rmd"
    description = (
        "A collection of drivers for the RMD series servos and actuators")
    topics = ("servo", "smart servo", "PID")
    settings = "compiler", "os", "build_type", "arch"

    python_requires = "libhal-bootstrap/[^1.0.0]"
    python_requires_extend = "libhal-bootstrap.library"

    def requirements(self):
        bootstrap = self.python_requires["libhal-bootstrap"]
        bootstrap.module.add_library_requirements(self)
        self.requires("libhal-canrouter/[^2.0.0]")

    def package_info(self):
        self.cpp_info.libs = ["libhal-rmd"]
        self.cpp_info.set_property("cmake_target_name", "libhal::rmd")

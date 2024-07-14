# libhal-rmd

[![✅ Checks](https://github.com/libhal/libhal-rmd/actions/workflows/ci.yml/badge.svg)](https://github.com/libhal/libhal-rmd/actions/workflows/ci.yml)
[![GitHub stars](https://img.shields.io/github/stars/libhal/libhal-rmd.svg)](https://github.com/libhal/libhal-rmd/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/libhal/libhal-rmd.svg)](https://github.com/libhal/libhal-rmd/network)
[![GitHub issues](https://img.shields.io/github/issues/libhal/libhal-rmd.svg)](https://github.com/libhal/libhal-rmd/issues)

libhal device library for the series of the RMD smart motors from
[MyActuator](https://www.myactuator.com/).

## 📚 Software APIs & Usage

Take a look at the
[`include/libhal-rmd`](https://github.com/libhal/libhal-rmd/tree/main/include/libhal-rmd)
directory.

To see how each driver is used see the
[`demos/`](https://github.com/libhal/libhal-rmd/tree/main/demos) directory.

## 🧰 Setup

Following the
[🚀 Getting Started](https://libhal.github.io/2.1/getting_started/)
instructions.

## 📡 Installing Profiles

The `libhal-lpc40` profiles used for demos. To install them use the following
commands.

```bash
conan config install -tf profiles -sf conan/profiles/v1 https://github.com/libhal/arm-gnu-toolchain.git
conan config install -sf conan/profiles/v2 -tf profiles https://github.com/libhal/libhal-lpc40.git
```

## 🏗️ Building Demos

To build demos, start at the root of the repo and execute the following command:

```bash
conan build demos -pr lpc4078 -pr arm-gcc-12.3
```

or for the `lpc4074`

```bash
conan build demos -pr lpc4074 -pr arm-gcc-12.3
```

## 🔌 Device Wiring & Hookup guide (CAN BUS)

1. Locate the CANTD (CAN Transmit Data) and CANRD (Can Receive Data) pins on
   your microcontroller port.
2. Connect CANTD and CANRD lines to a CAN transceiver.
3. Connect CAN transceiver's CANL and CANH lines and connect them to an motor's
   CANL and CANH lines.
4. Supply adequate power to the CAN transceiver and the smart motor.

## 📦 Adding `libhal-rmd` to your project

Add the following to your `requirements()` method:

```python
    def requirements(self):
        self.requires("libhal-rmd/[^5.0.0]")
```

If you are using CMake make sure to first find the package then link it to your
binary.

```CMake
# Find & load package
find_package(libhal-rmd REQUIRED CONFIG)

# Link library to your binary
target_link_libraries(my_binary.elf PRIVATE libhal::rmd)
```

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for details.

## License

Apache 2.0; see [`LICENSE`](LICENSE) for details.

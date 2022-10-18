| Supported Targets | [DShanMCU-Mio](https://forums.100ask.net/c/esp/esp32s3/50) |
| ----------------- | ------------ |

---
<p align="right">
  <b>English</b> | <a href="./README_zh.md">中文</a></a>
</p>

# ESP-IDF components for the DShanMCU(ESP32)

### Need help or have a question? Join the forum at [![https://forums.100ask.net/c/esp/49](https://badges.gitter.im/espressif/arduino-esp32.svg)](https://forums.100ask.net/c/esp/49)

## Contents

  - [Documentation](#documentation)

### Add 100ask_components to the project

`100ask_components` has been embedded in the [esp-idf](https://github.com/100askTeam/esp-idf) repository maintained by 100ask as a submodule. It can be compiled and used directly according to the [document](https://esp32.100ask.net/#/DShanMCU-Mio/ESP-IDF/chapter2) without manually adding it to the components directory of its own project.

> If you use [esp-idf](https://github.com/100askTeam/esp-idf) maintained by 100ask, run `idf.py menuconfig` before compiling and then configure each component in `(Top) → Component config → 100ASK components`.

Method of manually adding to the components directory of your own project:

1. clone the 100ask_components:

```shell
git clone https://github.com/100askTeam/100ask_components.git
```

2. Then copy the component directory in 'AA100ask_componentsA/(the component you want)' to the **components** directory of your project.

3. Run `idf.py menuconfig` before compiling and then configure each component in `(Top) → Component config`.

### Documentation

  - [https://esp32.100ask.net](https://esp32.100ask.net)

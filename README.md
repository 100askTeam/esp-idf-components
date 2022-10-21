| Supported Targets | [DShanMCU-Mio](https://forums.100ask.net/c/esp/esp32s3/50) |
| ----------------- | ------------ |

---
<p align="right">
  <b>English</b> | <a href="./README_zh.md">中文</a></a>
</p>

# ESP-IDF components for the DShanMCU(ESP32)

### Need help or have a question? Join the forum at [![https://forums.100ask.net/c/esp/49](https://badges.gitter.im/espressif/arduino-esp32.svg)](https://forums.100ask.net/c/esp/49)

## Contents

  - [Add esp-100ask-components](#Add esp-100ask-components)
  - [Documentation](#documentation)

### Add esp-100ask-components

`esp-100ask-components ` has been embedded in the [esp-100ask-examples](https://github.com/100askTeam/esp-100ask-examples)  repository maintained by 100ask as a submodule. It can be compiled and used directly according to the [document](https://esp32.100ask.net/#/DShanMCU-Mio/ESP-IDF/chapter1) without manually adding it to the components directory of its own project.

> Run `idf.py menuconfig` before compiling and then configure each component in `(Top) → Component config → 100ASK components`.

Method of manually adding to the components directory of your own project:

1. clone the esp-100ask-components :

```shell
cd components
git clone https://github.com/100askTeam/esp-100ask-components.git
git submodule update --init --recursive
```

2. Run `idf.py menuconfig` before compiling and then configure each component in `(Top) → Component config → 100ASK components`.

### Documentation

  - [https://esp32.100ask.net](https://esp32.100ask.net)

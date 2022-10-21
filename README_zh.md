| 支持的硬件 | [DShanMCU-Mio(澪)](https://forums.100ask.net/c/esp/esp32s3/50) |
| ----------------- | ------------ |

---
<p align="right">
  <a href="../README.md">English</a>  |  <b>中文</b></a>
</p>

# 用于DShanMCU(ESP32)的ESP-IDF 组件

### 需要帮助或者有疑问吗？加入论坛 [![https://forums.100ask.net/c/esp/49](https://badges.gitter.im/espressif/arduino-esp32.svg)](https://forums.100ask.net/c/esp/49)

## 目录

  - [将 esp-100ask-components  添加到项目](#将 esp-100ask-components  添加到项目)
  - [学习文档](#学习文档)

### 将 esp-100ask-components  添加到项目

esp-100ask-components  已作为子模块嵌入到100ask维护的 [esp-100ask-examples](https://github.com/100askTeam/esp-100ask-examples) 中，按照 [文档](https://esp32.100ask.net/#/DShanMCU-Mio/ESP-IDF/chapter1) 进行即可直接编译使用而不需要手动添加到自己的项目的 components 目录中。

> 在编译前先运行 `idf.py menuconfig` 然后在 `(Top) → Component config → 100ASK components` 中对各个组件进行配置。


手动添加到自己的项目的 components 目录中的方法：

1. clone此仓库：

```shell
cd components
git clone https://github.com/100askTeam/esp-100ask-components.git
git submodule update --init --recursive
```

2. 编译前先运行 `idf.py menuconfig` 然后在 `(Top) → Component config → 100ASK components` 中对各个组件进行配置


### 学习文档

  - [https://esp32.100ask.net](https://esp32.100ask.net)

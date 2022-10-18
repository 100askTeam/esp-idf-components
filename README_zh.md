| 支持的硬件 | [DShanMCU-Mio(澪)](https://forums.100ask.net/c/esp/esp32s3/50) |
| ----------------- | ------------ |

---
<p align="right">
  <a href="../README.md">English</a>  |  <b>中文</b></a>
</p>

# 用于DShanMCU(ESP32)的ESP-IDF 组件

### 需要帮助或者有疑问吗？加入论坛 [![https://forums.100ask.net/c/esp/49](https://badges.gitter.im/espressif/arduino-esp32.svg)](https://forums.100ask.net/c/esp/49)

## 目录

  - [将 100ask_components 添加到项目](#将 100ask_components 添加到项目)
  - [学习文档](#学习文档)

### 将 100ask_components 添加到项目

100ask_components 已作为子模块嵌入到100ask维护的 [esp-idf](https://github.com/100askTeam/esp-idf) 中，按照 [文档](https://esp32.100ask.net/#/DShanMCU-Mio/ESP-IDF/chapter2) 进行即可直接编译使用而不需要手动添加到自己的项目的 components 目录中。

> 如果使用100ask维护的 [esp-idf](https://github.com/100askTeam/esp-idf)，那么在编译前先运行 `idf.py menuconfig` 然后在 `(Top) → Component config → 100ASK components` 中对各个组件进行配置。


手动添加到自己的项目的 components 目录中的方法：

1. clone此仓库：

```shell
git clone https://github.com/100askTeam/100ask_components.git
```

2. 之后将 `100ask_components/(你想要的组件)` 中的组件目录复制到你项目的 components 目录中
3. 编译前先运行 `idf.py menuconfig` 然后在中 `(Top) → Component config` 对各个组件进行配置


### 学习文档

  - [https://esp32.100ask.net](https://esp32.100ask.net)

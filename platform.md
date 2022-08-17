# platformio

## Init Project


### 查找Platform

平台即Platform通常与芯片对应，同一platform下通常有多种board

```bash
pio pkg search Seeed
```

### 查找Board

1. 通过board名称查找

```bash
platformio boards  |grep  nodemcu
```

2. 也可以限定平台后查找Board

```bash
pio boards espressif8266 |grep nodemcu
```


创建并初始化项目

```bash
mkdir sample
cd sample
pio project init --board seeed_wio_terminal
```

也可以使用--ide选项配置IDE，

```bash
pio project init --board seeed_wio_terminal --ide emacs
```

## 添加包

根据header文件搜索包：

pio lib search "header:WiFiManager.h"

使用结果列表里的ID安装包，例如

pio lib install  567


## normal operation

查看当前项目支持的操作

```bash
pio run --list-targets
```

清除固件

```bash
pio run --target erase
```

编译固件，

```bash
pio run
```

仅编译指定配置项

```bash
pio run -e nodemcu
```

编译成功后固件保存在.piio/build/[env]目录。对于ESP，可用于OTA的固件为.pio/build/[env]/firmware.bin。

编译并上传固件，连接设备后执行

```bash
pio run -t upload
```

## convert bin to uf2

```bash
git clone https://github.com/microsoft/uf2.git
```

for Wio Terminal

```bash
python3 uf2conv.py -f 0x55114460 -b 0x4000 firmwork.bin -o firmwork.uf2
```
# LexxPluss Main Board Software

## Install dependencies

https://docs.zephyrproject.org/latest/getting_started/
を参考に開発環境を準備する。

macOSだとたぶんこんな感じ。

### Install Homebrew

```bash
$ /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

### Install Utils

```bash
$ brew install cmake ninja gperf python3 ccache qemu dtc
$ pip3 install -U west
```

### Setup Zephyr

```bash
$ west init --manifest-url=https://github.com/LexxPluss/zephyr ~/zephyrproject
$ cd ~/zephyrproject
$ west update
$ west zephyr-export
$ pip3 install -r ~/zephyrproject/zephyr/scripts/requirements.txt
```

### Install Toolchain

https://docs.zephyrproject.org/latest/getting_started/toolchain_3rd_party_x_compilers.html#gnu-arm-embedded
を参考にToolchainをインストールして環境変数を設定する。

```bash
export ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb
export GNUARMEMB_TOOLCHAIN_PATH=/Applications/ARM
```

## Build

```bash
$ cd ~/zephyrproject/zephyr
$ west build -p auto -b lexxpluss_mb01 lexxpluss_apps
```

## Program

### STLINK Tools (Open souce version)

```bash
$ brew install stlink
$ st-flash --reset --connect-under-reset write build/zephyr/zephyr.bin  0x8000000
```


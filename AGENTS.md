# AGENTS.md

## プロジェクト概要

`atmega32u4-avrisp` は、ATmega32U4 搭載ボード（Arduino Leonardo 互換など）を AVR ISP（In-System Programmer）として動作させるファームウェアと、それを制御する PC 側ツールから構成されるプロジェクトです。

ファームウェアは Arduino ISP 互換の STK500 プロトコルで動作し、USB CDC-ACM 経由で PC と通信します。さらに、ビットバンギングモードを備えており、PC 側から各 GPIO ピンを直接制御できます。

## ディレクトリ構成

```
.
├── .clang-format              # フォーマット設定（WebKit ベース、タブ使用）
├── .gitignore
├── AGENTS.md                  # 本ファイル
├── bitbang/                   # PC 側制御ツール
│   ├── bitbang.pro            # Qt Creator プロジェクトファイル
│   ├── main.cpp               # ビットバンギングデモ
│   ├── serial.cpp             # シリアル通信実装
│   └── serial.h               # シリアル通信ヘッダ
└── firmware/                  # ATmega32U4 用ファームウェア
    ├── Makefile               # avr-gcc 用ビルド設定
    ├── atmega32u4-avrisp.pro  # Qt Creator プロジェクトファイル
    ├── main.cpp               # AVRISP + ビットバンギング本体
    ├── usb.c / usb.h          # USB CDC-ACM スタック
    ├── waitloop.cpp           # ソフトウェア遅延
    └── waitloop.h
```

## ファームウェア（`firmware/`）

### 対象ハードウェア

- MCU: ATmega32U4
- クロック: 16 MHz
- USB VID/PID: `0x16c0` / `0x05e1`
- 製品名: `Arduino ISP compatible`

### ピン割り当て

| ピン名      | ポート | 機能               |
|-------------|--------|--------------------|
| RESET       | PB6    | ターゲット RESET   |
| SCK         | PB1    | ターゲット SCK     |
| MOSI        | PB2    | ターゲット MOSI    |
| MISO        | PB3    | ターゲット MISO    |
| LED_PMODE   | PB0    | 書き込みモード LED |
| LED_ERROR   | PD5    | エラー LED         |

### 動作モード

ファームウェアは 2 つの動作モードを持ちます。

1. **AVRISP モード**
   - STK500 準拠のシリアルプログラマとして動作
   - ハードウェア SPI を使用（`USE_HARDWARE_SPI` 有効時）
   - デフォルト SPI クロック: F_CPU/128 = 125 kHz

2. **BITBANG モード**
   - PC 側から `.BITBANG.` コマンドを受信すると切り替わる
   - RESET/SCK/MOSI/MISO ピンを個別に入出力制御可能
   - 下位 4 ビットでピン番号、上位 4 ビットで操作を指定

### ビルド

```bash
cd firmware
make
```

- `main.hex` が生成されます。
- 書き込み例（Makefile 内 `write` ターゲット、Linux 想定）:
  ```bash
  avrdude -c avrisp -P /dev/ttyUSB0 -b 19200 -p m32u4 \
    -U hfuse:w:0xd9:m -U lfuse:w:0x5e:m -U flash:w:main.hex
  ```

## PC 側ツール（`bitbang/`）

### 概要

`bitbang` は Windows/Linux に対応したコンソールアプリケーションです。シリアルポート経由でファームウェアの BITBANG モードに切り替え、ターゲットの RESET ピンを点滅させるデモ動作を行います。

### 接続設定

- ポート:
  - Windows: `\\.\COM4`
  - Linux: `/dev/ttyACM0`
- ボーレート: 115200 bps

### `Serial` クラス

`bitbang/serial.h` / `bitbang/serial.cpp` で定義されています。

```cpp
class Serial {
public:
    struct Option {
        std::string port;
        int speed;
    };

    Serial();
    bool open(Option *option);
    void cancel();
    void close();
    int write(const void *ptr, int len);
    int read(void *ptr, int len, int timeout = -1);
};
```

- `timeout` はミリ秒単位。
  - `< 0` : 無限待ち
  - `== 0` : 即座に返す（ノンブロッキング）
  - `> 0` : 指定ミリ秒でタイムアウト
- エラー時は `-1` を返します。

### `Connection` クラス

`Serial` の薄いラッパーです。`main.cpp` 内で使用されています。`open`/`close`/`read`/`write` をそのまま委譲します。

### BITBANG コマンド

`main.cpp` 内で使用されるコマンドバイトは以下の通りです。

| マクロ              | 値     | 動作                          |
|---------------------|--------|-------------------------------|
| `CMD_READ`          | `0x80` | 指定ピンを入力に設定して読む  |
| `CMD_READ_PULLUP`   | `0x90` | プルアップ付き入力で読む      |
| `CMD_WRITE_LOW`     | `0xa0` | 指定ピンを出力 LOW に設定     |
| `CMD_WRITE_HIGH`    | `0xb0` | 指定ピンを出力 HIGH に設定    |

ピン番号は下位 4 ビットで指定します。

| マクロ     | 値 |
|------------|-----|
| `PIN_RST`  | 0   |
| `PIN_SCK`  | 1   |
| `PIN_MOSI` | 2   |
| `PIN_MISO` | 3   |

### ビルド

#### MSVC（Windows）

Visual Studio 2022 の開発者コマンドプロンプトなどで:

```cmd
cd bitbang
cl.exe /EHsc /std:c++17 /W4 /Febitbang.exe main.cpp serial.cpp /link /SUBSYSTEM:CONSOLE
```

#### Qt Creator

`bitbang/bitbang.pro` を開いてビルドできます。

## コーディングスタイル

- `.clang-format` に基づく。
- ベース: WebKit スタイル
- インデント: タブ（TabWidth: 4）
- カラム制限: なし（ColumnLimit: 0）
- ポインタ/参照の位置: 右寄せ

## 注意点・既知の制限

- `firmware/waitloop.cpp` はファームウェア用のソフトウェア遅延実装ですが、`firmware/main.cpp` では主に `<util/delay.h>` の `_delay_ms()` / `_delay_us()` を直接使用しています。
- `firmware/Makefile` の `write` ターゲットは Linux 環境の `/dev/ttyUSB0` を想定しています。Windows 環境では別途 avrdude を実行してください。
- PC 側ツールの接続先ポートは `main.cpp` 内にハードコードされています。

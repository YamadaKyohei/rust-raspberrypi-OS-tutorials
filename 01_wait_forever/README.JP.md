# Tutorial 01 - 無限に待機する

## 概要

- プロジェクトの骨組みをセットアップします。
- カーネルコードを実行している全CPUコアを停止するだけの、小さなアセンブリコードを実行します。

## ビルド

- `Makefile` には以下のtargetsが設定されています:
    - `doc`: ドキュメントを生成します。
    - `qemu`: QEMUで `kernel` を実行します。
    - `clippy`
    - `clean`
    - `readelf`: `ELF`の出力を確認します。
    - `objdump`: assemblyの情報を確認します。
    - `nm`: symbolsの情報を確認します。

## 注目すべきコード

- `BSP`-特定の `link.ld` リンカースクリプト
    - アドレス `0x8_0000` をロードします。
    - `.text` セクションのみ
- `main.rs`: 重要な[inner attributes]:
    - `#![no_std]`, `#![no_main]`
- `boot.s`: 全コアを停止する`wfe` (Wait For Event)を実行する、アセンブリの `_start()` 関数。
- `#[panic_handler]`関数を定義してあげるとコンパイラが喜ぶので、（必ず）定義しましょう。
    - 一旦`unimplemented!()`としておきます。 この部分は使わないので取り除かれる予定です。

[inner attributes]: https://doc.rust-lang.org/reference/attributes.html

### テストする

プロジェクトフォルダ内でQEMUを起動すると、`wfe`でコアが回り続ける様子が見られます:

```console
$ make qemu
[...]
IN:
0x00080000:  d503205f  wfe
0x00080004:  17ffffff  b        #0x80000
```

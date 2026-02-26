<!--
種別: decisions
対象: 技術スタック選定
作成日: 2026-02-26
更新日: 2026-02-26
担当: AIエージェント
-->

# 技術スタック選定

## 概要

teleop-hand の技術スタックを選定する。ロボットハンドのリアルタイム制御と finger-tracker との通信を実現するための言語・ライブラリ・ハードウェアインターフェースを決定する。

## 設計判断

### 判断1: 言語 — C++

**問題**: メイン開発言語を何にするか

**選択肢**:
1. C++
2. Python
3. C++ + Python ハイブリッド

**決定**: C++

**理由**:
- controller_try が C++ で書かれており、移植・改修のベースとなる
- リアルタイム制御ループに必要な低レイテンシ・決定的実行時間を確保できる
- Contec PCI ボードのI/Oアクセス（`iopl`, `outl`, `inl`）が C/C++ で直接可能

**トレードオフ**:
- **利点**: リアルタイム性能、ハードウェアアクセス、controller_try との互換性
- **欠点**: 開発速度は Python に劣る、finger-tracker（Python）との通信にプロセス間通信が必要

### 判断2: ビルドシステム — CMake

**問題**: ビルドシステムを何にするか

**選択肢**:
1. CMake
2. Makefile
3. Meson

**決定**: CMake

**理由**:
- controller_try が CMake を使用
- ライブラリ依存関係の管理が容易
- IDE サポートが充実

**トレードオフ**:
- **利点**: controller_try との互換性、広く使われているため情報が豊富
- **欠点**: CMakeLists.txt の記述が冗長になりやすい

### 判断3: 線形代数 — Eigen

**問題**: 行列演算ライブラリを何にするか

**選択肢**:
1. Eigen
2. Armadillo
3. 自前実装

**決定**: Eigen

**理由**:
- controller_try で既に使用
- ヘッダオンリーで導入が容易
- ロボティクス分野で広く使われている

**トレードオフ**:
- **利点**: 高性能、ヘッダオンリー、広いコミュニティ
- **欠点**: コンパイル時間が長くなりやすい

### 判断4: GUI — ImGui + ImPlot + GLFW + OpenGL

**問題**: GUI フレームワークを何にするか

**選択肢**:
1. ImGui + ImPlot + GLFW + OpenGL（controller_try 踏襲）
2. Qt
3. OpenGL 直接描画のみ（ImGui なし）

**決定**: ImGui + ImPlot + GLFW + OpenGL（controller_try のGUI構造を踏襲）

**理由**:
- controller_try が ImGui（UI）+ ImPlot（グラフ）+ GLFW（ウィンドウ）+ OpenGL（描画）で実装済み
- ImGui はイミディエイトモードGUIで、制御系のライブ表示に適している
- ImPlot で時系列グラフを簡単に描画できる

**備考**: GUI スレッドは制御タイマーに同期しない非リアルタイムスレッド（OS 描画速度に依存、30〜60fps 程度）。制御ループ（10kHz）とは独立して動作する。

**teleop-hand の GUI 構成**:

```
┌─────────────────────────────────────────────┐
│ 状態表示                                      │
│  Mode: remote   compute: 12μs               │
│  θ_res=0.52rad  dθ=0.01rad/s  f_dis=0.05Nm  │
│  distance_mm=45.2mm  θ_cmd=0.48rad           │
├───────────────────────┬─────────────────────┤
│                       │                     │
│  グラフ (ImPlot)       │  finger-tracker     │
│  - θ_cmd vs θ_res     │  受信状態            │
│  - f_dis (推定外乱)    │  - distance_mm      │
│  - distance_mm        │  - パケット受信率     │
│                       │                     │
├───────────────────────┴─────────────────────┤
│ モード選択                                    │
│ [idle] [DA_check] [remote] [Record] [Bilateral] │
└─────────────────────────────────────────────┘
```

**controller_try からの変更点**:
- モード選択: 16 個 → 5 個（idle / DA_check / remote / Record / Bilateral）
- 関節表示: 16 関節 → 1 関節
- 追加: finger-tracker 受信データ表示（distance_mm、パケット受信率）
- 追加: θ_cmd vs θ_res のグラフ（指令追従の確認用）
- 削除: record_mode_selection（record_motion スレッドに統合）
- 削除: パラメータ入力フォーム（設定ファイルで管理）
- 変更: start/stop ボタン廃止。モードボタン直押しで即切替（idle = 停止）
- 削除: カメラ移動（W/A/S/D、3Dビュー不要）

**トレードオフ**:
- **利点**: controller_try のコードを流用可能、ImGui でウィジェット追加が容易、ImPlot でリアルタイムグラフ
- **欠点**: ImGui / ImPlot の依存が追加される（ただし controller_try に同梱済み）

### 判断5: ハードウェアI/O — Contec PCI ボード

**問題**: モータ制御のハードウェアインターフェースを何にするか

**選択肢**:
1. Contec PCI DA/カウンタボード（既存）
2. EtherCAT
3. USB DAQ

**決定**: Contec PCI DA/カウンタボード

**理由**:
- 研究室の既存ハードウェア資産
- controller_try でドライバコードが実装済み
- PCI バス経由の低レイテンシI/O

**トレードオフ**:
- **利点**: 既存資産活用、低レイテンシ、実績あるドライバコード
- **欠点**: PCI スロット搭載PCが必要、root権限が必要

### 判断6: 通信方式 — UDP ソケット

**問題**: finger-tracker（Python）と teleop-hand（C++）間の通信方式を何にするか

**決定**: UDP ソケット（localhost）

**理由**: 詳細は [ADR 002](./002-communication-protocol.md) を参照

### 判断7: 設定ファイル — JSON (Boost.PropertyTree)

**問題**: 設定ファイルの形式と読み込みライブラリを何にするか

**選択肢**:
1. JSON (Boost.PropertyTree)
2. YAML (yaml-cpp)
3. TOML

**決定**: JSON (Boost.PropertyTree)

**理由**:
- controller_try が JSON + Boost.PropertyTree を使用
- 関節パラメータの JSON ファイルがそのまま再利用可能

**トレードオフ**:
- **利点**: controller_try の設定ファイルとの互換性、追加依存なし
- **欠点**: JSON はコメントが書けない（必要ならキー名で代用、controller_try と同様）

## 技術スタック一覧

| レイヤー | 技術 | 用途 |
|---------|------|------|
| 言語 | C++ | メイン言語 |
| ビルド | CMake | ビルドシステム |
| 線形代数 | Eigen | 行列演算 |
| GUI | ImGui + ImPlot + GLFW + OpenGL | ライブ状態表示・グラフ・モード切替 |
| ハードウェアI/O | Contec PCI DA/カウンタ | モータ制御・エンコーダ |
| 設定 | JSON (Boost.PropertyTree) | 設定ファイル読み込み |
| 通信 | UDP ソケット (localhost) | finger-tracker 連携（[ADR 002](./002-communication-protocol.md)） |

## 関連ドキュメント

- [002-communication-protocol.md](./002-communication-protocol.md) — 通信方式の詳細（UDP ソケット）
- [003-robot-hand-specification.md](./003-robot-hand-specification.md) — ロボットハンド・モータ仕様
- [004-control-architecture.md](./004-control-architecture.md) — 制御アーキテクチャ
- [005-control-philosophy.md](./005-control-philosophy.md) — 制御思想
- [初期仕様書](../../archive/initial_plan.md)
- [CLAUDE.md](../../../CLAUDE.md)

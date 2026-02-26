<!--
種別: plans
対象: Phase 1-2 実装（基盤構築 + 通信・遠隔操作）
作成日: 2026-02-26
更新日: 2026-02-26
ステータス: active
担当: AIエージェント
-->

# Phase 1-2 実装計画

## 概要

controller_try から必要なコードを移植・整理し、finger-tracker の指間距離でロボットハンドを遠隔操作できる状態まで構築する。Phase 1（基盤構築: 単体モータ制御）と Phase 2（通信・遠隔操作: finger-tracker 連携）を一括で計画する。

## 現状分析

- 全モジュールが `not-started`
- ADR 001〜005 で設計判断は完了（技術スタック、通信方式、ロボットハンド仕様、制御アーキテクチャ、制御思想）
- controller_try のソースコード（src/ 約 5,300 行、inc/ 約 2,000 行 ※サードパーティ除く）が移植元として利用可能
- controller_try は Blue SCARA 14 関節向け。teleop-hand は 1 関節（指の開閉）のみ

## ゴール

- [ ] `sudo ./control` でプログラムが起動し、GUI が表示される
- [ ] idle モードで全出力 = 0 を確認
- [ ] DA_check モードでモータに定数出力が出ることを確認
- [ ] finger-tracker から UDP で指間距離を受信できる
- [ ] remote モードで finger-tracker の指の動きにロボットハンドが追従する
- [ ] Record モードで遠隔操作データが CSV に記録される

## スコープ

### 含む

- CMake ビルド環境の構築
- controller_try からの移植: config / hardware / 制御フレームワーク / signal_processing
- 制御モード: idle / DA_check / remote / Record
- comm モジュール（UDP 受信）
- 逆運動学（余弦定理）+ 指令値フィルタ + PD 制御 + DOB
- GUI（状態表示、モード切替、グラフ）
- record_motion スレッド（データ記録）

### 含まない

- Bilateral モード（Phase 3）
- EMS 力フィードバック（Phase 3）
- テストコード（実機動作確認を優先）
- finger-tracker 側の修正（UDP 送信は実装済み前提）

## タスクリスト

### Phase 1: 基盤構築

#### #001-01: CMake ビルド環境 + ディレクトリ構造

サードパーティライブラリの配置とビルドが通る状態を作る。

- `CMakeLists.txt` を作成（C++17, -Wall -Wextra -O3）
- `lib/` に ImGui, ImPlot, Eigen をコピー（controller_try の `lib/` から）
- `lib/CMakeLists.txt` で GUI ライブラリをビルド
- `src/main.cc` に最小限のエントリポイント（空 main）
- `cmake .. && make` でビルドが通ることを確認

対象ファイル:
- `CMakeLists.txt`（新規、controller_try ベース）
- `lib/`（controller_try から ImGui + ImPlot + Eigen をコピー）
- `src/main.cc`（新規、最小限）

依存: なし

#### #001-02: config モジュール — 設定読み込み基盤

JSON 設定ファイルの読み込みと関節パラメータ管理を移植する。

- `inc/json_helper.h` — Boost.PropertyTree による JSON 読み込み
- `inc/environment.h` — グローバル設定（system.json）
- `inc/joint.hpp` — 関節構造体（data / parameter / data_buf）
- `inc/joint_parameter_definition.h` — パラメータ enum（mass, k_p, k_v, g_dis 等）
- `inc/robot_system.h` — ロボット全体管理（joints 配列、モード管理）
- `inc/enum_helper.h` — enum ↔ string 変換
- `config/system.json` — sample_frequency 等
- `config/thread_config.json` — スレッド周波数設定
- `config/joints/4018_finger.json` — 関節パラメータ（controller_try から）

変更点（controller_try からの差分）:
- `joint_parameter_definition.h`: teleop-hand に不要なパラメータを整理
- `robot_system.h`: 14 関節 → 1 関節に簡略化
- `mode_definition.h`: 制御モード enum を idle / DA_check / remote / Record / Bilateral の 5 つに
- `config/`: `blue_scara/` → `joints/` にリネーム、`4018_finger.json` のみ

依存: #001-01

#### #001-03: hardware モジュール — Contec PCI ボード制御

DA ボード（電圧出力）とカウンタボード（エンコーダ読み取り）の移植。

- `inc/contec_da.h` — DA ボードクラス（open / write / flush）
- `inc/contec_counter.h` — カウンタボードクラス（open / read）
- `inc/pci_helper.h` — PCI デバイス探索
- `inc/reader.h` — 読み取りインターフェース（抽象基底）
- `inc/writer.h` — 書き込みインターフェース（抽象基底）
- `config/contec_da1.json` — DA ボード設定
- `config/contec_counter1.json` — カウンタボード設定

変更点:
- DA: 16ch → 1ch（必要なチャネルのみ）
- カウンタ: 6ch → 1ch
- ボード 2 枚目（da2, counter2）は不要

依存: #001-02

#### #001-04: 制御フレームワーク — スレッド管理 + 信号処理

マルチスレッド実行基盤と信号処理ライブラリの移植。

- `inc/signal_processing.h` — 擬似微分、DOB（変更なし）
- `inc/control_timer.h` + `src/control_timer.cc` — 高精度タイマー（変更なし）
- `inc/thread_definition.h` — スレッド型 enum（recv_command を追加）
- `inc/thread_controller.h` + `src/thread_controller.cc` — スレッド登録・実行ループ
- `inc/system_controller.h` + `src/system_controller.cc` — タスク登録（read_sensor / compute_engine / write_output / record_motion / recv_command）

変更点:
- `thread_definition.h`: `recv_command` スレッドを追加
- `system_controller`: `recv_command` タスクの登録、play_motion 削除
- read_sensor / compute_engine / write_output のタスク実装は controller_try から移植

依存: #001-02, #001-03

#### #001-05: controller — idle + DA_check モード

最小限の制御モード実装。

- `inc/controller.h` — コントローラインターフェース
- `src/controller.cc` — idle（出力 = 0）、DA_check（定数出力）の実装

controller_try の controller.cc（4,525 行）から idle と DA_check のみ抽出（各 10 行程度）。残りの 4,500 行は不要。remote と Record は Phase 2 で追加。

依存: #001-02, #001-04

#### #001-06: record_motion — データ記録スレッド

制御データを CSV ファイルに記録する機能の移植。

- `src/system_controller.cc` 内の record_motion タスク実装
- 記録対象: 時刻、θ_res、dθ_res、f_dis、f_out、θ_cmd、distance_mm
- 出力先: `data/` ディレクトリ

依存: #001-04

#### #001-07: GUI — teleop-hand 用画面

controller_try の GUI を teleop-hand 用に簡略化して移植。

- `inc/gui.h` + `src/gui.cc` — GUI メインクラス（ImGui 初期化、描画ループ）
- `inc/gui_widget.h` — ウィジェット定義（teleop-hand 用に新規作成）
- `inc/layout.h` + `config/layout.json` — グリッドレイアウト
- `src/load_shader.cc` + `config/opengl/` — シェーダー（変更なし）
- `config/fonts/` — フォント（controller_try から）

表示内容（ADR 001 で定義）:
- 状態表示: モード、θ_res、dθ_res、f_dis、distance_mm、θ_cmd
- グラフ: θ_cmd vs θ_res、f_dis の時系列（ImPlot）
- モード選択: [idle] [DA_check] [remote] [Record] [Bilateral]（直押しで即切替、idle = 停止）

依存: #001-04, #001-05

#### #001-08: Phase 1 動作確認

実機でモータが動くことを確認する。

- `src/main.cc` を完成（関節追加、PCI デバイス初期化、スレッド起動）
- `sudo ./control` で起動
- idle モードで出力 = 0 を確認
- DA_check モードでモータに定数電圧が出ることを確認
- GUI に状態が表示されることを確認
- record_motion でデータが CSV に出力されることを確認

依存: #001-01 〜 #001-07 すべて

### Phase 2: 通信・遠隔操作

#### #001-09: comm モジュール — UDP 受信

finger-tracker からの指間距離データを UDP で受信する。

- `inc/comm/finger_data.h` — finger_data 構造体（ADR 002 準拠、28 bytes）
- `inc/comm/udp_receiver.h` + `src/comm/udp_receiver.cc` — UDP ソケット受信クラス
  - ソケット作成・バインド（ポート番号は config で指定）
  - ノンブロッキング受信（recvfrom + MSG_DONTWAIT）
  - 最新データを共有変数に書き込み（std::atomic または軽量ロック）
  - パケット未着時は直前値を保持
  - 無効データ判定（distance_mm < 0）
- `src/system_controller.cc` に recv_command タスク追加
- `config/comm.json` — UDP 設定（ポート番号: 50000）

依存: #001-04（スレッドフレームワーク）

#### #001-10: remote モード — 逆運動学 + PD 制御 + DOB

finger-tracker の指間距離でモータを位置制御する。

- `src/controller.cc` に remote モードと Record モードを追加
  - 逆運動学: `θ_cmd = acos(1 - x_d² / (2r²))`、r = 0.0725、クランプ処理
  - 指令値フィルタ: 1 次遅れ LPF（g_cmd = 60 rad/s）
  - PD 制御: `f_ref = M * (k_p * (θ_cmd_filtered - θ_res) + k_v * (0 - dθ_res))`
  - DOB 補償: `f_vol = f_ref + f_dis`
  - Record モード: remote と同じ制御則 + record_motion スレッドへの記録トリガ
- `config/remote.json` — remote モード設定（link_length, g_cmd）

依存: #001-05, #001-09

#### #001-11: GUI に finger-tracker 受信状態を追加

GUI に指間距離と受信状態の表示を追加する。

- `inc/gui_widget.h` に finger-tracker 表示ウィジェット追加
  - distance_mm の数値表示
  - パケット受信率の表示
  - distance_mm の時系列グラフ（ImPlot）
- θ_cmd vs θ_res の追従グラフを有効化

依存: #001-07, #001-09

#### #001-12: エンドツーエンド動作確認

finger-tracker → teleop-hand の全経路で遠隔操作が動作することを確認する。

- finger-tracker を起動し、指間距離の UDP 送信を確認
- teleop-hand の remote モードで指の動きにモータが追従することを確認
- GUI でθ_cmd vs θ_res のグラフが正常に表示されることを確認
- 各種パラメータ（k_p, k_v, g_cmd）の調整
- record_motion で制御データを記録し、応答を分析

依存: #001-08, #001-10, #001-11

## 依存関係図

```
Phase 1:
  #001-01 (CMake)
     │
  #001-02 (config)
     │
     ├── #001-03 (hardware)
     │      │
     │      ├── #001-04 (制御フレームワーク)
     │      │      │
     │      │      ├── #001-05 (controller: idle/DA_check) ──┐
     │      │      ├── #001-06 (record_motion)              │
     │      │      └── #001-07 (GUI) ←─────────────────────┘
     │      │
     └──────┴──────→ #001-08 (Phase 1 動作確認)

Phase 2:
  #001-04 ───→ #001-09 (comm: UDP受信)
                  │
  #001-05 ───→ #001-10 (remote モード)
                  │
  #001-07 ───→ #001-11 (GUI: finger-tracker表示)
                  │
  #001-08 ───→ #001-12 (エンドツーエンド動作確認)
```

## リスク

| リスク | 影響 | 対策 |
|--------|------|------|
| PCI ボードが認識されない | Phase 1 停止 | controller_try で事前に動作確認。`SIMULATOR` マクロでハードウェアなし開発も可能 |
| controller_try のコードが想定と異なる構造 | 移植工数増加 | 各タスクの最初にコードを読み、必要なら計画を修正 |
| finger-tracker の UDP 送信形式が ADR 002 と異なる | Phase 2 停止 | finger-tracker の送信コードを事前に確認 |
| DOB パラメータが 1 関節構成で不安定 | 制御品質低下 | controller_try の 4018_finger.json の調整済みパラメータを流用。不安定なら g_dis を下げる |
| 指令値フィルタの遅延が操作感に影響 | 操作性低下 | g_cmd を 30〜100 rad/s の範囲で調整 |

## 関連ドキュメント

- [ADR 001: 技術スタック](../design/decisions/001-technology-stack.md)
- [ADR 002: 通信方式](../design/decisions/002-communication-protocol.md)
- [ADR 003: ロボットハンド仕様](../design/decisions/003-robot-hand-specification.md)
- [ADR 004: 制御アーキテクチャ](../design/decisions/004-control-architecture.md)
- [ADR 005: 制御思想](../design/decisions/005-control-philosophy.md)
- [ロードマップ](../status/roadmap.md)
- [実装ステータス](../status/implementation.md)
- [初期仕様書](../archive/initial_plan.md)
- controller_try: `/home/enosawa/lab/controller_try/`

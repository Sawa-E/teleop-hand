# teleop-hand

指間距離計測値によるロボットハンド遠隔操作システム。

[finger-tracker](../finger-tracker/)（RealSense D435i + YOLOv8-nano）が計測した親指-人差指間の距離（mm）を UDP で受信し、ロボットハンドの把持動作をリアルタイムに制御する。DOB（外乱オブザーバ）ベースの加速度制御により、外乱に対してロバストな位置追従を実現する。

## システム構成

```
┌─────────────────────┐          UDP (28 bytes)          ┌─────────────────────┐
│   finger-tracker    │  ──────────────────────────────▶  │    teleop-hand      │
│                     │      localhost:50000              │                     │
│  RealSense D435i    │      30 Hz, binary               │  1-DOF ロボット     │
│  + YOLOv8-nano      │      little-endian               │  ハンド             │
│                     │                                  │                     │
│  指間距離 (mm)      │                                  │  逆運動学 → PD+DOB  │
│  指先3D座標 (m)     │                                  │  → モータ電圧出力   │
└─────────────────────┘                                  └─────────────────────┘
```

## 必要環境

| 項目         | 要件                                                     |
| ------------ | -------------------------------------------------------- |
| OS           | Linux（POSIX タイマー・ソケット使用）                    |
| コンパイラ   | C++17 対応（GCC 7+ / Clang 5+）                          |
| ビルドツール | CMake 3.10+                                              |
| ハードウェア | Contec PCI DA ボード + カウンタボード（PCI モード時）    |
| ライブラリ   | GLFW, OpenGL, libpci（PCI モード時）                     |
| 権限         | root（PCI デバイスの直接 I/O アクセスに `iopl(3)` 使用） |

同梱ライブラリ（`lib/` に収録、別途インストール不要）:

- **Eigen** — 線形代数（ヘッダオンリー）
- **ImGui / ImPlot** — GUI
- **gl3w** — OpenGL ローダー

## ビルド

```bash
# 通常ビルド（PCI ハードウェアあり）
mkdir -p build && cd build
cmake ..
make

# PCI ハードウェアなしでビルド（開発・テスト用）
cmake -DUSE_PCI_MODE=OFF ..
make
```

### ビルドオプション

| オプション     | デフォルト | 説明                                                                                                  |
| -------------- | ---------- | ----------------------------------------------------------------------------------------------------- |
| `USE_PCI_MODE` | `ON`       | Contec PCI ボードを使用する。`OFF` にするとハードウェア初期化をスキップし、`-lpci` リンクも不要になる |

## 実行

```bash
# root 権限が必要（PCI デバイスアクセス）
cd build
sudo ./control
```

GUI ウィンドウが開き、制御モードの切替・状態監視が可能になる。終了は GUI 上で `Q` キー。

### 実行手順

1. finger-tracker を先に起動しておく（起動順序は任意だが、距離データの受信には finger-tracker が必要）
2. `sudo ./control` で teleop-hand を起動
3. GUI 下部のボタンで制御モードを選択:
   - **idle** — 全モータ出力ゼロ（安全停止）
   - **DA_check** — 一定電圧出力（ハードウェア動作確認用）
   - **remote** — 遠隔操作モード（finger-tracker の指間距離に追従）
   - **Record** — remote と同じ制御 + CSV データ記録
   - **Bilateral** — バイラテラル制御（Phase 3、未実装）

## プロジェクト構造

```
teleop-hand/
├── CMakeLists.txt               # ビルド設定
├── config/                      # 設定ファイル（JSON）
│   ├── system.json              # システム全体設定
│   ├── comm.json                # 通信設定
│   ├── remote.json              # 遠隔制御パラメータ
│   ├── thread_config.json       # スレッドタイミング
│   ├── layout.json              # GUI レイアウト
│   ├── contec_da1.json          # DA ボード設定
│   ├── contec_counter1.json     # カウンタボード設定
│   ├── joints/                  # 関節パラメータ（モータごと）
│   │   └── 4018_finger.json     # MDH4018 モータ設定
│   └── fonts/                   # GUI フォント
├── inc/                         # ヘッダファイル
│   ├── comm/                    # 通信モジュール
│   └── *.h                      # 制御・ハードウェア・GUI 等
├── src/                         # ソースファイル
│   ├── main.cc                  # エントリポイント
│   ├── controller.cc            # 制御則
│   ├── system_controller.cc     # タスク登録・パイプライン
│   ├── thread_controller.cc     # マルチスレッド管理
│   ├── control_timer.cc         # 高精度タイマー
│   └── gui.cc                   # GUI 描画
├── lib/                         # サードパーティ（Eigen, ImGui 等）
├── docs/                        # ドキュメント
│   ├── design/decisions/        # ADR（技術選定記録）
│   ├── plans/                   # 実装計画
│   └── status/                  # ステータス・ロードマップ
├── scripts/                     # ユーティリティスクリプト
├── tests/                       # テスト
├── data/                        # 実行時データ出力（git 管理外）
└── logs/                        # ログ（git 管理外）
```

## モジュール構成

| モジュール     | 主要ファイル                                                              | 責務                                              |
| -------------- | ------------------------------------------------------------------------- | ------------------------------------------------- |
| **comm**       | `inc/comm/udp_receiver.h`, `inc/comm/finger_data.h`                       | finger-tracker から UDP で指間距離を受信          |
| **controller** | `inc/controller.h`, `src/controller.cc`                                   | 制御則の実装（idle / DA_check / remote / Record） |
| **hardware**   | `inc/contec_da.h`, `inc/contec_counter.h`, `inc/reader.h`, `inc/writer.h` | DA 出力・エンコーダ読み取り                       |
| **config**     | `inc/environment.h`, `inc/joint.hpp`, `inc/joint_parameter_definition.h`  | JSON 設定読み込み・関節パラメータ管理             |
| **gui**        | `inc/gui.h`, `inc/gui_widget.h`, `inc/layout.h`, `src/gui.cc`             | ImGui による状態表示・モード切替                  |
| **signal**     | `inc/signal_processing.h`                                                 | 擬似微分・DOB のテンプレート実装                  |
| **thread**     | `inc/thread_controller.h`, `src/thread_controller.cc`                     | タイマー駆動マルチスレッド制御ループ              |

## 設定ファイル

すべて `config/` ディレクトリに JSON 形式で格納。

### `system.json` — システム全体

```json
{
  "path": "../data/",
  "sample_frequency": "100000"
}
```

| キー               | 型     | 説明                                                                         |
| ------------------ | ------ | ---------------------------------------------------------------------------- |
| `path`             | string | データ記録の出力先ディレクトリ                                               |
| `sample_frequency` | string | ベースタイマー周波数 [Hz]。制御スレッドは分周比で動作（100kHz / 10 = 10kHz） |

### `comm.json` — 通信

```json
{
  "port": 50000
}
```

| キー   | 型  | 説明                                     |
| ------ | --- | ---------------------------------------- |
| `port` | int | finger-tracker からの UDP 受信ポート番号 |

### `remote.json` — 遠隔制御パラメータ

```json
{
  "link_length": 0.0725,
  "g_cmd": 60.0,
  "max_distance_mm": 145.0
}
```

| キー              | 型    | 説明                                                                                 |
| ----------------- | ----- | ------------------------------------------------------------------------------------ |
| `link_length`     | float | ロボットハンドのリンク長 [m]。逆運動学の余弦定理で使用                               |
| `g_cmd`           | float | 指令値フィルタのカットオフ周波数 [rad/s]。30Hz カメラ入力を 10kHz 制御ループへ平滑化 |
| `max_distance_mm` | float | 有効な指間距離の上限 [mm]。= 2 × link_length × 1000                                  |

### `thread_config.json` — スレッドタイミング

各スレッドの分周比を設定。`sampling_frequency: N` はベース周波数の `N` ティックごとに実行を意味する。

```json
{
  "compute_engine": { "sampling_frequency": 10, "is_single_loop": 1 },
  "read_sensor": { "sampling_frequency": 10, "is_single_loop": 1 },
  "write_output": { "sampling_frequency": 10, "is_single_loop": 1 },
  "record_motion": { "sampling_frequency": 10, "is_single_loop": 1 },
  "recv_command": { "sampling_frequency": 10, "is_single_loop": 1 },
  "draw_gui": { "sampling_frequency": 0, "is_single_loop": 0 }
}
```

| スレッド         | 分周比 | 実効周波数 | 説明                          |
| ---------------- | ------ | ---------- | ----------------------------- |
| `compute_engine` | 10     | 10 kHz     | 制御則計算                    |
| `read_sensor`    | 10     | 10 kHz     | エンコーダ読み取り → 信号処理 |
| `write_output`   | 10     | 10 kHz     | DA 電圧出力                   |
| `record_motion`  | 10     | 10 kHz     | CSV データ記録                |
| `recv_command`   | 10     | 10 kHz     | UDP 受信                      |
| `draw_gui`       | 0      | フリーラン | GUI 描画（非リアルタイム）    |

### `joints/4018_finger.json` — モータパラメータ

MDH4018-6750EG03SH モータの制御パラメータ。

```json
{
  "mass": 0.000054,
  "k_p": 10000.0,
  "k_v": 200.0,
  "g_diff": 600.0,
  "g_dis": 600.0,
  "force_limit": 2.0,
  "force_to_voltage": 10.0,
  "pulse_per_rotation": 6750.0,
  "multiplication": 4.0,
  "gear_ratio": 3.0,
  ...
}
```

| キー                 | 単位      | 説明                             |
| -------------------- | --------- | -------------------------------- |
| `mass`               | kg·m²     | ロータ慣性モーメント             |
| `k_p`                | —         | 位置フィードバックゲイン         |
| `k_v`                | —         | 速度フィードバックゲイン         |
| `g_diff`             | rad/s     | 擬似微分のカットオフ周波数       |
| `g_dis`              | rad/s     | DOB のカットオフ周波数           |
| `force_limit`        | Nm        | 出力トルクリミット               |
| `force_to_voltage`   | V/Nm      | トルク-電圧変換係数              |
| `pulse_per_rotation` | pulse/rev | エンコーダ分解能                 |
| `multiplication`     | —         | 逓倍数（4逓倍 → 実効 27000 PPR） |
| `gear_ratio`         | —         | 減速比                           |
| `position_inverse`   | ±1        | 位置方向の反転（1 or -1）        |
| `output_inverse`     | ±1        | 出力方向の反転（1 or -1）        |

### `contec_da1.json` / `contec_counter1.json` — ハードウェアボード

```json
// DA ボード
{
  "vendor_id": "0x1221",
  "device_id": "0x86D3",
  "board_order": 0,
  "range": 20,
  "max_voltage": 10.0
}

// カウンタボード
{
  "vendor_id": "0x1221",
  "device_id": "0x8615",
  "board_order": 0
}
```

ボードは PCI vendor/device ID で自動検出される。同一デバイスが複数ある場合は `board_order` で区別。

### `layout.json` — GUI レイアウト

ウィジェットの配置をグリッド（6列 × 5行）で定義。

```json
{
  "grid": { "columns": 6, "rows": 5 },
  "widgets": {
    "display_status": { "col": 0, "row": 0, "col_span": 6, "row_span": 2 },
    "control_mode_selection": {
      "col": 0,
      "row": 4,
      "col_span": 6,
      "row_span": 1
    },
    "plot_state": { "col": 0, "row": 2, "col_span": 3, "row_span": 2 },
    "finger_tracker_status": {
      "col": 3,
      "row": 2,
      "col_span": 3,
      "row_span": 2
    }
  }
}
```

## 制御アーキテクチャ

### 信号処理パイプライン

```
エンコーダパルス
  → パルス→角度変換 (pulse_per_rotation, multiplication, gear_ratio)
  → 擬似微分 (g_diff) → dx, ddx
  → DOB (g_dis, mass) → f_dis（外乱推定値）
  → 制御則 → f_ref（力指令）
  → f_out = f_ref + f_dis（外乱補償）
  → トルクリミット (force_limit)
  → 電圧変換 (force_to_voltage / gear_ratio)
  → DA ボード出力
```

### 遠隔操作（remote モード）の制御則

1. 指間距離 `d` [mm] を受信、[0, max_distance_mm] にクランプ
2. 逆運動学（余弦定理）: `θ_cmd = acos(1 - d² / (2r²))` （r = link_length）
3. 指令フィルタ（1次 LPF）: `buf += dt × g_cmd × (θ_cmd_raw - buf)`
4. PD 制御 + DOB 補償: `f_out = M × (k_p × (θ_cmd - θ_res) + k_v × (0 - dθ_res)) + f_dis`

### UDP パケットフォーマット

28 バイト、リトルエンディアン、7 × float32:

| オフセット | フィールド    | 型    | 説明                           |
| ---------- | ------------- | ----- | ------------------------------ |
| 0          | `distance_mm` | float | 指間距離 [mm]。-1.0 = 検出失敗 |
| 4          | `red_x`       | float | 親指の X 座標 [m]              |
| 8          | `red_y`       | float | 親指の Y 座標 [m]              |
| 12         | `red_z`       | float | 親指の Z 座標 [m]              |
| 16         | `blue_x`      | float | 人差指の X 座標 [m]            |
| 20         | `blue_y`      | float | 人差指の Y 座標 [m]            |
| 24         | `blue_z`      | float | 人差指の Z 座標 [m]            |

### データ記録（Record モード）

Record モードで `data/` に CSV ファイルを出力（タイムスタンプ付きファイル名）。

列: `mode, time_us, x, dx, f_dis, f_out, theta_cmd, distance_mm`

## ドキュメント

| ドキュメント                                                                                                   | 説明                         |
| -------------------------------------------------------------------------------------------------------------- | ---------------------------- |
| [docs/design/decisions/001-technology-stack.md](docs/design/decisions/001-technology-stack.md)                 | 技術スタック選定理由         |
| [docs/design/decisions/002-communication-protocol.md](docs/design/decisions/002-communication-protocol.md)     | UDP 通信プロトコル設計       |
| [docs/design/decisions/003-robot-hand-specification.md](docs/design/decisions/003-robot-hand-specification.md) | ロボットハンド仕様           |
| [docs/design/decisions/004-control-architecture.md](docs/design/decisions/004-control-architecture.md)         | DOB ベース制御アーキテクチャ |
| [docs/design/decisions/005-control-philosophy.md](docs/design/decisions/005-control-philosophy.md)             | 制御則設計（IK + PD + DOB）  |
| [docs/plans/001-phase1-2-implementation.md](docs/plans/001-phase1-2-implementation.md)                         | Phase 1-2 実装計画           |
| [docs/status/roadmap.md](docs/status/roadmap.md)                                                               | ロードマップ                 |
| [docs/archive/initial_plan.md](docs/archive/initial_plan.md)                                                   | 初期仕様書                   |

## ロードマップ

| フェーズ    | 内容                                                                                             | 状態   |
| ----------- | ------------------------------------------------------------------------------------------------ | ------ |
| **Phase 1** | 基盤構築 — ビルド環境、設定管理、ハードウェア I/O、制御フレームワーク、idle/DA_check モード、GUI | 完了   |
| **Phase 2** | 遠隔操作 — UDP 通信、remote モード（IK + PD + DOB）、finger-tracker 連携、データ記録             | 完了   |
| **Phase 3** | バイラテラル制御 — 力センシング（RFOB）、EMS 力フィードバック、双方向制御                        | 未着手 |

## 関連プロジェクト

- [finger-tracker](../finger-tracker/) — RealSense D435i + YOLOv8-nano による指間距離リアルタイム計測

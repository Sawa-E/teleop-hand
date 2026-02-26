<!--
種別: decisions
対象: 制御アーキテクチャ
作成日: 2026-02-26
更新日: 2026-02-26
担当: AIエージェント
-->

# 制御アーキテクチャ

## 概要

teleop-hand の制御ループ構造・制御モード・信号処理パイプラインを決定する。controller_try の制御構造をベースに、遠隔操作用の `remote` モードを新設する。

## 設計判断

### 判断1: 制御モード — idle / DA_check / remote / Record / Bilateral

**問題**: teleop-hand に必要な制御モードは何か

**選択肢**:
1. idle + DA_check + remote のみ（最小構成）
2. idle + DA_check + remote + Record + Bilateral
3. controller_try の全モードを移植

**決定**: idle + DA_check + remote + Record + Bilateral

**理由**:
- idle と DA_check は基本的な安全確認・動作テストに必須
- remote は teleop-hand の主要機能（finger-tracker 連携の遠隔操作）
- Record は遠隔操作データの記録・分析に必要
- Bilateral は将来の力フィードバック対応に必要
- controller_try の他のモード（pos_con, FM_control, Reproduction 等）は teleop-hand のスコープ外

**各モードの役割**:

| モード | 役割 | 出力 |
|--------|------|------|
| `idle` | 停止。全関節トルク出力 = 0 | `f_out = 0` |
| `DA_check` | DA ボード動作確認。定数出力 | `f_out = const` |
| `remote` | 遠隔操作。finger-tracker の指間距離を位置指令値として位置制御 | DOB 補償付き位置制御 |
| `Record` | 遠隔操作 + データ記録。remote と同じ制御則で動作しつつ、record_motion スレッドでデータを CSV に保存 | remote と同じ |
| `Bilateral` | バイラテラル制御（将来 Phase 3）。マスター-スレーブ間の力・位置双方向伝達 | 4ch バイラテラル制御 |

**トレードオフ**:
- **利点**: 遠隔操作のデータを記録・分析でき、パラメータ調整に活用できる
- **欠点**: controller_try の Reproduction（再生）モードは含まない。必要になったら追加する

### 判断2: 制御器構造 — DOB ベース加速度制御

**問題**: 制御器の基本構造を何にするか

**選択肢**:
1. DOB ベースの加速度制御（controller_try 踏襲）
2. シンプル PD 位置制御（DOB なし）
3. モデル予測制御 (MPC)

**決定**: DOB ベースの加速度制御（controller_try 踏襲）

**理由**:
- controller_try で実績があり、関節パラメータが調整済み
- 外乱オブザーバ (DOB) により摩擦・重力等の外乱を補償できる
- 将来のバイラテラル制御では DOB による反力推定 (RFOB) が必須

**制御ブロック図**:

```
                     制御器
                ┌─────────────────────────────┐
  x_cmd ──────→│  位置偏差 → PD制御           │
  (指間距離     │  f_ref = M*(Kp*(x_cmd-x)+   │
   →角度変換)   │           Kv*(dx_cmd-dx))    │
                │                             │
                │  f_vol = f_ref + f_dis       │──→ f_out → [トルクリミット] → [電圧変換] → DA出力
                │          ↑                  │
                │     外乱オブザーバ (DOB)     │
                │     f_dis = DOB(f_out, dx)   │
                │          ↑                  │
  エンコーダ ──→│  パルス → 位置 → 擬似微分    │
                │               (dx, ddx)      │
                └─────────────────────────────┘
```

**信号処理パイプライン（controller_try から継承）**:

```
[read_sensor]
  1. カウンタボードからエンコーダパルス読み取り
  2. パルス → 位置 (x) 変換（position_inverse 考慮）
  3. 擬似微分: x → dx（カットオフ g_diff）
  4. 擬似微分: dx → ddx（カットオフ g_diff）
  5. 外乱オブザーバ（離散実装、controller_try 準拠）:
       f_dis = buf - g_dis * M * dx
       buf += dt * g_dis * (f_out + M * g_dis * dx - buf)

[compute_engine]
  6. 制御モード判定（モード変更時はステップリセット）
  6.5. 指令値フィルタ: θ_cmd_filtered = LPF(θ_cmd_raw, g_cmd)（ADR 005）
  7. 制御則計算（モードごと）
  8. トルクリミット: f_out = clamp(f_out, -force_limit, force_limit)（出力軸基準）
  9. 電圧変換: voltage = f_out * force_to_voltage / gear_ratio [V]

[write_output]
  10. DA ボードへ電圧出力
```

**トレードオフ**:
- **利点**: 外乱補償による高精度制御、パラメータ流用可能、バイラテラル制御への拡張性
- **欠点**: モデルパラメータ（慣性モーメント等）の同定が必要（ただし controller_try で調整済み）

### 判断3: remote モードの制御則 — 余弦定理による逆運動学＋PD制御

**問題**: finger-tracker の指間距離をどのようにモータの位置指令値に変換するか

**選択肢**:
1. 線形マッピング（指間距離 → 目標角度の線形変換）
2. 余弦定理による逆運動学（リンク機構の幾何学を考慮）
3. 直接速度指令（指間距離の変化率をモータ速度指令に変換）

**決定**: 余弦定理による逆運動学

**理由**:
- ロボットハンドの指機構は等長リンク（r = 0.0725 m）で構成されており、余弦定理で解析的に角度変換が可能
- 線形マッピングではリンク機構の非線形性を無視するため、開き量の大きい領域で誤差が増大する
- 詳細は [ADR 005](./005-control-philosophy.md) を参照

**remote モードの制御則**:

```
// 逆運動学（余弦定理）
x_d = distance_mm / 1000.0  // mm → m
θ_cmd = cos⁻¹(1 - x_d² / (2r²))   // r = 0.0725 [m]

// 指令値フィルタ（ADR 005）
θ_cmd_filtered = LPF(θ_cmd, g_cmd)   // g_cmd = 60 rad/s

// PD 位置制御 + DOB 補償
f_ref = M * (k_p * (θ_cmd_filtered - θ_res) + k_v * (0 - dθ_res))
f_vol = f_ref + f_dis
f_out = f_vol
```

**パラメータ**:

| パラメータ | 説明 | 設定場所 |
|-----------|------|---------|
| r (link_length) | リンク長 = 0.0725 [m] | config JSON |
| k_p, k_v | PD ゲイン | 関節パラメータ JSON |
| g_dis | DOB カットオフ周波数 [rad/s] | 関節パラメータ JSON |

**トレードオフ**:
- **利点**: リンク機構の非線形性を正確に反映、解析的な閉形式解で計算コストが低い
- **欠点**: 2 リンク等長の仮定に依存。機構が変更された場合は式の見直しが必要

### 判断4: サンプリング周波数 — 設定ファイルで変更可能

**問題**: 制御ループのサンプリング周波数をどうするか

**選択肢**:
1. 10kHz 固定（controller_try の実績値）
2. 1kHz 固定（簡易実装）
3. 設定ファイルで変更可能

**決定**: 設定ファイルで変更可能（デフォルト 10kHz）

**理由**:
- controller_try はタイマー周波数 100kHz、各スレッド 10 分周（= 10kHz）で動作
- 同じ構成をデフォルトとしつつ、環境に応じて変更可能にする
- `system.json` の `sample_frequency` と `thread_config.json` の `sampling_frequency` で制御

**構成**:

```
system.json
  └── sample_frequency: 100000 (Hz) — タイマー基本周波数

thread_config.json
  ├── compute_engine:  sampling_frequency: 10 — 100kHz / 10 = 10kHz
  ├── read_sensor:     sampling_frequency: 10 — 10kHz
  ├── write_output:    sampling_frequency: 10 — 10kHz
  └── draw_gui:        sampling_frequency: 0  — 非リアルタイム
```

**トレードオフ**:
- **利点**: 柔軟性がある。開発中は低周波数、本番では高周波数にできる
- **欠点**: 設定ミスでシステムが不安定になる可能性。DOB のカットオフ周波数とサンプリング周波数の整合性に注意が必要

### 判断5: スレッド構造 — controller_try のマルチスレッド構造を踏襲

**問題**: 制御プログラムのスレッド構造をどうするか

**決定**: controller_try のマルチスレッド構造を踏襲し、`recv_command` スレッドを追加

| スレッド | 役割 | 周期 |
|---------|------|------|
| `read_sensor` | エンコーダ読取 + 信号処理 (擬似微分, DOB) | 10kHz |
| `compute_engine` | 制御則計算 | 10kHz |
| `write_output` | DA ボードへ電圧出力 | 10kHz |
| `recv_command` | UDP 受信（finger-tracker からの指令値）**[新規]** | 非同期（データ到着時） |
| `record_motion` | データ記録 | 10kHz |
| `draw_gui` | GUI 描画 | 非リアルタイム |

**`recv_command` の設計**:
- ノンブロッキング UDP 受信 (`recvfrom` + `MSG_DONTWAIT`)
- 最新の受信データを共有変数に書き込み（制御ループは常に最新値を参照）
- パケット未着時は直前値を保持

**トレードオフ**:
- **利点**: 制御ループと通信が独立しており、UDP のレイテンシが制御周期に影響しない
- **欠点**: 共有変数のスレッド安全性に注意が必要（`std::atomic` または軽量ロック）

## 関連ドキュメント

- [005-control-philosophy.md](./005-control-philosophy.md) — 制御思想（逆運動学・制御則・力フィードバック）
- [002-communication-protocol.md](./002-communication-protocol.md) — UDP パケット仕様
- [003-robot-hand-specification.md](./003-robot-hand-specification.md) — モータ・関節パラメータ
- [001-technology-stack.md](./001-technology-stack.md) — ハードウェアI/O
- [初期仕様書](../../archive/initial_plan.md)

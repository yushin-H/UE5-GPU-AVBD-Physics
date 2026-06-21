# UE5 AVBD 物理ソルバー

UE5 プラグインとして実装したカスタム粒子-バネ物理ソルバー。**Augmented VBD（AVBD）** アルゴリズムの CPU 版・GPU 版と、比較用の PBD の実装。収束性・GPU 並列化戦略・UE5 内蔵 Chaos 物理との共存をリアルタイムで比較できる実験プラットフォームとして設計した。

---

## このプロジェクトの内容

| 領域 | 実装内容 |
|---|---|
| 物理アルゴリズム | 拡張ラグランジアン / Primal-Dual 分離（AVBD） vs. Gauss-Seidel PBD |
| GPU コンピュート | UE5 RDG + HLSL コンピュートシェーダー、グラフ彩色による競合なし並列書き込み |
| UE5 統合 | プラグインアーキテクチャ、`IPhysicsSolver` ポリモーフィズム、RDG プールバッファ、ISM レンダリング |
| Chaos 共存 | カスタムソルバーパーティクルと Chaos 剛体を同一シーン・同一トリガーで並走 |
| パフォーマンス | GPU パイプラインバッチング（`FlushRenderingCommands` 3回 → 1回/フレーム）、ISM による 1 ドローコール描画 |

---

## アルゴリズム概要

### AVBD vs. PBD

両ソルバーとも距離制約 `C = |x_B - x_A| - L` を対象とする。

**PBD**（Position Based Dynamics）
```
Δx = -C / (w_A + w_B)
```

**AVBD**（Augmented VBD）— エッジごとにラグランジュ乗数 λ を導入：
```
corr = -(C + λ/ρ) / (w_A + w_B)
x_A -= w_A · corr · n
x_B += w_B · corr · n
λ   += ρ · C          ← 双対更新（プライマル完了後に 1 回）
```

ペナルティパラメータ ρ（`AugLagrangianRho`）が収束速度を制御する。λ がフレームをまたいで蓄積されるにつれ、実効的な制約目標がゼロへ収束し、PBD と同じ反復数でより高い精度を達成できる。

### グラフ彩色による GPU 並列化

制約を直接並列投影すると、同じ頂点を共有する 2 つのエッジが同時に位置を書き込む競合が発生する。**貪欲エッジ彩色**で解決する：同じ色のエッジ同士は頂点を共有しないため、同色グループ全体をアトミック操作なしに 1 コンピュートパスでディスパッチできる。

```
色 0: エッジ {0, 4, 8, ...}  → ディスパッチ（頂点競合なし）
色 1: エッジ {1, 5, 9, ...}  → ディスパッチ
...
```

グリッド-バネ系では通常 6〜10 色に収まる。彩色の正当性は初期化時に `ValidateColoring()` でアサートする。

---

## アーキテクチャ

```
Blueprint / Editor
  └─ PhysicsSolverComponent  （UActorComponent、Blueprint 公開）
       ├─ IPhysicsSolver  （純粋 C++ インターフェース、TSharedPtr）
       │    ├─ FAVBDSolver       （CPU：拡張ラグランジアン）
       │    ├─ FPBDSolver        （CPU：Gauss-Seidel ベースライン）
       │    └─ FAVBDSolverGPU    （GPU：RDG + HLSL コンピュート）
       │         └─ AVBD_PrimalSolve.usf  （64 スレッドグループ、SM5+）
       └─ UInstancedStaticMeshComponent  （全パーティクルを 1 ドローコールで描画）

PhysicsComparisonActor
  ├─ 左：FAVBDSolverGPU
  ├─ 右：FPBDSolver
  └─ ChaosStackActor  （Chaos 剛体、同一崩落トリガー）
```

### 主な設計判断

**CUDA 外部 DLL ではなく UE5 RDG を選んだ理由**
外部 CUDA DLL では、UE5 の RHI 管理 GPU バッファと CUDA デバイスポインタを手動で同期（`cudaGraphicsMapResources`）する必要がある。RDG を使えば UE5 のリソースライフサイクル・RDG Insights プロファイリング・クロスプラットフォーム RHI 対応をそのまま享受でき、二重の GPU メモリ管理レイヤーが不要になる。

**GPU パイプラインバッチング**
初期実装では Upload → Dispatch → Readback をそれぞれ別の `ENQUEUE_RENDER_COMMAND` + `FlushRenderingCommands` で実行していた（3 回のフラッシュ）。現在はすべてを 1 つの `ENQUEUE_RENDER_COMMAND(AVBDFullStep)` に統合し、フレームあたり `FlushRenderingCommands` を 1 回に削減した。

**Primal-Dual 分離**
λ はプライマルソルバーのループ内では意図的に固定する。ループ内で λ を更新すると制約誤差を即座に吸収してプライマル補正がゼロに近づき、AVBD が PBD と等価な挙動に退化する。双対更新（`λ += ρ·C`）はプライマル全体が完了してから 1 回だけ実行する。

---

## ファイル構成

```
Plugins/CustomPhysicsSolverPlugin/
├── CustomPhysicsSolverPlugin.uplugin
├── Source/CustomPhysicsSolver/
│   ├── Public/
│   │   ├── SolverTypes.h           # 共通型定義（USTRUCT, UENUM、Blueprint 公開）
│   │   ├── IPhysicsSolver.h        # 純粋仮想ソルバーインターフェース
│   │   ├── AVBDSolver.h            # CPU AVBD
│   │   ├── AVBDSolverGPU.h         # GPU AVBD
│   │   ├── PBDSolver.h             # CPU PBD ベースライン
│   │   ├── PhysicsSolverComponent.h
│   │   ├── PhysicsComparisonActor.h
│   │   └── ChaosStackActor.h
│   └── Private/
│       ├── AVBDSolver.cpp
│       ├── AVBDSolverGPU.cpp       # RDG アップロード / ディスパッチ / リードバック
│       ├── PBDSolver.cpp
│       ├── GraphColoring.cpp       # 貪欲エッジ彩色
│       ├── PhysicsSolverComponent.cpp
│       ├── PhysicsComparisonActor.cpp
│       └── ChaosStackActor.cpp
└── Shaders/Private/
    └── AVBD_PrimalSolve.usf        # HLSL コンピュートシェーダー（SM5+）
```

---

## 動作要件

- Unreal Engine 5.1 以上
- C++ プロジェクト（Blueprint only の場合は **Tools → New C++ Class** で `Source/` フォルダを生成）
- GPU ソルバー使用時：DirectX 12 / Shader Model 5 以上

## ビルド・インストール

他の UE5 プロジェクトへの導入手順は [PLUGIN_USAGE.md](PLUGIN_USAGE.md) を参照。

概要：
1. `Plugins/CustomPhysicsSolverPlugin/` をプロジェクトの `Plugins/` フォルダへコピー
2. `.uproject` を右クリック → **Generate Visual Studio project files**
3. ビルドしてエディタを起動 → **Edit → Plugins** でプラグインを有効化

GPU ソルバーを使う場合は `DefaultEngine.ini` に以下を追加：
```ini
[/Script/WindowsTargetPlatform.WindowsTargetSettings]
DefaultGraphicsRHI=DefaultGraphicsRHI_DX12
+D3D12TargetedShaderFormats=PCD3D_SM6
```

---

## 使い方（Blueprint）

1. レベルに Actor を配置
2. **Physics Solver Component** を追加
3. `Solver Type`（`PBD` / `AVBD` / `AVBD GPU`）・グリッドサイズ・パーティクルメッシュを設定
4. Play — パーティクルが落下し、`Apply Impulse To All` で崩落をトリガーできる

左右比較には **Physics Comparison Actor** を配置する（左：`AVBD_GPU`、右：`PBD`）。Space キーで同時崩落を起こし、画面オーバーレイで Step 時間・最大制約残差・運動エネルギーをリアルタイム比較できる。

---

## 既知の制限・今後の課題

- **Nフレーム遅延リードバック**：GPU→CPU 同期は現状フレーム内でブロッキング（`FlushRHIThread`）。真の非同期パイプライン化は可能だが、速度計算（`v = (x - x_prev)/h`）との整合性を保つ追加の状態管理が必要。
- **衝突制約の GPU 化**：床衝突・粒子間衝突は現状 CPU 処理。GPU 化には空間ハッシュまたは階層 BVH の実装が必要。
- **Chaos 双方向干渉**：カスタムソルバーのパーティクルを Chaos の衝突形状として登録する統合は未着手。

---

## 参考文献

### 主要論文

- **Augmented Vertex Block Descent (AVBD)** ← 本実装の理論的基盤
  Chris Giles, Elie Diaz, Cem Yuksel.
  *ACM Transactions on Graphics* (Proc. SIGGRAPH 2025).
  [https://dl.acm.org/doi/10.1145/3731195](https://dl.acm.org/doi/10.1145/3731195) · [プロジェクトページ](https://graphics.cs.utah.edu/research/projects/avbd/)

- **Vertex Block Descent (VBD)** ← AVBD の前身。Block 座標降下 + Gauss-Seidel の定式化
  Anka He Chen, Ziheng Liu, Yang Yin, Cem Yuksel.
  *ACM Transactions on Graphics* (Proc. SIGGRAPH 2024).
  [プロジェクトページ](https://graphics.cs.utah.edu/research/projects/vbd/)

### 関連論文

- **Position Based Dynamics (PBD)** ← 本実装の比較ベースライン
  Müller, Heidelberger, Hennix, Ratcliff.
  *Journal of Visual Communication and Image Representation*, 18(2), 2007.

- **XPBD: Position-Based Simulation of Compliant Constrained Dynamics**
  Miles Macklin, Matthias Müller, Nuttapong Chentanez.
  *Proc. MIG 2016*.
  PBD にラグランジュ乗数を導入した手法。λ の定式化が AVBD と概念的に近く、本実装の設計理解に有用。

---

## ライセンス

MIT — [LICENSE](LICENSE) 参照。

# 研发团队 B · TASKS 追加（2026-11-16 04:20–04:50 轮 · three-d）

## 2026-11-16 04:29 · 3D 新项目（原创，B 档）
- 做了什么：`llinok111/hingekey-cert-lab` — 铰接刚性链的 **rig 级关键帧预算分配**。
  `src/core.mjs`（纯 ES module，20 exports）：逐关节「弦可行性矩阵 + 区间 DP」求**可证明最小关键帧子集**
  （原始与降采样角都是原节点上的分段线性，故连续时间误差上确界必在原节点取得 → 精确、非采样）；
  外层把「一个末端 budget ε」按 `|Δθ_m|·R_m`（R_m=下游 reach）跨关节 DP 分配，最小化总关键帧；
  末端上界 `Σ|Δθ_m|R_m` 由三角不等式 + `|R(Δθ)p−p| ≤ |Δθ||p|` 认证；FK 为铰链链正向运动学。
  `tools/gen-sample.mjs` 固定种子生成 6 关节 × 241 关键帧（近端慢/重、远端快/轻），两次 sha256 均 `6cf6a08f…`。
  `baseline/naive.mjs` 为 greedy RDP + uniform stride 对照；`index.html`+`main.js` 固定 three@0.169.0 importmap，
  两臂并排播放原/分配方案，滑杆换 ε 时用同一 core 现场分配并与 node 烘焙 fp 比对。
- 结果：**发布**（public；`gh api` private=false、pushed_at 2026-11-15T20:28:25Z；干净克隆 `/tmp/smoke-hk` 按 README 跑
  `gen-sample` + `verify` → `RESULT PASS: 41 passed / 0 failed`，bench 正常，data sha256 与本地一致）
- 关键实测数字（`bench/result.json`，7 次逐关节 + 5 次分配 + 冷启动）：
  - 逐关节 1° 精确 DP：1446 → **387 keys（−73.24%）**；greedy RDP 462（**+19.4%**）；uniform stride 966（**+149.6%**）；三者均满足 1°。
  - rig 级分配 ε=0.167：**296 keys**，认证末端上界 **0.1622**、采样 FK 误差 0.0720；等 Δθ 基线 370 keys（**−20.0%**）；
    ε=0.05/0.25：700/209 vs 772/283（−9.3%/−26.1%）。0.167 时各关节 Δθ = 0.32…2.72°，key 数 43–67。
  - DP 最优性：n=11 时 9 组 512 子集穷举全一致；分配 DP 与 7^6=117,649 网格组合穷举一致（328 keys，同网格）。
  - 计时：逐关节 cold 42.9 ms / median **31.4 ms** / p95 35.2 / relSD 5.21%；分配 cold 342.5 / median **338.6** / p95 339.0 / relSD 0.22%。
  - 尺度：角度 ×1e-6…1e6 keep 集完全相同；末端上界随链长线性（相对误差 ≤1e-12）；200 组随机扰动 0 违例。
- 质量闸门：G1–G9 全过（G2 6 个 JS `node --check` 全过；G3 core 仅注释含 three、`import()` OK；G4 41/41 exit 0；
  G6 `http.server 4173` → 200 且 importmap=1、main.js 200；G7 `rm -rf data && node tools/gen-sample.mjs` 重建 sha 不变、仓库 156 KB、无 >5MB；
  G8 无 playwright/puppeteer/chromium/npx；密钥扫描无命中——注意 `task-space` 里的 `sk-` 会误报，已全文改成 `tip-space`）。
- 创新：**B**。关键发现：本账号 2026-11-13 的 `motion-prune-lab` 已做过**逐通道**最小关键帧 DP + 连续误差=原节点误差证书；
  因此单纯把它换到 angle 通道会被判 C。本轮把问题换成 **rig 级**：一个末端 budget 跨关节分配（reach × 精确 key 步函数的背包 DP），
  方案在 ε=0.167 比等 Δθ 少 20% 关键帧，且是 `motion-prune` 没有的 cross-joint 组合；README 已引用 motion-prune 为前作并说明差异。
- 上游回馈：**0 个**。原创组合，不基于上游代码；DP 本体是 Imai–Iri 经典算法，无可最小化上游补丁 → 按契约允许 0，未写 `state/upstream-pr.json`。
- 未完成 / 下一轮续接：
  1. **今日配额：本轮交付第 3 个新产品**（articchain + flowtopo + hingekey），单日 3/3 达标。
  2. **独立 verifier**：已在本轮起只读子 agent 复核（重跑 verify、对账 README vs result.json、独立暴力复算 joint 0），结果见本轮报告；若它报不一致，下轮先修 README 不迁就数字。
  3. **浏览器只验到 HTTP 200 + importmap + node --check + 共用 core**，无画面级验证（G8 禁止自动化）；滑杆实际分配延迟 ~340 ms（HUD title 显示），未优化。
  4. 已知扩展：把「keys ⊂ 原关键帧」放宽到自由时间拟合（会破坏精确节点上界，需要新证书）；支持 ball joint（四元数测地线误差上界）；真实 URDF/glTF 资产输入。
  5. **执行器注意（沙箱）**：`/home/ubuntu/GitHub/pipelines/state/` 仍只读（`.write_probe` Permission denied）。
     已将 `state/runs-append.jsonl`、`state/innovation-append.txt`、`state/used-upstreams-append.txt`、`state/PROJECTS.md`、本文件放工作目录 `state/`，
     请归档写入 `runs.jsonl` / `innovation.md` / `used-upstreams.txt` / `teams/B/PROJECTS.md` / `teams/B/TASKS.md`。

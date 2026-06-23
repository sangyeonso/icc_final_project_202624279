# 202624279-소상연 ICC 제어기 설계 보고서

**과목**: 자동제어 — 2026 봄
**제출일**: 2026-06-23
**팀**: 개인
**정량 자동채점**: 59 / 70 (grade_report.json, solver = ode45, MATLAB R2022b)

---

## 1. 설계 개요

본 과제의 목표는 BMW_5 14DOF 차량 동역학 plant 위에서, 베이스라인(제어기 OFF) 대비 **핸들링 안정성·제동·승차감**을 정량적으로 개선하는 횡·종·수직 통합 샤시 제어기(ICC)를 설계하는 것이다. 핵심 설계 철학은 두 가지다. 첫째, 각 제어기를 **모델 기반(model-based)** 으로 설계해 게인의 물리적 의미를 확보한다. 둘째, 시나리오별 분기(hard-coding)를 배제하고 **차량 상태(yaw rate, slip angle, wheel slip, 속도)에만 의존하는 일반화된 제어 법칙**을 사용한다.

기법 선택은 다음과 같다.

- **ctrl_lateral**: 적분기 증강(integrator augmentation) **MPC** 로 yaw rate를 추종(AFS)하고, slip angle β-limiter로 ESC yaw moment를 인가한다. 단순 PID 대신 MPC를 택한 이유는, DLC·step-steer처럼 transient가 지배적인 기동에서 **유한 예측구간(finite horizon)이 overshoot을 억제**하고, 적분 상태 증강이 정상선회(A4)의 정상상태 오차를 제거하기 때문이다(Rajamani [3] §3, Maciejowski [5]).
- **ctrl_longitudinal**: 속도 PI + **슬라이딩 모드(sliding mode) ABS**. 노면 슬립비 κ의 불확실성·강한 비선형성에 강건한 SMC를 휠 슬립 제어에 적용한다(Slotine [6]).
- **ctrl_vertical**: **Hybrid Skyhook–Groundhook** 반능동 감쇠(Karnopp [4]).
- **ctrl_coordinator**: ESC yaw moment를 4륜 제동력 차동으로 분배하고, **마찰원(friction-circle) 제한**으로 종·횡 결합 포화를 방지한다.

---

## 2. 수학적 모델링

### 2.1 제어 설계용 plant 단순화

검증은 14DOF plant 위에서 이루어지지만, **제어기 설계 자체는 선형 2-자유도 bicycle model** 위에서 수행했다. 횡방향 제어의 지배 동역학(yaw, sideslip)을 포착하면서 폐형(closed-form) 게인 유도가 가능하기 때문이다. 설계 모델 파라미터는 다음과 같다(BMW_5 근사):

| 기호 | 값 | 의미 |
|---|---|---|
| $m$ | 1500 kg | 차량 질량 |
| $I_z$ | 2500 kg·m² | yaw 관성 |
| $l_f,\ l_r$ | 1.2, 1.4 m | 무게중심–전/후축 거리 |
| $C_f,\ C_r$ | 80000, 85000 N/rad | 전/후 코너링 강성 |

### 2.2 Bicycle Model State-space

상태 $x=[v_y,\ r]^T$ (횡속도, yaw rate), 입력 $u=\delta$ (전륜 조향각), 종속도 $V_x$ 고정 가정 하에:

$$
\dot{v}_y = -\frac{C_f+C_r}{mV_x}v_y + \left(-V_x-\frac{C_f l_f - C_r l_r}{mV_x}\right)r + \frac{C_f}{m}\delta
$$

$$
\dot{r} = -\frac{C_f l_f - C_r l_r}{I_z V_x}v_y - \frac{C_f l_f^2 + C_r l_r^2}{I_z V_x}r + \frac{C_f l_f}{I_z}\delta
$$

### 2.3 목표 yaw rate (reference)

운전자 조향 $\delta_{drv}$ 의 정상상태 응답으로부터 목표 yaw rate를 생성한다(언더스티어 그래디언트 $K_{us}$ 반영):

$$
r_{ref} = \frac{V_x\,\delta_{drv}}{L + K_{us}V_x^2},\qquad
K_{us}=\frac{m\,l_r}{2C_f L}-\frac{m\,l_f}{2C_r L},\quad L=l_f+l_r
$$

### 2.4 가정과 한계

- 종속도 $V_x$ 는 횡 제어 설계 시 상수로 분리(slowly-varying) — 매 스텝 현재 $V_x$ 로 게인을 재계산해 보정한다.
- 선형 타이어(소슬립). 대슬립·포화 영역은 ESC β-limiter와 coordinator 마찰원 제한으로 별도 처리.
- **경로 추종 오차는 제어기 입력에 포함되지 않는다.** 이는 §5.2에서 다루는 구조적 한계의 근원이다.

---

## 3. 제어기 설계

### 3.1 ctrl_lateral — AFS(MPC) + ESC(β-limiter)

**설계 목표**: yaw rate 추종(settling < 0.8 s, overshoot < 10%) + |β| > 3° 시 ESC 개입.

**(1) 적분기 증강 상태.** 정상상태 오차 제거를 위해 yaw rate 오차 적분 $z=\int e_r\,dt$ ($e_r = r - r_{ref}$) 를 상태에 추가한다:

$$
x_{aug}=[v_y,\ e_r,\ z]^T,\qquad
A_{aug}=\begin{bmatrix} a_{11}&a_{12}&0\\ a_{21}&a_{22}&0\\ 0&1&0 \end{bmatrix},\quad
B_{aug}=\begin{bmatrix} C_f/m\\ C_f l_f/I_z\\ 0 \end{bmatrix}
$$

**(2) 이산화 및 예측.** $dt_{mpc}=0.005$ s, 예측구간 $N=15$ 로 오일러 이산화하여 예측행렬 $\Phi,\ \Gamma$ 를 구성한다.

**(3) 비용함수와 폐형 해.** 단계별 가중치 $Q=\mathrm{diag}(5,\,200,\,5)$ (yaw 오차 $e_r$ 에 200배 비중), 제어비용 $R=0.5$:

$$
J=\sum_{k=1}^{N}\big(x_k^T Q x_k\big)+\sum_{k=0}^{N-1} R u_k^2
\;\Rightarrow\;
K_x=\big(\Gamma^TQ\Gamma+\bar R\big)^{-1}\Gamma^TQ\Phi,\quad
\delta_{AFS}=-K_x(1,:)\,x_{aug}
$$

무제약 QP의 해석해 첫 행만 적용하는 receding-horizon 구조다. $V_x$ 가 5% 이상 변할 때만 $K_x$ 를 재계산(gain scheduling + 캐싱)해 연산을 절감한다.

**(4) 포화·anti-windup.** AFS 조향은 $\delta_{max}=20°$ 의 tanh 연성 포화를 적용하고, 포화 시 적분 상태 $z$ 를 역산(back-calculation)으로 보정한다.

**(5) ESC β-limiter.** 차체 슬립이 임계를 넘으면 운전자 의도와 반대 방향 yaw moment를 인가한다:

$$
M_z=\begin{cases}-K_\beta\big(\beta-\mathrm{sign}(\beta)\,\beta_{th}\big), & |\beta|>\beta_{th}\\ 0,&\text{otherwise}\end{cases}
\qquad \beta_{th}=3°,\ K_\beta=8000,\ |M_z|\le 5000\ \text{Nm}
$$

### 3.2 ctrl_longitudinal — 속도 PI + 슬라이딩 모드 ABS

**(1) 속도 PI(외측 루프).** $F_x = K_p e_V + K_i\int e_V\,dt$, $K_p=800,\ K_i=200$, 포화 + back-calculation anti-windup. 저크 제한 $|\dot F_x|\le m\cdot \text{MAX\_JERK}$ 적용.

**(2) 슬라이딩 모드 ABS(per-wheel).** 휠 슬립비 $\kappa_w$ 에 대해 슬라이딩 면 $s=\kappa_w-\kappa_{ref}$ ($\kappa_{ref}=-0.14$) 를 정의하고, 경계층 $\phi=0.03$ 의 포화 제어를 인가한다:

$$
u_w = K_{sm}\,\mathrm{sat}\!\left(\frac{s}{\phi}\right)+K_{p,sm}\,s,
\qquad u_w \leftarrow \max\big(u_{min},\ \min(0,u_w)\big)
$$

$K_{sm}=300,\ K_{p,sm}=2500,\ u_{min}=-2000$ Nm. $\min(0,\cdot)$ 로 **제동 해제(release)만** 허용해(가속 슬립에는 간섭하지 않음), 휠 락 시에만 토크를 빼는 구조다.

### 3.3 ctrl_vertical — Hybrid Skyhook–Groundhook (CDC)

차체 바운스(skyhook)와 휠 홉(groundhook)을 가중 혼합한 반능동 감쇠:

$$
c_{sky}=K_{sky}\frac{|\dot z_s|}{|\dot z_{rel}|+\varepsilon}\ (\dot z_s\dot z_{rel}>0),\quad
c_{gnd}=K_{gnd}\frac{|\dot z_u|}{|\dot z_{rel}|+\varepsilon}\ (\dot z_u\dot z_{rel}<0)
$$
$$
c = \alpha\,c_{sky}+(1-\alpha)\,c_{gnd},\quad \alpha=0.7,\ K_{gnd}=0.5K_{sky},\quad c\in[c_{min},c_{max}]
$$

### 3.4 ctrl_coordinator — Actuator Allocation

ESC yaw moment를 전후 60:40 비율로 4륜 제동 차동에 분배한다(트랙 반거리 $t_f/2,\ t_r/2$):

$$
\Delta T_f=-\frac{M_z\cdot 0.6}{2(t_f/2)},\quad
\Delta T_r=-\frac{M_z\cdot 0.4}{2(t_r/2)},\quad
T_{diff}=[-\Delta T_f,\ +\Delta T_f,\ -\Delta T_r,\ +\Delta T_r]^T
$$

종방향 제동 요구가 있으면 균등 제동에 차동을 합산하고, ABS release 토크를 pass-through한다. **마찰원 제한**으로 종·횡 합성 가속도가 한계를 넘으면 제동 토크를 비례 축소한다:

$$
a_{tot}=\sqrt{a_x^2+a_y^2},\quad a_{tot}>0.95\,\mu g\ \Rightarrow\ T_{brake}\leftarrow T_{brake}\cdot\frac{0.95\mu g}{a_{tot}}
$$

마지막으로 조향각·제동 토크를 $\text{LIM}$ 범위로 saturate한다.

---

## 4. 시뮬레이션 결과

### 4.1 P1 시나리오 benchmark — 베이스라인(OFF) vs 본인 설계(ON)

`run('scripts/grade.m')` 산출 KPI(14DOF plant, ode45). Δ는 OFF 대비 개선율(음수 = 개선).

| 시나리오 | KPI | 목표 | OFF | **ON** | Δ% | 점수 |
|---|---|---|---|---|---|---|
| **A3** Step Steer | yawRateOvershoot [%] | ≤10 | 2.70 | **2.12** | −21% | 4/4 |
| A3 | yawRateRiseTime [s] | ≤0.3 | 0.247 | **0.073** | −70% | 4/4 |
| A3 | yawRateSettling [s] | ≤0.8 | 1.462 | **0.72** | −51% | 4/4 |
| **A1** DLC | sideSlipMax [°] | ≤3 | 3.02 | **1.80** | −40% | 6/6 |
| A1 | LTR_max | ≤0.6 | 0.864 | **0.595** | −31% | 5/5 |
| A1 | lateralDevMax [m] | ≤0.7 | 1.827 | 2.164 | +18% | 0/4 |
| **A4** SS Circular | understeerGradient | 0.003±80% | 0.00075 | **0.00075** | — | 5/5 |
| A4 | sideSlipMax [°] | ≤2 | 1.18 | **1.14** | −3% | 5/5 |
| **A7** Brake-in-Turn | sideSlipMax [°] | ≤5 | 30.48 | **1.82** | −94% | 8/8 |
| A7 | LTR_max | ≤0.7 | 0.681 | **0.322** | −53% | 7/7 |
| **B1** Straight Brake | stoppingDistance [m] | ≤40 | 72.30 | 68.79 | −5% | 0/5 |
| B1 | absSlipRMS | ≤0.10 | 0.730 | **0.089** | −88% | 5/5 |
| **D1** DLC+Brake | sideSlipMax [°] | ≤4 | 4.91 | **2.14** | −56% | 4/4 |
| D1 | LTR_max | ≤0.6 | 0.864 | **0.496** | −43% | 2/2 |
| D1 | lateralDevMax [m] | ≤1.0 | 1.827 | 2.164 | +18% | 0/2 |

**정량 합계: 59 / 70.**

### 4.2 핵심 deep-dive — A7 Brake-in-Turn (가장 큰 개선)

A7은 선회 중 제동으로 베이스라인이 **sideSlipMax 30.5°** 에 달하는 사실상 스핀아웃 시나리오다. 본인 설계는 이를 **1.82°(−94%)** 로 억제했다. 핵심 메커니즘은 ESC β-limiter다 — β가 3°를 넘는 순간 운전자 반대 방향 $M_z$ 가 인가되고, coordinator가 이를 4륜 제동 차동으로 변환해 yaw를 적극 감쇠한다. 동시에 LTR도 0.681→0.322(−53%)로 전복 마진을 크게 확보했다.

### 4.3 그림

시나리오 형상은 `docs/figures/scenarios/`(예: `scn_A7_brake_in_turn.png`, `scn_A1_dlc.png`)에 수록돼 있다. OFF/ON trajectory·yaw rate 비교 plot은 다음 스크립트로 재생성 가능하다:

```matlab
[r_off,~] = run_icc_scenario('A1','14dof','Controller','off','SavePlot',false);
[r_on, ~] = run_icc_scenario('A1','14dof','Controller','on', 'SavePlot',false);
figure; plot(r_off.x_pos,r_off.y_pos,'r--', r_on.x_pos,r_on.y_pos,'b-', ...
             r_on.scenario.refPath(:,1),r_on.scenario.refPath(:,2),'k:');
axis equal; legend('off','on','ref'); xlabel('x [m]'); ylabel('y [m]');
```

---

## 5. 분석 + 한계

### 5.1 가장 성공적이었던 부분

**ESC β-limiter(A7·D1)와 yaw 추종(A3)** 이 가장 큰 효과를 냈다. A7에서 −94%, A3 settling −51%는 모델 기반 설계(적분 증강 MPC + β-limiter)가 transient·대슬립 모두에서 작동함을 보인다. 슬라이딩 모드 ABS도 absSlipRMS를 0.730→0.089(−88%)로 낮춰 휠 락을 효과적으로 방지했다.

### 5.2 가장 부족했던 부분 ① — lateralDevMax (A1·D1, 6점 미획득)

경로이탈은 ON(2.16 m)이 OFF(1.83 m)보다 **악화**되어 0점이 되었다. 원인을 sweep 실험으로 규명했다(`afs_max` 를 20°→5°로 변화):

| afs_max | A1 latDev [m] | A1 LTR |
|---|---|---|
| 20° (현재) | 2.164 | 0.595 ✅ |
| 10° | 2.025 | 0.645 ❌ |
| 5° | 1.913 | 0.745 ❌ |
| OFF(driver 단독) | 1.827 | 0.864 |

**구조적 원인**: ① 운전자 모델(Stanley path-follow)은 경로 오차를 피드백하는 완결된 경로추종기이지만, **본인 제어기의 입력에는 경로 오차(lateral deviation/refPath)가 전혀 포함되지 않는다.** 따라서 AFS는 경로와 무관한 yaw rate 목표를 추종하며 운전자 조향 위에 더해져, transient에서 같은 방향으로 과조향(over-cut)해 경로추종을 오히려 방해한다. ② AFS를 끄면(=OFF) 1.83 m가 하한이지만 이 역시 목표 1.4 m를 넘는다. 즉 **칼만필터·추가 MPC·학습 기반 기법을 동원해도** 제어기에 reference 경로가 주어지지 않는 한 경로추종 루프를 닫을 수 없다(관측가능성 한계). 이는 과제가 "경로추종 = 운전자, 안정성 = 제어기"로 역할을 분리한 결과이며, 안정성(slip·LTR 11점)을 위해 AFS를 켜는 것이 latDev(0점 불가피)를 포기하더라도 전체 점수상 우월하다.

### 5.3 가장 부족했던 부분 ② — B1 stoppingDistance (5점 미획득)

제동거리 68.8 m(목표 40 m)는 슬라이딩 모드 ABS가 **휠 락 방지를 우선해 제동 토크를 과도하게 해제**한 결과다(absSlipRMS는 통과). $\kappa_{ref}=-0.14$ 와 $u_{min}=-2000$ Nm가 release 쪽으로 보수적이어서, 슬립 안전과 제동력 사이 trade-off에서 슬립을 과보호했다. 향후 $\kappa_{ref}$ 를 마찰 최대 슬립(≈−0.1)에 더 근접시키고 release 한계를 조정하면 개선 여지가 있다.

### 5.4 더 시간이 있었다면

- B1 ABS의 $\kappa_{ref}$·release 게인 재튜닝으로 제동거리 단축(+5점 잠재).
- ctrl_vertical(CDC)을 C1/C2에서 정량 검증(가산점).
- coordinator에 WLS allocation 도입으로 마찰원 내 최적 분배(가산점).

---

## 6. 참고문헌

[1] ISO 3888-1:2018 — *Passenger cars — Test track for a severe lane-change manoeuvre — Part 1: Double lane-change.*
[2] ISO 4138:2021 — *Passenger cars — Steady-state circular driving behaviour.*
[3] R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012. (§3 yaw rate response, §8 ESC/yaw stability)
[4] D. Karnopp, M. J. Crosby, R. A. Harwood, "Vibration Control Using Semi-Active Force Generators," *J. Eng. Ind.*, 1974. (skyhook)
[5] J. M. Maciejowski, *Predictive Control with Constraints*, Prentice Hall, 2002. (MPC)
[6] J.-J. E. Slotine, W. Li, *Applied Nonlinear Control*, Prentice Hall, 1991. (sliding mode, boundary layer)

---

## 부록 A — 사용한 AI 도구

본 과제에서 **Claude (Claude Code)** 를 다음 범위로 사용했다:
- 코드베이스 구조·KPI 정의·채점 로직 파악 및 설명
- benchmark/grade 실행과 결과 해석
- **lateralDevMax 미달 원인 진단** — `afs_max` sweep 실험 설계·실행·분석 (§5.2의 구조적 한계 규명)
- 본 보고서 초안 작성

제어기 설계(기법 선택, 게인 값, 수식 유도)는 본인이 수행했으며, AI는 분석·검증·문서화 보조로 활용했다.

---

## 부록 B — sim_params.m 변경사항

`config/sim_params.m` 의 `CTRL.*`/`LIM.*` 는 **기본값을 그대로 사용**했다. 모든 제어 게인(MPC 가중치, ABS·CDC 파라미터)은 각 `ctrl_*.m` 내부에 설계값으로 직접 정의했으며(§3 참조), solver는 기본 `ode45` 를 사용했다.

function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL Method: Speed PI + Sliding Mode ABS (per-wheel)
%
%   Phase 2 (재작성):
%   1. Speed PI (정상). brake phase 트리거는 *driver 가 감속 요구할 때만* —
%      vxRef < vx - dead-zone 일 때만 brake_phase = true.
%   2. Sliding Mode ABS: brake_phase 일 때, 각 wheel slip 이 κ_ref 초과하면
%      smooth-sat sliding mode 로 release (PI 보다 빠른 응답).
%   3. κ_ref = -0.14, K_sm/K_p 적당히 낮춤 (lateral 시나리오 영향 최소화).

    if ~isfield(ctrlState, 'eIntV');    ctrlState.eIntV    = 0; end
    if ~isfield(ctrlState, 'FxPrev');   ctrlState.FxPrev   = 0; end
    if ~isfield(ctrlState, 'absInt');   ctrlState.absInt   = zeros(4,1); end
    if ~isfield(ctrlState, 'wheelSlip'); ctrlState.wheelSlip = zeros(4,1); end

    m_est       = 1500;
    Fx_max      = m_est * LIM.MAX_AX;
    Fx_max_rate = m_est * LIM.MAX_JERK;

    %% 1) Outer loop — Speed PI
    eV = vxRef - vx;
    Kp_v = 800; Ki_v = 200;
    ctrlState.eIntV = ctrlState.eIntV + eV * dt;
    Fx_pre = Kp_v * eV + Ki_v * ctrlState.eIntV;
    Fx = max(-Fx_max, min(Fx_max, Fx_pre));
    ctrlState.eIntV = ctrlState.eIntV + (Fx - Fx_pre) / max(Ki_v, 1e-6) * dt;
    ctrlState.eIntV = max(-CTRL.LON.intMax, min(CTRL.LON.intMax, ctrlState.eIntV));

    %% 2) Jerk limit
    dFx_cap = Fx_max_rate * dt;
    dFx = Fx - ctrlState.FxPrev;
    if abs(dFx) > dFx_cap
        Fx = ctrlState.FxPrev + sign(dFx) * dFx_cap;
    end
    ctrlState.FxPrev = Fx;

    %% 3) Sliding Mode ABS (per-wheel)
    %   Trigger: wheel slip 자체 (scenario 가 직접 brake 인가하는 B1 도 커버).
    %   Plant 의 brake torque 부호 규약: brake_total = brk_scenario + brakeESC,
    %   absBrakeMod < 0 → release (잠김 시 필요).
    %   dκ/dT_b < 0 이므로 슬라이딩 모드 u = +K_sm*sat(s/φ) + K_p*s.
    %   - s<0 (κ<κ_ref): u<0 → release ✓
    %   - s>0 (덜 잠김 / 가속 슬립): u>0 → min(0,·) 으로 zero (간섭 없음)
    kappa_ref = -0.14;
    K_sm      = 300;
    K_p_sm    = 2500;
    boundary  = 0.03;
    u_release_min = -2000;   % per-wheel 최대 release 토크 [Nm]
    slip_trigger  = -0.03;   % κ 가 이보다 음수면 ABS 활성

    kappa = ctrlState.wheelSlip(:);
    absBrakeMod = zeros(4, 1);

    for w = 1:4
        if kappa(w) < slip_trigger
            s = kappa(w) - kappa_ref;
            sat_s = max(-1, min(1, s / boundary));
            u = K_sm * sat_s + K_p_sm * s;
            u = min(0, u);                 % only release, never extra brake
            u = max(u_release_min, u);
            absBrakeMod(w) = u;
        end
    end

    %% Output
    forceCmd.Fx_total    = Fx;
    forceCmd.absBrakeMod = absBrakeMod;
    if Fx < 0
        forceCmd.brakeRatio = min(abs(Fx) / Fx_max, 1.0);
    else
        forceCmd.brakeRatio = 0;
    end

end

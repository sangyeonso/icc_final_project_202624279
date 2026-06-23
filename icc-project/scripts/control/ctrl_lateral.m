function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL Method: MPC with integrator augmentation (best of LQI + MPC preview)
%
%   State: x_aug = [vy; e_r; z]  where e_r = r - r_ref, z = ∫e_r dt
%   Cost:  J = Σ (Q_vy*vy² + Q_e*e² + Q_z*z²) + R*u²
%
%   - Integrator removes steady-state error (A4 manageable)
%   - Finite horizon prevents overshoot (A3 OS manageable)
%   - Combined benefit of LQI3 + MPC

    m  = 1500; Iz = 2500; lf = 1.2; lr = 1.4; Cf = 80000; Cr = 85000;
    vx_safe = max(abs(vx), 1.0);

    %% Cache prediction matrices
    rebuild = true;
    if isfield(ctrlState, 'mpc_vx') && isfield(ctrlState, 'mpc_K_x') ...
            && abs(ctrlState.mpc_vx - vx_safe) / vx_safe < 0.05
        rebuild = false;
    end

    if rebuild
        a11 = -(Cf + Cr) / (m * vx_safe);
        a12 = -vx_safe - (Cf * lf - Cr * lr) / (m * vx_safe);
        a21 = -(Cf * lf - Cr * lr) / (Iz * vx_safe);
        a22 = -(Cf * lf^2 + Cr * lr^2) / (Iz * vx_safe);
        A_c = [a11, a12; a21, a22];
        B_c = [Cf / m; Cf * lf / Iz];

        % Augmented continuous-time: x_aug = [vy; r; z]
        % dz/dt = r - r_ref = (r) - r_ref  → integrator state
        % In error coords (e_r = r - r_ref, assume r_ref constant in horizon):
        %   dvy/dt = a11*vy + a12*r + b1*u
        %   dr/dt  = a21*vy + a22*r + b2*u
        %   dz/dt  = r - r_ref = e_r
        % Define x_e = [vy; e_r; z] where e_r = r - r_ref
        %   dvy/dt = a11*vy + a12*(e_r + r_ref) + b1*u = a11*vy + a12*e_r + b1*u + a12*r_ref
        %   de_r/dt = a21*vy + a22*(e_r + r_ref) + b2*u = a21*vy + a22*e_r + b2*u + a22*r_ref
        %   dz/dt  = e_r
        % So A_aug acts on [vy; e_r; z] and B_aug acts on u (ignoring constant r_ref drift for cost)
        A_aug = [a11, a12, 0;
                 a21, a22, 0;
                 0,   1,   0];
        B_aug = [Cf/m; Cf*lf/Iz; 0];

        dt_mpc = 0.005;
        A_d = eye(3) + A_aug * dt_mpc;
        B_d = B_aug * dt_mpc;

        N = 15;
        Phi   = zeros(3*N, 3);
        Gamma = zeros(3*N, N);
        A_pow = eye(3);
        for i = 1:N
            A_pow = A_d * A_pow;
            Phi((i-1)*3+1 : i*3, :) = A_pow;
            for j = 1:i
                Gamma((i-1)*3+1 : i*3, j) = A_d^(i-j) * B_d;
            end
        end

        % Cost: penalize all 3 states (LQI3 weights)
        Q_per_step = diag([5, 200, 5]);    % [vy, e_r, z] — matches LQI3
        R_u        = 0.5;

        Q     = kron(eye(N), Q_per_step);
        R_bar = R_u * eye(N);

        H_inv = inv(Gamma' * Q * Gamma + R_bar);
        K_x_full = H_inv * Gamma' * Q * Phi;

        ctrlState.mpc_vx  = vx_safe;
        ctrlState.mpc_K_x = K_x_full(1, :);   % 1 × 3
        ctrlState.mpc_N   = N;
    end

    %% Integrator update (z = ∫e_r dt)
    if ~isfield(ctrlState, 'zInt'); ctrlState.zInt = 0; end
    e_r = yawRate - yawRateRef;
    ctrlState.zInt = ctrlState.zInt + e_r * dt;
    z_max = 1.0;
    ctrlState.zInt = max(-z_max, min(z_max, ctrlState.zInt));

    %% Current state
    vy_est = slipAngle * vx_safe;
    x_aug  = [vy_est; e_r; ctrlState.zInt];

    %% Closed-form MPC: u = -K_x * x_aug
    delta_AFS = -ctrlState.mpc_K_x * x_aug;

    afs_max = deg2rad(20);   % Phase 3: 5°→20° (LTR 공략 sweet spot)
    u_pre   = delta_AFS;
    u_post  = afs_max * tanh(u_pre / afs_max);
    if abs(u_pre - u_post) > 1e-6 && abs(ctrlState.mpc_K_x(3)) > 1e-6
        ctrlState.zInt = ctrlState.zInt + (u_post - u_pre) / ctrlState.mpc_K_x(3) * dt;
        ctrlState.zInt = max(-z_max, min(z_max, ctrlState.zInt));
    end
    delta_AFS = u_post;

    %% ESC β-limiter
    beta_th = deg2rad(3.0);
    Kbeta   = 8000;
    if abs(slipAngle) > beta_th
        betaExc = slipAngle - sign(slipAngle) * beta_th;
        M_z = -Kbeta * betaExc;
    else
        M_z = 0;
    end
    Mz_max = 5000;
    M_z = max(-Mz_max, min(Mz_max, M_z));

    deltaAdd.steerAngle = delta_AFS;
    deltaAdd.yawMoment  = M_z;

end

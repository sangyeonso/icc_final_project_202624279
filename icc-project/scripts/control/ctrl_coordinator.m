function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR Method: Mz allocation + friction-circle aware + ABS pass-through

    actuatorCmd.steerAngle = max(-LIM.MAX_STEER_ANGLE, ...
                                  min(LIM.MAX_STEER_ANGLE, latCmd.steerAngle));

    Mz = latCmd.yawMoment;
    halfTrack_f = VEH.track_f / 2;
    halfTrack_r = VEH.track_r / 2;

    ratio_f = 0.6;
    ratio_r = 1 - ratio_f;

    dTf = -Mz * ratio_f / (2 * halfTrack_f);
    dTr = -Mz * ratio_r / (2 * halfTrack_r);
    Tdiff = [-dTf;  +dTf;  -dTr;  +dTr];

    if isfield(lonCmd, 'Fx_total') && lonCmd.Fx_total < 0
        T_total = abs(lonCmd.Fx_total) * VEH.rw;
        Tbrake_eq = [ratio_f * T_total / 2;
                     ratio_f * T_total / 2;
                     ratio_r * T_total / 2;
                     ratio_r * T_total / 2];
    else
        Tbrake_eq = zeros(4, 1);
    end

    Tbrake = Tbrake_eq + Tdiff;

    if isfield(lonCmd, 'absBrakeMod')
        Tbrake = Tbrake + lonCmd.absBrakeMod(:);
    end

    mu = 1.0; g = 9.81;
    if isfield(lonCmd, 'Fx_total')
        ax_est = lonCmd.Fx_total / VEH.mass;
    else
        ax_est = 0;
    end
    ay_est = Mz / VEH.Iz * vx;
    a_total = sqrt(ax_est^2 + ay_est^2);
    a_max   = mu * g * 0.95;
    if a_total > a_max
        scale = a_max / a_total;
        Tbrake = Tbrake * scale;
    end

    actuatorCmd.brakeTorque = max(-LIM.MAX_BRAKE_TRQ, ...
                                   min(LIM.MAX_BRAKE_TRQ, Tbrake));

    actuatorCmd.dampingCoeff = max(CTRL.VER.cMin, ...
                                    min(CTRL.VER.cMax, verCmd));
end

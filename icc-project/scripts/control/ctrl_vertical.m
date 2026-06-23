function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL Method: Hybrid Skyhook-Groundhook

    if nargin < 4; dt = 0.001; end %#ok<NASGU>

    cMin   = CTRL.VER.cMin;
    cMax   = CTRL.VER.cMax;
    cNom   = (cMin + cMax) / 2;
    K_sky  = CTRL.VER.skyGain;
    K_gnd  = 0.5 * K_sky;
    alpha  = 0.7;

    dampingCmd = cNom * ones(4, 1);

    if ~isfield(suspState, 'zs_dot') || ~isfield(suspState, 'zu_dot')
        return;
    end

    for i = 1:4
        zsd  = suspState.zs_dot(i);
        zud  = suspState.zu_dot(i);
        zrel = zsd - zud;
        eps_v = 0.01;

        if zsd * zrel > 0
            c_sky = K_sky * abs(zsd) / (abs(zrel) + eps_v);
        else
            c_sky = cMin;
        end

        if zud * zrel < 0
            c_gnd = K_gnd * abs(zud) / (abs(zrel) + eps_v);
        else
            c_gnd = cMin;
        end

        c_hyb = alpha * c_sky + (1 - alpha) * c_gnd;
        dampingCmd(i) = max(cMin, min(cMax, c_hyb));
    end
end

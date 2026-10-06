function hi_valid = valid_sweep_limit(param_name, lo, hi, amp_base, wavelen_base, nodes, r0)
% Largest value in [lo, hi] of the swept parameter ("amplitude" or "wavelength") for
% which BOTH gaits are valid (snake_gait_check), with the other parameters at baseline.
% The scan uses 100 values; validity changes only once along each sweep.
vals = linspace(lo, hi, 100);
hi_valid = NaN;
for v = vals
    switch param_name
        case "amplitude",  a = v;        w = wavelen_base;
        case "wavelength", a = amp_base; w = v;
        otherwise, error('valid_sweep_limit: only amplitude or wavelength');
    end
    g1 = snake_gait_check(a, w, 1, nodes, r0);
    g2 = snake_gait_check(a, w, 2, nodes, r0);
    if g1.is_valid && g2.is_valid
        hi_valid = v;
    elseif ~isnan(hi_valid)
        break;      % first invalid value after valid ones: stop
    end
end
end

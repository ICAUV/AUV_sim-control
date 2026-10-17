function n_p = propellerSpeed(n_p, n_d, model)
% n_p = propellerSpeed(n_p, n_d, model) moves the propeller speed n_p (RPM)
% towards the desired speed n_d, rate limited by model.n_rate and saturated
% at model.n_max.

if n_p < n_d
    n_p = n_p + model.n_rate;
elseif n_p > n_d
    n_p = n_p - model.n_rate;
end
n_p = sat(n_p, model.n_max);

end

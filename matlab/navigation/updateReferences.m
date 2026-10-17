function ref = updateReferences(ref, cmd, ControlFlag, ctrl, guidance, h)
% ref = updateReferences(ref, cmd, ControlFlag, ctrl, guidance, h) propagates
% the desired states for the tracking controller to the next sample, from the
% navigation commands cmd:
%
%   ControlFlag 1-2 : Heading reference model
%   ControlFlag 3   : LOS observer for heading, reference model for depth
%   ControlFlag 4   : LOS observers for heading and pitch
%
% The desired pitch angle of ControlFlag 1-3 comes from depthController.m.

switch ControlFlag

    case {1, 2}
        [ref.psi_d, ref.r_d, ref.a_d] = refModel(ref.psi_d, ref.r_d, ref.a_d, ...
            cmd.psi_ref, ctrl.r_max, ctrl.zeta_d_psi, ctrl.wn_d_psi, h, 1);

    case 3
        [ref.psi_d, ref.r_d] = LOSobserver(ref.psi_d, ref.r_d, cmd.psi_ref, h, guidance.K_f);
        ref.r_d = sat(ref.r_d, ctrl.r_max);

        % a_d is shared with the heading autopilot, as in the original loop
        [ref.z_d, ref.zdot_d, ref.a_d] = refModel(ref.z_d, ref.zdot_d, ref.a_d, ...
            cmd.z_ref, ctrl.w_max, ctrl.zeta_d_z, ctrl.wn_d_z, h, 0);

    otherwise
        [ref.theta_d, ref.q_d] = LOSobserver(ref.theta_d, ref.q_d, cmd.theta_ref, h, guidance.K_f);
        [ref.psi_d, ref.r_d] = LOSobserver(ref.psi_d, ref.r_d, cmd.psi_ref, h, guidance.K_f);
        ref.r_d = sat(ref.r_d, ctrl.r_max);
end

end

function fanat = ActiveCspace_for_AnatomyRichnessCalculation(step_a23,wi_best,xi_ai,Pi,gsCoMi0,Mbi)
% Calculates the mean DCI value of anatomy, given the min and max DCI
% values in the C-space of the anatomy investigated

p2 = 0;
for ta2=-1.5708:step_a23:1.5708
    p2 = p2+1;
    p3 = 0;
    for ta3=-1.5708:step_a23:1.5708
        p3 = p3+1;
        q = [0.1 ta2 ta3];
        exp_ai(:,:,1) = twistexp(xi_ai(:,1),q(1));
        exp_ai(:,:,2) = twistexp(xi_ai(:,2),q(2));
        exp_ai(:,:,3) = twistexp(xi_ai(:,3),q(3));
        % Compute Link CoM Jacobians
        for i=1:3
            [Jbsli(:,:,i)] = Jbody_CoM_3DoF_forDCI(xi_ai, exp_ai, Pi, gsCoMi0, i);
        end
        % Compute mass matrix
        [Mb] = CalculateOnlyBodyMassMatrix(Jbsli,Mbi);
        % DCI
        wi = [wi_best(1) wi_best(2) wi_best(3) wi_best(1) wi_best(2) wi_best(3)];
        [DCI(p2,p3)] = CalculateDynamicConditioningIndex2_3DoF(Mb,3,wi);
    end
end

fmin = min(DCI(:)); fmax = max(DCI(:));
fanat = (fmax+fmin)/2;

end
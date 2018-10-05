function [z_mu, z_Sigma] = kf_predict_obs(kf)
    
    mu = kf.mu_preds(:,end);
    Sigma = kf.Sigma_preds(:,:,end);

    z_mu = kf.H * mu;
    z_Sigma = kf.H * Sigma * kf.H' + kf.Sigma_z;
end

%-------------------------------------------------------
% samples from a multivariate gaussian with given mean and covariance
%-------------------------------------------------------
function s = sample(mu, Sigma)

[V,D] = eig(Sigma);

s=mu+V*sqrt(D)*randn(length(mu),1);

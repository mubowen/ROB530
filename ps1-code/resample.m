function [newSamples, newWeight] = resample(samples, weight)

% do some stuff here
W=cumsum(weight);
n=size(samples,2);
r=rand/n;
j=1;
newSamples = zeros(size(samples));
newWeights = zeros(size(weight));
for i =1:n
    u = r + (i-1) / n;
    while u > W(j)
        j = j + 1;
    end
    newSamples(:,i) = samples(:,j);
    newWeight(i) = 1/n;
end
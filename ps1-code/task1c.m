clc
clear
%%probelm3
 
for location=0:9
%initial setting
n=1;
p=[0.8,0.4,0.4,0.8,0.4,0.4,0.8,0.4,0.4,0.4];
bel_x=0.1;
p_m=1;
%first prediction and Correction
pz_x=p(location+1);
belx=n*pz_x*bel_x;
bel_x=p_m*belx;
%second move
location =3+location;
if location>9
   location =location-10;  
end
pz_x=p(location+1);
belx=n*pz_x*bel_x;
bel_x=p_m*belx;
%third move
location =4+location;
if location>9
   location =location-10;
end
pz_x=1-p(location+1);
belx=n*pz_x*bel_x;
bel_x=p_m*belx;
%save result
collect(location+1)=belx;
end
%calculate the normalizer to make sume of belx is 1
n=1/sum(collect);
answer=collect*n;
plot(answer,'o')
title('Task1_c distribution plot')
 
xlabel('base')
ylabel('possibility')

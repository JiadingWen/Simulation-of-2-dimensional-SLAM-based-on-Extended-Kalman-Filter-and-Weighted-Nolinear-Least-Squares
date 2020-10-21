run F01_GenerateData.m

%% 
time_start = cputime;
run F00_Main_EKF.m
time_used_KF = cputime - time_start;

%%
time_start = cputime;
run F00_Main_NLS.m
time_used_LS = cputime - time_start;

%%
disp(time_used_KF - time_used_LS);
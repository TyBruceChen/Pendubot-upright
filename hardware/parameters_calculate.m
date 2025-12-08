T = 1e-3; % Sample time (for simulating A/D and D/A discrete-time effects)

%%
u_array  = OL_TO();

%%
K=k_PFL_LQR();
Tsim = 5;

save('Config.mat', 'u_array', 'K', 'Tsim', 'T');




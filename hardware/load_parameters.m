%%
load('Config.mat')
model = 'hardware';
load_system(model);
set_param([model '/Sample Time'], 'Value', num2str(Tsim));
set_param([model '/K'], 'Value', num2str(K));
set_param([model '/TO'], 'Value', mat2str(u_array));
set_param(['hardware' '/Time'], 'SampleTime', num2str(T));
clear;
sim_data = readmatrix('sim_result.dat');
%%
flag = 'quat';%'euler','quat'
t = sim_data(:,1);
states = sim_data(:,2:end);
hopter_plot(t,states,flag);
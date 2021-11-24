clear;
sim_data = readmatrix('../build/sim_result.dat');
%%
t = sim_data(:,1);
states = sim_data(:,2:end);
hopter_plot(t,states);
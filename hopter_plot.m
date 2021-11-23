function hopter_plot(t,states,flag)
if strcmp(flag,'euler')
    pitch = states(:,1);
    roll = states(:,2);
    yaw = mod(states(:,3),2*pi);
    wB = states(:,4:6);
    wQ = [wB(:,1).*cos(yaw) - wB(:,2).*sin(yaw),wB(:,1).*sin(yaw) + wB(:,2).*cos(yaw),-(wB(:,1).*sin(yaw) + wB(:,2).*cos(yaw)).*tan(roll)];
    qQ = states(:,7:10);
    theta = states(:,11:16);
    [pitchQ,rollQ,yawQ] = quat2angle(qQ,"YXZ");
else
    qB = states(:,1:4);
    [pitch,roll,yaw] = quat2angle(qB,"YXZ");
    wB = states(:,5:7);
    wQ = [wB(:,1).*cos(yaw) - wB(:,2).*sin(yaw),wB(:,1).*sin(yaw) + wB(:,2).*cos(yaw),-(wB(:,1).*sin(yaw) + wB(:,2).*cos(yaw)).*tan(roll)];
    qQ = states(:,8:11);
    theta = states(:,12:17);
    [pitchQ,rollQ,yawQ] = quat2angle(qQ,"YXZ");
end
    marker_nums = 10;
    marker_step = floor(size(states,1)/marker_nums);
    marker_index = 1:marker_step:size(states,1);
    %%
    figure("Name","Euler Angle Body Fixed","Position",[100,100,900,800]);
    subplot(3,1,1)
    plot(t,rad2deg(pitch),'-*r',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
    grid on;
    ax = gca;
    ax.TickLabelInterpreter = "latex";
    ax.FontSize = 20;
    ylabel('$\theta \rm{[deg]}$','Interpreter','latex','FontSize',20);
    
    subplot(3,1,2)
    plot(t,rad2deg(roll),'-.sb',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
    grid on;
    ax = gca;
    ax.TickLabelInterpreter = "latex";
    ax.FontSize = 20;
    ylabel('$\phi \rm{[deg]}$','Interpreter','latex','FontSize',20);
    
    subplot(3,1,3)
    plot(t,rad2deg(yaw),'--hg',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
    grid on;
    ax = gca;
    ax.TickLabelInterpreter = "latex";
    ax.FontSize = 20;
    xlabel('$t\rm{[s]}$','Interpreter','latex','FontSize',20);
    ylabel('$\psi \rm{[deg]}$','Interpreter','latex','FontSize',20);
    %%
    figure("Name","Euler Angle Quasi Body Fixed","Position",[100,100,900,800]);
    subplot(3,1,1)
    plot(t,rad2deg(pitchQ),'-*r',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
    grid on;
    ax = gca;
    ax.TickLabelInterpreter = "latex";
    ax.FontSize = 20;
    ylabel('$\theta_Q \rm{[deg]}$','Interpreter','latex','FontSize',20);
    
    subplot(3,1,2)
    plot(t,rad2deg(rollQ),'-.sb',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
    grid on;
    ax = gca;
    ax.TickLabelInterpreter = "latex";
    ax.FontSize = 20;
    ylabel('$\phi_Q \rm{[deg]}$','Interpreter','latex','FontSize',20);
    
    subplot(3,1,3)
    plot(t,rad2deg(yawQ),'--hg',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
    grid on;
    ax = gca;
    ax.TickLabelInterpreter = "latex";
    ax.FontSize = 20;
    xlabel('$t\rm{[s]}$','Interpreter','latex','FontSize',20);
    ylabel('$\psi_Q \rm{[deg]}$','Interpreter','latex','FontSize',20);

    %%
    figure("Name","Angular Rate Fixed Body","Position",[100,100,900,800]);
    subplot(3,1,1)
    plot(t,rad2deg(wB(:,1)),'-*r',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
    grid on;
    ax = gca;
    ax.TickLabelInterpreter = "latex";
    ax.FontSize = 20;
    ylabel('$\omega_{\mathcal{B}_x} \rm{[deg/s]}$','Interpreter','latex','FontSize',20);
    
    subplot(3,1,2)
    plot(t,rad2deg(wB(:,2)),'-.sb',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
    grid on;
    ax = gca;
    ax.TickLabelInterpreter = "latex";
    ax.FontSize = 20;
    ylabel('$\omega_{\mathcal{B}_y} \rm{[deg/s]}$','Interpreter','latex','FontSize',20);
    
    subplot(3,1,3)
    plot(t,wB(:,3),'--hg',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
    grid on;
    ax = gca;
    ax.TickLabelInterpreter = "latex";
    ax.FontSize = 20;
    xlabel('$t\rm{[s]}$','Interpreter','latex','FontSize',20);
    ylabel('$\omega_{\mathcal{B}_z} \rm{[deg/s]}$','Interpreter','latex','FontSize',20);

    %%
%     figure("Name","Angular Rate Quasi Fixed Body","Position",[100,100,900,800]);
%     subplot(3,1,1)
%     plot(t,rad2deg(wQ(:,1)),'-*r',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
%     grid on;
%     ax = gca;
%     ax.TickLabelInterpreter = "latex";
%     ax.FontSize = 20;
%     ylabel('$\omega_{\mathcal{Q}_x} \rm{[deg/s]}$','Interpreter','latex','FontSize',20);
%     
%     subplot(3,1,2)
%     plot(t,rad2deg(wQ(:,2)),'-.sb',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
%     grid on;
%     ax = gca;
%     ax.TickLabelInterpreter = "latex";
%     ax.FontSize = 20;
%     ylabel('$\omega_{\mathcal{Q}_y} \rm{[deg/s]}$','Interpreter','latex','FontSize',20);
%     
%     subplot(3,1,3)
%     plot(t,wQ(:,3),'--hg',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
%     grid on;
%     ax = gca;
%     ax.TickLabelInterpreter = "latex";
%     ax.FontSize = 20;
%     xlabel('$t\rm{[s]}$','Interpreter','latex','FontSize',20);
%     ylabel('$\omega_{\mathcal{Q}_z} \rm{[deg/s]}$','Interpreter','latex','FontSize',20);

    %%
    figure("Name","Estimation Value","Position",[100,100,1200,600]);
    subplot(3,2,1)
    plot(t,theta(:,1),'-*r',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
    grid on;
    ax = gca;
    ax.TickLabelInterpreter = "latex";
    ax.FontSize = 20;
    ylabel('$\theta_1$','Interpreter','latex','FontSize',20);
    
    subplot(3,2,2)
    plot(t,theta(:,2),'-.sb',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
    grid on;
    ax = gca;
    ax.TickLabelInterpreter = "latex";
    ax.FontSize = 20;
    ylabel('$\theta_2$','Interpreter','latex','FontSize',20);
    
    subplot(3,2,3)
    plot(t,theta(:,3),'--hg',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
    grid on;
    ax = gca;
    ax.TickLabelInterpreter = "latex";
    ax.FontSize = 20;
    ylabel('$\theta_3$','Interpreter','latex','FontSize',20);

    subplot(3,2,4)
    plot(t,theta(:,4),'--hg',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
    grid on;
    ax = gca;
    ax.TickLabelInterpreter = "latex";
    ax.FontSize = 20;
    ylabel('$\theta_4$','Interpreter','latex','FontSize',20);

    subplot(3,2,5)
    plot(t,theta(:,5),'--hg',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
    grid on;
    ax = gca;
    ax.TickLabelInterpreter = "latex";
    ax.FontSize = 20;
    xlabel('$t\rm{[s]}$','Interpreter','latex','FontSize',20);
    ylabel('$\theta_5$','Interpreter','latex','FontSize',20);

    subplot(3,2,6)
    plot(t,theta(:,6),'--hg',"LineWidth",4,"MarkerIndices",marker_index,"MarkerSize",12);
    grid on;
    ax = gca;
    ax.TickLabelInterpreter = "latex";
    ax.FontSize = 20;
    xlabel('$t\rm{[s]}$','Interpreter','latex','FontSize',20);
    ylabel('$\theta_6$','Interpreter','latex','FontSize',20);

end
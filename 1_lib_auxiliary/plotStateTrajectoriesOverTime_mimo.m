function [fig_X, fig_U] = plotStateTrajectoriesOverTime_mimo(X, U, chabo_constraints)
    N = size(X,2);
    
    fig_X = figure;
    t = 0.02*(0:N-1);
    subplot(3,1,1),
    plot(t, X(1,:)); hold on;
    xlabel('time [s]');
    ylabel('pitch [rad]');
    subplot(3,1,2);
    plot(t, X(3,:)); hold on;
    xlabel('time [s]');
    ylabel('position [m]');
    subplot(3,1,3);
    plot(t, X(5,:)); hold on;
    xlabel('time [s]');
    ylabel('velocity [$\frac{rad}{sec}$]');
    yline(chabo_constraints.flywheelVelocityMax, '--k', 'LineWidth', 3);
    yline(-chabo_constraints.flywheelVelocityMax, '--k', 'LineWidth', 3);
    
    fig_U = figure;
    subplot(2,1,1);  
    plot(t, U(1,:)); hold on;
    xlabel('time [s]');
    ylabel('torque [Nm]');
    yline(chabo_constraints.tiresTorqueMax, '--k', 'LineWidth', 3);
    yline(-chabo_constraints.tiresTorqueMax, '--k', 'LineWidth', 3);
    legend('$u_T$');
    
    subplot(2,1,2);
    plot(t, U(2,:)); hold on;
    xlabel('time [s]');
    ylabel('torque [Nm]');
    yline(chabo_constraints.flywheelTorqueMax, '--k', 'LineWidth', 3);
    yline(-chabo_constraints.flywheelTorqueMax, '--k', 'LineWidth', 3);
    legend('$u_F$');
    
end


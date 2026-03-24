function [infig, outfig] = plot_test_data(filename, varargin)
    % fig = plot_test_data(filename, [mdm, dt, kp, ki]);
    
    mdm = false;
    if nargin > 1
        mdm = varargin{1};
    end
    
    dt = 1;
    if nargin > 2
        dt = varargin{2};
    end
    
    kp = 1;
    if nargin > 3
        kp = varargin{3};
    end
    
    ki = 1;
    if nargin > 4
        ki = varargin{4};
    end
    
    % should be six column csv file with accelerometer and gyroscope data, sampled at 50 Hz:
    % ax, ay, az, wx, wy, wz
    data = readtable(filename);
    data = table2array(data);
    
    a = data(:, 1:3).';
    w = data(:, 4:6).';
    
    down = a(:, 1);
    down = down / norm(down);
    
    if ~mdm
        d = test_mahoney(a, w, dt, kp, ki);
    else
        d = test_mdm(a, w, dt, kp, ki);
    end
    
    s = size(d);
    t = (0:s(2)-1) / 50;
    
    infig = figure();
    subplot(2, 1, 1);
    plot(t, a.', 'LineWidth', 1.5);
    grid on;
    legend('a_x', 'a_y', 'a_z');
    ylabel('acceleration (m / s^2)');
    title('Accelerometer and Gyroscope Input Data')
    
    subplot(2, 1, 2);
    plot(t, w.', 'LineWidth', 1.5);
    grid on;
    legend('\omega_x', '\omega_y', '\omega_z');
    xlabel('time (s)');
    ylabel('angular velocity (rad / s)');
    
    outfig = figure();
    hold on;
    yline(down, 'k--', 'LineWidth', 1.5);
    plot(t, d.', 'LineWidth', 1.5);
    grid on;
    legend('down', '', '', '\delta_x', '\delta_y', '\delta_z');
    xlabel('time (s)');
    ylabel('down vector');
    title(['Mahoney Filter Output Data', newline(), 'k_p = ', num2str(kp), ', k_i = ', num2str(ki)]);
end
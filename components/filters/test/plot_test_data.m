function [outfig, infig] = plot_test_data(filename, varargin)
    % [outfig, infig] = plot_test_data(filename, [mdm, dt, kp, ki, fc]);
    
    %% parse input args
    
    if (~ischar(filename) && ~isstring(filename)) || ~isvector(filename)
        error([newline(), ...
            '    InvalidInput: filename must be valid string or char!']);
    end
    
    valid = cellfun(@(input) isscalar(input) && (isnumeric(input) || ...
        islogical(input)), varargin);
    
    mdm = false;
    if nargin > 1 && valid(1)
        mdm = varargin{1};
    end
    
    dt = 1;
    if nargin > 2 && valid(2)
        dt = varargin{2};
    end
    
    kp = 1;
    if nargin > 3 && valid(3)
        kp = varargin{3};
    end
    
    ki = 1;
    if nargin > 4 && valid(4)
        ki = varargin{4};
    end
    
    fc = inf;
    tofilter = false;
    if nargin > 5 && valid(5)
        tofilter = true;
        fc = varargin{5};
    end
    
    %% read, parse and filter data
    
    % should be six column csv file with accelerometer and gyroscope data,
    % sampled at 50 Hz:
    % ax, ay, az, wx, wy, wz
    data = readtable(filename);
    data = table2array(data);
    
    a = data(:, 1:3).';
    w = data(:, 4:6).';
    
    if tofilter
        [alpha, beta] = getbutter(fc, dt, 4);
        a = filter(beta, alpha, a, [], 2);
        w = filter(beta, alpha, w, [], 2);
    end
    
    down = a(:, 1);
    down = down / norm(down);
    
    if ~mdm
        title_name = 'Mahoney Filter Output Data';
        d = test_mahoney(a, w, dt, kp, ki);
    else
        title_name = 'MDM Filter Output Data';
        d = test_mdm(a, w, dt, kp, ki);
    end
    
    %% plot input data
    
    s = size(d);
    t = (0:s(2)-1) * dt;
    
    if nargout == 2
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
    end
    
    %% plot output data
    
    outfig = figure();
    hold on;
    yline(down, 'k--', 'LineWidth', 1.5);
    plot(t, d.', 'LineWidth', 1.5);
    grid on;
    legend('down', '', '', '\delta_x', '\delta_y', '\delta_z');
    xlabel('time (s)');
    ylabel('down vector');
    title([title_name, newline(), 'k_p = ', num2str(kp), ', k_i = ', ...
        num2str(ki), ', f_c = ', num2str(fc)]);
end

function plot_feet_com_steps(dir)
%
% dir - path to the directory containing logs.
%
figure ('Position', get(0,'Screensize')*0.9);
run (strcat(dir, '/oru_steps.m'))

try
    load (strcat(dir, '/oru_com.log'));
    load (strcat(dir, '/oru_feet.log'));

    hold on;
    if exist('oru_com')
        CoM_expected = oru_com(:, 1:3);
        CoM_sensor = oru_com(:, 4:6);
        mean_z = mean ([CoM_expected(:,3); CoM_sensor(:,3)]);
        plot3 (CoM_expected(:,1), CoM_expected(:,2), CoM_expected(:,3) - mean_z, 'b');
        plot3 (CoM_sensor(:,1), CoM_sensor(:,2), CoM_sensor(:,3) - mean_z, 'r');
    end

    if exist('oru_feet')
        l_expected = oru_feet(:, 1:3);
        l_real = oru_feet(:, 4:6);
        r_expected = oru_feet(:, 7:9);
        r_real = oru_feet(:, 10:12);
        plot3 (l_expected(:,1), l_expected(:,2), l_expected(:,3), 'b');
        plot3 (l_real(:,1), l_real(:,2), l_real(:,3), 'r');
        plot3 (r_expected(:,1), r_expected(:,2), r_expected(:,3), 'b');
        plot3 (r_real(:,1), r_real(:,2), r_real(:,3), 'r');
    end
    hold off;
catch
end

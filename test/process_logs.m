function process_logs(dir, plot_velocities)
%
% dir - path to the directory containing logs.
% plot_velocities - controls the plotting of velocities (1=true, 0=false)
%


Names = {
' 1 L HIP YAW PITCH' \
' 2 L HIP ROLL' \
' 3 L HIP PITCH' \
' 4 L KNEE PITCH' \
' 5 L ANKLE PITCH' \
' 6 L ANKLE ROLL' \
' 7 R HIP YAW PITCH ' \
' 8 R HIP ROLL' \
' 9 R HIP PITCH' \
'10 R KNEE PITCH' \
'11 R ANKLE PITCH' \
'12 R ANKLE ROLL' \
'13 L SHOULDER PITCH' \
'14 L SHOULDER ROLL' \
'15 L ELBOW YAW' \
'16 L ELBOW ROLL' \
'17 L WRIST YAW' \
'18 R SHOULDER PITCH' \
'19 R SHOULDER ROLL' \
'20 R ELBOW YAW' \
'21 R ELBOW ROLL' \
'22 R WRIST YAW' \
'23 HEAD PITCH' \
'24 HEAD YAW'};


PlotSensors = {[5 11] [6 12] [3 9] [2 8] [4 10] [1]};

%   min     max     max_velocity
Limits = [
  -1.14530299999999996   0.74080999999999997   4.16173744201999973;
  -0.37947199999999998   0.79047699999999999   4.16173744201999973;
  -1.77391199999999993   0.48409000000000002   6.40239143372000008;
  -0.09234600000000000   2.11252800000000018   6.40239143372000008;
  -1.18951600000000002   0.92274699999999998   6.40239143372000008;
  -0.76900100000000005   0.39788000000000001   4.16173744201999973;
  -1.14530299999999996   0.74080999999999997   4.16173744201999973;
  -0.73832100000000001   0.41475400000000001   4.16173744201999973;
  -1.77230799999999999   0.48562400000000000   6.40239143372000008;
  -0.10308299999999999   2.12019799999999980   6.40239143372000008;
  -1.18644799999999995   0.93205600000000000   6.40239143372000008;
  -0.38867600000000002   0.78587499999999999   4.16173744201999973;
  -2.08559999999999990   2.08559999999999990   8.26797389983999942;
   0.00870000000000000   1.64939999999999998   7.19407272338999970;
  -2.08559999999999990   2.08559999999999990   8.26797389983999942;
  -1.56210000000000004  -0.00870000000000000   7.19407272338999970;
  -1.82380000000000009   1.82380000000000009   5.03143501282000027;
  -2.08559999999999990   2.08559999999999990   8.26797389983999942;
  -1.64939999999999998  -0.00870000000000000   7.19407272338999970;
  -2.08559999999999990   2.08559999999999990   8.26797389983999942;
   0.00870000000000000   1.56210000000000004   7.19407272338999970;
  -1.82380000000000009   1.82380000000000009   5.03143501282000027;
  -2.08570000000000011   2.08570000000000011   7.19407272338999970;
  -0.67200000000000004   0.51490000000000002   8.26797389983999942;
];


NumJoints = size(Names, 2);


try
    load (strcat(dir, '/oru_joints.log'));

    if exist('oru_joints')

        SensorValues = oru_joints(:,1:NumJoints);
        ActuatorValues = oru_joints(:,NumJoints+1:NumJoints*2);

        for i = 1:size(PlotSensors, 2);
            figure ('Position', get(0,'Screensize')*0.9);
            for j = 1:length(PlotSensors{i});
                subplot (length(PlotSensors{i}), 1, j);
                hold on;
                jointId = PlotSensors{i}(j);
                title (Names{1,jointId});
                plot (ActuatorValues(:,jointId), 'b');
                plot (SensorValues(:,jointId), 'r');
                axis ([0, size(SensorValues,1), Limits(jointId,1), Limits(jointId,2)]);
                legend ('Expected values', 'Sensor data')
                hold off;
            end
        end
        % close all
    end
catch
end

if (plot_velocities == 1)
    try
        load (strcat(dir, '/oru_joint_velocities.log'));

        if exist('oru_joints')
            Velocities = oru_joint_velocities(:,1:NumJoints);
            for i = 1:size(PlotSensors, 2);
                figure ('Position', get(0,'Screensize')*0.9);
                for j = 1:length(PlotSensors{i});
                    subplot (length(PlotSensors{i}), 1, j);
                    hold on;
                    jointId = PlotSensors{i}(j);
                    title (Names{1,jointId});

                    plot (Velocities(:,jointId), 'b');
                    plot ([0 size(Velocities, 1)], [Limits(jointId,3) Limits(jointId,3)], 'r');
                    legend ('Velocity', 'Limit')
                    hold off;
                end
            end
        end
    catch
    end
end


try
    load (strcat(dir, '/oru_com.log'));

    if exist('oru_com')
        CoM_expected = oru_com(:, 1:3);
        CoM_sensor = oru_com(:, 4:6);
        figure ('Position', get(0,'Screensize')*0.9);
        hold on;
        title ('CoM');
        plot3 (CoM_expected(:,1), CoM_expected(:,2), CoM_expected(:,3), 'b');
        plot3 (CoM_sensor(:,1), CoM_sensor(:,2), CoM_sensor(:,3), 'r');
        legend ('Expected', 'Computed from sensor data')
        hold off;

        figure ('Position', get(0,'Screensize')*0.9);
        hold on;
        title ('Error in CoM position');
        plot(oru_com (:,1) - oru_com(:,4), 'b')
        plot(oru_com (:,2) - oru_com(:,5), 'r')
        plot(oru_com (:,3) - oru_com(:,6), 'k')
        legend ('along X axis', 'along Y axis', 'along Z axis');
        hold off;
    end
catch
end


try
    load (strcat(dir, '/oru_feet.log'));

    if exist('oru_feet')
        l_expected = oru_feet(:, 1:3);
        l_real = oru_feet(:, 4:6);
        r_expected = oru_feet(:, 7:9);
        r_real = oru_feet(:, 10:12);
        figure ('Position', get(0,'Screensize')*0.9);
        hold on;
        title ('Feet positions');
        plot3 (l_expected(:,1), l_expected(:,2), l_expected(:,3), 'b');
        plot3 (l_real(:,1), l_real(:,2), l_real(:,3), 'r');
        plot3 (r_expected(:,1), r_expected(:,2), r_expected(:,3), 'b');
        plot3 (r_real(:,1), r_real(:,2), r_real(:,3), 'r');
        hold off;
    end
catch
end

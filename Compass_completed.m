clear all
close all
clc
%% 

%Preliminary test program for the acquisition of acc values

disp('Open the Matlab app on your smartphone')
disp('Connect to device...')
dev_name = 'iPhone - iPhone di giulio';
m = mobiledev(dev_name);
disp('Device connected.')

disp('Start inizialization...');
m.MagneticSensorEnabled = 1;
m.OrientationSensorEnabled = 1;
m.SampleRate = 10; % [S/s]
Ts = 1/m.SampleRate; %[s] sampling period
disp('Initialization completed.');

m.Logging = 1;
pause(1);

%% 

%calibrate the compass
disp('Rotate the compass in a horizontal plane...');
for i = 1:5
    fprintf('Calibration starts in:%d \n', 6-i); pause(1)
end

old_buffersize = 0; % for connection control
num = 100;
hx_max = 0; hx_min = 0;
hy_max = 0; hy_min = 0;
for j = 1:num
    [mag,t] = magfieldlog(m);

    % Connection control
    pause(0.3) %needed for load the data,otherwise it will disp 'Connection lost'
    if length(t) <= old_buffersize
        disp('Connection lost, please turn on your device...')
        continue
    end
    old_buffersize = length(t);

    x = mag(end,1);
    y = mag(end,2);
    
    hx_max = max([hx_max , x]);
    hx_min = min([hx_min , x]);

    hy_max = max([hy_max , y]);
    hy_min = min([hy_min , y]);
end

disp('Calibration Completed...')
%determine now the slopes and offset
Xsf = max([1 , (hy_max - hy_min) / (hx_max - hx_min)])
Ysf = max([1 , (hx_max - hx_min) / (hy_max - hy_min)])

Xoff = ((hx_max - hx_min)/2 - hx_max) * Xsf
Yoff = ((hy_max - hy_min)/2 - hy_max) * Ysf

save Deviation Xsf Ysf Xoff Yoff %for work online
pause(1)
 %%
load Deviation %for work online

%set the declination angle according to Padova GPS coordinates
dec_angle = deg2rad(3); %radians


disp('Start electronic compass')
n = 50;
x=0;
y=0;

%initialize plot
xmax = 10; ymax = 10;
L = strsplit(sprintf('%c\n','N','W','S','E'));  % Letter Labels
figure(1);
hold on
line([0 10],[0 0],'Color','r','LineWidth', 2);
line([0 0],[0 10],'Color','g','LineWidth', 2);
rectangle('Position', [-1, -2, 2, 4], 'EdgeColor', 'k', 'LineWidth', 4);
h = plot(0,0,'k');
N = plot(x,y,'+r'); w = plot(0,0,'+r'); s = plot(0,0,'+r'); e = plot(0,0,'+r');
Ne = plot(x,y,'+r'); sw = plot(0,0,'+r'); se = plot(0,0,'+r'); Nw = plot(0,0,'+r');
%l = text(x, y, L(1), 'HorizontalAlignment','center', 'VerticalAlignment','middle');
grid on;
xlim([-xmax, xmax])
ylim([-ymax, ymax])

old2_buffersize = 0; %for connection check

len = 5;

ang = 0;

%Start timer
Tgame = 0;
Tin = tic();
z = 0;

for i = 1:n
    [mag,t1] = magfieldlog(m);
    [o , t] = orientlog(m);

    % Connection control
    %pause(0.3) %needed for load the data,otherwise it will disp 'Connection lost'
    if length(t1) <= old2_buffersize
        disp('Connection lost, please turn on your device...')
        pause(0.5)
        continue
    end
    old2_buffersize = length(t1);
    

    if(abs(o(end,2)) > 30)
        disp('Put the phone in a horizontal plane')
    else
        %convert the angle into radians
        o = deg2rad(o); 

        %backrotate mag into H in earth reference frame
        H = [mag(end,1)*cos(o(end,2)) + mag(end,2)*sin(o(end,3))*sin(o(end,2)) - mag(end,3)*cos(o(end,3))*sin(o(end,2)) , mag(end,2)*cos(o(end,3)) + mag(end,3)*sin(o(end,3))];

        %compensate the offset and sensitivity
        H = [Xsf*H(1)+Xoff , Ysf*H(2)+Yoff];

        %determine angle from vector IN RADIAN
        azimuth = atan2(H(2), H(1)) + dec_angle;
        deg_az = rad2deg(azimuth) ;


        if(deg_az < 30 && deg_az > -30)
            ang = 0 %NORTH
        elseif(deg_az < 60 && deg_az >= 30)
            ang = pi/4 %NORTH WEST
        elseif(deg_az < 120 && deg_az >= 60)
            ang = pi/2 %WEST
        elseif(deg_az < 150 && deg_az >= 120)
            ang = 3*pi/4 %SOUTH WEST
        elseif(deg_az < 180 && deg_az >= 150 || deg_az > -180 && deg_az < -150)
            ang = pi     %SOUTH
        elseif(deg_az < -120 && deg_az >= -150)
            ang = 5*pi/4 %SOUTH EAST
        elseif(deg_az < -60 && deg_az >= -120)
            ang = 3*pi/2 %EAST
        elseif(deg_az < -30 && deg_az >= -60)
            ang = 7*pi/4 %NORTH EAST
        end


        %rotation matrix of 45 degrees
        rot_45 = [cos(pi/4) , -sin(pi/4);
                  sin(pi/4) , cos(pi/4)];

        %compute the cardinal vectors by rotating the first one 
        N_vec = [linspace(0,len*cos(ang),50) ;
                       linspace(0,len*sin(ang),50)];
        
        NW_vec = rot_45*N_vec;
        W_vec = rot_45*NW_vec;
        SW_vec = rot_45*W_vec;
        S_vec = rot_45*SW_vec;
        SE_vec = rot_45*S_vec;
        E_vec = rot_45*SE_vec;
        NE_vec = rot_45*E_vec;

        %draw the segments
        set(h, 'XData', ...
            [N_vec(1,:) , S_vec(1,:) , E_vec(1,:) , W_vec(1,:), ...
            NW_vec(1,:) , SW_vec(1,:) , SE_vec(1,:) , NE_vec(1,:)], ...
            'YData', [N_vec(2,:) , S_vec(2,:) , E_vec(2,:) , W_vec(2,:), ...
            NW_vec(2,:) , SW_vec(2,:) , SE_vec(2,:) , NE_vec(2,:)]);

        %delete the previous objects
        delete(N); delete(w); delete(s); delete(e);
        delete(Nw); delete(sw); delete(se); delete(Ne);

        %write the cardinal points
        N = text(N_vec(1,end), N_vec(2,end) , L(1) , 'HorizontalAlignment','center', 'VerticalAlignment','middle');
        w = text(W_vec(1,end), W_vec(2,end) , L(2) , 'HorizontalAlignment','center', 'VerticalAlignment','middle');
        s = text(S_vec(1,end), S_vec(2,end) , L(3) , 'HorizontalAlignment','center', 'VerticalAlignment','middle');
        e = text(E_vec(1,end), E_vec(2,end) , L(4) , 'HorizontalAlignment','center', 'VerticalAlignment','middle');

        Nw = text(NW_vec(1,end), NW_vec(2,end) , 'NW' , 'HorizontalAlignment','center', 'VerticalAlignment','middle');
        sw = text(SW_vec(1,end), SW_vec(2,end) , 'SW' , 'HorizontalAlignment','center', 'VerticalAlignment','middle');
        se = text(SE_vec(1,end), SE_vec(2,end) , 'SE' , 'HorizontalAlignment','center', 'VerticalAlignment','middle');
        Ne = text(NE_vec(1,end), NE_vec(2,end) , 'NE' , 'HorizontalAlignment','center', 'VerticalAlignment','middle');

        drawnow

        Tgame = toc(Tin)
        z=z+1;
    end
end

Tcycle = Tgame/z

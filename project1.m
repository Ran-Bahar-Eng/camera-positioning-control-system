clear; clc; close all;
%% part A
% parameters
M = 0.1;      
m = 0.01;   
R = 6.5;     
Kt = 0.0301;   
Kb = 0.0301;   
c = 8.35*10^-5;    
a =0.02;   

%  transfer function
A = Kt / (R *a* (M + m));
B = (R*c + Kt * Kb) / (R * a^2 * (M + m));
s = tf('s');
G_ux = A / (s * (s + B));


% Pole-Zero map of G_ux
figure;
pzmap(G_ux);
grid on;
title('Pole-Zero map of G_ux');

% Bode with margin of G_ux
figure;
margin(G_ux); 
grid on;
title('Bode of G_ux');

% Nichols of G_ux
figure;
nichols(G_ux);
ngrid; 
title('Nichols Chart of G_ux');


%% part B
F = 40 / (s + 40);
xr_mag = 0.08; 

% This code segment evaluates $k$ values below 6.4 for
%  the controller to determine which values satisfy the requirements.
%  The results are printed, and appropriate plots are generated for 
% the highest valid k value

k_vec = 0.1:0.1:6.4; 
valid_k = [];
overshoots = [];
settling_times = [];
max_voltages = [];


for k = k_vec
    %H=x/xr
    H = F * feedback(k * G_ux, 1);
    %G_xrU=U/xr
    G_xrU = (F * k) / (1 + k * G_ux);
    
    % step simulation for x
    [x_out, t_x] = step(H * xr_mag);
    info_x = stepinfo(x_out, t_x, xr_mag);
    
    % step simulation for U
    [u_out, ~] = step(G_xrU * xr_mag);
    max_U = max(abs(u_out));

    % Check conditions
    % 1. Overshoot < 5%
    % 2. Settling Time (2%) <= 1.2s
    % 3. |U| <= 4.8V
    if info_x.Overshoot < 5 && info_x.SettlingTime <= 1.2 && max_U <= 4.8
        valid_k = [valid_k, k];
        overshoots = [overshoots, info_x.Overshoot];
        settling_times = [settling_times, info_x.SettlingTime];
        max_voltages = [max_voltages, max_U];
    end
end

fprintf('\nPart B Results \n\n');

if isempty(valid_k)
    warning('not found');
else
    fprintf('there are %d valid k values .\n', length(valid_k));

    %create table witn min max and middle k
    indices = [1, round(length(valid_k)/2), length(valid_k)];
    T = table(valid_k(indices)', overshoots(indices)', settling_times(indices)', max_voltages(indices)', ...
        'VariableNames', {'k_Value', 'Overshoot_Pct', 'SettlingTime_s', 'Max_Voltage_V'});
    disp(T);
    
    %create graphs for maximal K
    best_k = max(valid_k);
    H_final = minreal(F * feedback(best_k * G_ux, 1));
    G_xrU_final = minreal((F * best_k) / (1 + best_k * G_ux));
    
    figure;
    subplot(2,1,1);
    step(H_final * xr_mag); grid on;
    title(['Position Response x(t), k = ', num2str(best_k)]);
    
    subplot(2,1,2);
    step(G_xrU_final * xr_mag); grid on;
    title('Control Voltage U(t)');
    ylabel('Voltage [V]');
    line([0 2], [4.8 4.8], 'Color', 'r', 'LineStyle', '--'); 
    line([0 2], [-4.8 -4.8], 'Color', 'r', 'LineStyle', '--');
end

% Bode with margin of system with C=5
figure;
margin(5 * G_ux); 
title("Bode of 5*G_ux")
grid on;


%% Part C
C=52*(2.2*s+9.31)/(s+2.2*9.31)
newsys=F*feedback(C*G_ux,1);
info = stepinfo(newsys*xr_mag);

% step response of x
figure;
step(newsys);
grid on;
title('Position Response x(t) with C(s)');

fprintf('\nPart C Results \n\n');
fprintf('Overshoot: %.2f%%\n', info.Overshoot);
fprintf('Settling Time: %.2f seconds\n', info.SettlingTime);

% step response of U
G_xrU = (F * C) / (1 + C * G_ux);
info = step(G_xrU * xr_mag);
max_U = max(abs(info));
fprintf('Max Voltage[V]:    %.4f\n', max_U);

figure;
step(G_xrU*xr_mag);
grid on;
title("Control Voltage U(t) with C(s)")

% bode with margin of Cx*G_ux
figure;
margin(C * G_ux); 
title("Bode of C(s)*G_ux")
grid on;

%% Part D
fprintf('\nPart D\n');
G_ux_delayed=G_ux*exp(-0.05*s);

% margin for part b with delay
figure;
margin(5*G_ux_delayed); 
title("Bode of 5*G_ux with delay")
grid on;

%Closed-loop performance for part b with delay
system=F*feedback(5*G_ux_delayed,1);
info=stepinfo(system*xr_mag);
ss_value=dcgain(system*xr_mag);

% step part b with delay
figure;
step(system * xr_mag);
grid on;
title('Step Response of C=5 and G_ux delayed');

% Show results 
fprintf('\nResults (C=5, delayed G_ux)\n');
fprintf('Overshoot:       %.2f%%\n', info.Overshoot);
fprintf('Settling Time:   %.4f seconds\n', info.SettlingTime);
fprintf('Steady State:    %.4f\n', ss_value);

% Checking max Voltage
k=5;
G_xrU_delayed = (F * k) / (1 + k * G_ux_delayed);
info = step(G_xrU_delayed * xr_mag);
max_U = max(abs(info));
fprintf('Max Voltage[V] of C=5 and G_ux delayed:    %.4f\n', max_U);


% margin for part c with delay
figure;
margin(C*G_ux_delayed); 
title("Bode of C(s)*G_ux with delay")
grid on;

%Closed-loop performance for part c with delay
system=F*feedback(C*G_ux_delayed,1);
info=stepinfo(system*xr_mag);
ss_value=dcgain(system*xr_mag);

% step part c with delay
figure;
step(system * xr_mag);
grid on;
title('Step Response of C=C(s) and G_ux delayed');

% Show results
fprintf('\nResults (C=c(s), delayed G_ux)\n');
fprintf('Overshoot:       %.2f%%\n', info.Overshoot);
fprintf('Settling Time:   %.4f seconds\n', info.SettlingTime);
fprintf('Steady State:    %.4f\n', ss_value);

% Checking max Voltage 
G_xrU_delayed = (F * C) / (1 + C * G_ux_delayed);
info = step(G_xrU_delayed * xr_mag);
max_U = max(abs(info));
fprintf('Max Voltage[V] of C=C(s) and G_ux delayed:    %.4f\n', max_U);

%% Part E
fprintf('\nPart E Results \n');

piC=G_ux*(1-exp(-0.05*s));
C_x=feedback(C,piC);
system=F*feedback(C_x*G_ux_delayed,1);
info=stepinfo(system*xr_mag);
ss_value = dcgain(minreal(system * xr_mag));

% step response
figure;
step(system * xr_mag);
grid on;
title('Step Response with C_x(s)');

% Show results
fprintf('\nResults (C=DTC, delayed G_ux)\n');
fprintf('Overshoot:       %.2f%%\n', info.Overshoot);
fprintf('Settling Time:   %.4f seconds\n', info.SettlingTime);
fprintf('Steady State:    %.4f\n', ss_value);

% Checking max Voltage
G_xrU_delayed = (F * C_x) / (1 + C_x * G_ux_delayed);
info = step(G_xrU_delayed * xr_mag);
max_U = max(abs(info));
fprintf('Max Voltage[V] with C_x(s):    %.4f\n', max_U);

% margin
figure;
margin(C_x*G_ux_delayed); 
title("Bode of C_x(s)*G_ux with delay")
grid on;

%% Part F
fprintf('\nPart F Results \n');
%C_f=(16.15*s^2+9*s+47.5)/(s^2+16);
C_f=(3.8*s^2-23.31*s+7.61)/(s^2+16);
fprintf('G_ux: \n');
disp(G_ux)

systemF = feedback(C_f*G_ux, 1);
t = 0:0.01:20;
y = lsim(systemF, 0.04*sin(4*t), t);
figure;
plot(t, 0.04*sin(4*t), 'r--', 'LineWidth', 1.5); 
hold on;
plot(t, y, 'b', 'LineWidth', 1.5); 
hold off;
grid on;
title('x response with xr=0.04*sin(4*t) and the controller');
xlabel('Time (sec)');
ylabel('Amplitude');
legend('xr(t)', 'x(t)', 'Location', 'best');

T_uxr=feedback(C_f,G_ux);
figure;
bode(T_uxr);
grid on;
title('bode of T uxr');



y = lsim(T_uxr, 0.04*sin(4*t), t);
figure;
plot(t, 0.04*sin(4*t), 'r--', 'LineWidth', 1.5); 
hold on;
plot(t, y, 'b', 'LineWidth', 1.5); 
hold off;
grid on;
title('U response with xr=0.04*sin(4*t) and the controller');
xlabel('Time (sec)');
ylabel('Amplitude');
legend('xr(t)', 'U(t)', 'Location', 'best');






%% Part H
fprintf('\nPart H Results \n');
alpha=(R*a)/Kt;
beta=(R*c+Kt*Kb)/(Kt*a);
l=0.2;
g=9.81;

G_ux_teta0=tf([l,0,g],[alpha*M*l,beta*l,alpha*(M+m)*g,beta*g,0]);
G_ux_tetapi=tf([l,0,-g],[alpha*M*l,beta*l,-alpha*(M+m)*g,-beta*g,0]);
G_xteta_teta0=tf([1,0,0],[l,0,g]);
G_xteta_tetapi=tf([1,0,0],[l,0,-g]);

systems = {G_ux_teta0, G_ux_tetapi, G_xteta_teta0, G_xteta_tetapi};
titles = {'G ux teta0',' G ux tetapi', 'G xteta_teta0', 'G xteta tetapi'};

% figure 4 poles zero maps of the transfer functions
figure;
for i = 1:4
    subplot(2, 2, i);

    pzmap(systems{i});
    grid on;

    p = pole(systems{i});

    fprintf('poles of %s:\n', titles{i});
    disp(p);

    pole_str = num2str(p, '%.2f  ');
    title(titles{i});
end

sgtitle('poles and zeroes maps');


%% Part I

C_x=2;
G_zteta=minreal(feedback(C_x*G_ux_teta0,1)*G_xteta_teta0);

% Calculate and display poles and zeros
fprintf('\npoles and zeroes maps of G_zteta\n');
disp('--- Zeros ---');
disp(zero(G_zteta));

disp('--- Poles ---');
disp(pole(G_zteta));


figure; 
pzmap(G_zteta); 
grid on;
title("poles and zeroes maps of G_zteta");


figure; 
impulse(G_zteta);
grid on;
title('Impulse Response of G zteta')

figure; 
margin(G_zteta);
grid on;
title('Bode of G zteta');


% Proportional and Lead Controller
k=4.67;
w_m=11.2;
deg=44;
a_sq=sqrt((sind(deg)+1)/(1-sind(deg)));
C_teta=k*(a_sq*s+w_m)/(s+a_sq*w_m);


figure; 
impulse(minreal(feedback(G_zteta,C_teta)));
grid on;
title('Impulse Response of teta, k=4.67');

G_du=C_x / (1 + C_x * G_ux_teta0 + C_teta * C_x * G_ux_teta0* G_xteta_teta0);
figure; 
impulse(G_du);
grid on;
title('Impulse Response of U, k=4.67');


% changing the proportional value of the controller
k=4.4;
C_teta=k*(a_sq*s+w_m)/(s+a_sq*w_m);


figure; 
impulse(minreal(feedback(G_zteta,C_teta)));
grid on;
title('Impulse Response of teta, k=4.4');


G_du=C_x / (1 + C_x * G_ux_teta0 + C_teta * C_x * G_ux_teta0* G_xteta_teta0);
figure; 
impulse(G_du);
grid on;
title('Impulse Response of U, k=4.4');

%% Part J

G_uteta=minreal(G_ux_tetapi*G_xteta_tetapi);

figure;
pzmap(G_uteta);
grid on;
title('Poles Zeroes map of G uteta');


figure;
nichols(G_uteta);
grid on;
title('nichols of G uteta');


figure;
nyquist(G_uteta);
grid on;
title('nyquist of G uteta');


%Lag Controller
C=(10*s+30)/(10*s);


figure;
nichols(C*G_uteta);
grid on;
title('nichols of C*G uteta(k=1)');

figure;
nyquist(C*G_uteta);
grid on;
title('nyquist of C*G uteta(k=1)')


% Proportional and Lag Controller
C=20*(10*s+30)/(10*s);

figure;
nichols(C*G_uteta);
grid on;
title('nichols of C*G uteta(k=20)');

figure;
nyquist(C*G_uteta);
grid on;
title('nyquist of C*G uteta(k=20)')

T=minreal(G_uteta/(1+C*G_uteta));

figure;
pzmap(T);
grid on;
title('pz map of T(s)=teta/d');

figure;
impulse(T);
grid on;
title('teta response to impulse in d');
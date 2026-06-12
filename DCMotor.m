clc
clear all
close all

%% System Variables
g = 9.81;   %[m/s^2] acc of gravity

m = 80; %[kg] mass
J = 20; %[kg*m^2] inertia of bar
L = 1;  %[m] length of bar

k = 500;    %[N/m] stiffness of spring
r = 25;     %[N*s/m] damping coeff

%% Motor Variables
La = 0.001; %[H] armature induction
Ra = 0.1; %[ohm] armature resistance
k_phi = 0.0001; %[Nm/A]

%% Uncontrolled System

ms = J + m*L^2/4;   %[kg*m^2] equivalent inertia
rs = r*L^2;     %[N*m*s] equivalent damping
ks = k*L^2 - m*g*L/2;   %[N*m] equivalent stiffness

w0 = sqrt(ks/ms);   %[rad/s] natural frequency
h = rs/(2*ms*w0);   % damping ratio
alph = -rs/(2*ms);     %[rad/s] alpha

if h<1
    w = w0*sqrt(1-h^2); %[rad] damped frequency
else
    w=0;
end

%% Control Parameters
kp=1000;
Td=0.5;
Ti=1;


kd = kp*Td; %[V] derivative gain
ki = kp/Ti; %[V] integral gain


%% Transfer Functions
G_IO = tf(k_phi,[ms rs ks]);    % TF from Ia to angle (O)
G_EI = tf(1, [La Ra]);  % TF from Va-k_phi*s*O to Ia

B = tf([k_phi 0],1);    % TF from O to K_ph*s*O
A = series(G_EI,G_IO);  % TF from Va-k_phi*s*O to angle

G = feedback(A,B);  % TF from Va to angle (O)
[G_num, G_den] = tfdata(G,'v');

% TF from Va to angle as calculated manually
s = tf('s');
% Gs = tf(k_phi)/((ms*s^2+rs*s+ks)*(La*s+Ra)+k_phi^2*s);

% Actuator TF, from Error (E) to Va
R = tf([kp*Td*Ti kp*Ti kp],[Ti 0]);
%Rs = kp*(1+Td*s+1/(Ti*s)); % As calculated by hand

GH = series(R, G);  % Controlled System Open-loop TF
Ls = feedback(GH,1); % Closed-loop TF

%% POLES
poles_passive = pole(G);

poles = pole(GH)
zeros = zero(GH)
polesL = pole(Ls)

stepinfo(Ls)

%% Diagrams
figure('Name','Bode of GH');         % Bode Diagram of GH
asymp(GH)
% 
% figure('Name','Margins');         % Bode Diagram with margins
% margin(GH)
% 
% figure('Name','Nyquist');      % Nyquist Diagram
% nyquist(GH)
% 
% figure('Name','Root Locus');   % Root Locus of GH
% rlocus(GH)

%% SIMULINK
theta_ref = 1;  %[rad] reference angle
sT = 1000; %[s] simulation time

sim_out = sim("simulationDCMotor.slx");    % access simulink data
time = sim_out.time.Time;           %[s] simulation time vector
position = sim_out.position.data;   %[rad] simulation position
close all
clear all

%Dimensions:
%Money: M; Power: P; Time: T; Volume: V

% Inputs:
%   C:          Electricity price per period. Vector 1xN.  [M/P·T]
%   Delta_C:    Time for each period. Vector 1xN        [T]
%   x0:         Initial state value. Vector nx1.        [V]
%   A:          Dynamic matrix. Matrix nxn. [Adimensional]
%   B:          Control Matrix. Matrix nxM.             [V/T]
%   W:          Integral of disturbance per time interval. Matrix DxN. [V]
%   Pmss:       Steady state power consmption of each actuator. Vector 1xM [P]
%   xmax:       Maximum state constraint. Matrix nx1.   [V]
%   xmin:       Minimum state constraint. Matrix nx1.   [V]
% Outputs:
%   U:          Integral control action. Matrix MxN.    [T]
%   Energy:     Amount of energy consumed. Real.        [P]
%   Cost:       Energy cost. Real.                      [M]

% PROBLEM PARAMETERS

C  = [11.87 14.11 82.05 14.11 82.05 11.87]*(1/60);  % Electricity rates c€/min
N  = length(C);
Delta_C = 60*[6 1 3 8 4 2];                         % length in minutes of the constant price intervals

% SYSTEM DYNAMICS

A  = zeros(3,3);                                                                  
B1 = 1;                                                            % Flow of Pump 1 [Kl/min]
B2 = 0.6;                                                          % Flow of Pump 2 [Kl/min]
B3 = 0.6;                                                          % Flow of Pump 3 [Kl/min]
B4 = 0.6;                                                          % Flow of Pump 4 [Kl/min]
B  = [B1 -B2 0 -B4 ; 0 B2 -B3 0 ; 0 0 B3 B4];                      % Control Actions Matrix [Kl/min]
Qe2 = 1/12;
Qs1 = 1/12 ;
Qs2 = 1/8;
Qs3 = 1/4;
Bw = [-Qs1 ; Qe2-Qs2 ; -Qs3];                                       % Disturbance matrix [Kl/min]
Disturbance = [Bw(1)*ones(1,N); Bw(2)*ones(1,N); Bw(3)*ones(1,N)];  % DxN [kl/min]
W = Disturbance.*[Delta_C; Delta_C; Delta_C];                       % Integral  [Kl]
  

% INITIAL CONDITIONS AND LIMITS

xmax = [400; 250; 250];                       % Maximum Volume [Kl]
xmin = [20; 20; 20];                          % Minimum Volume [Kl]
P1   = 5;                                     % Power of Pump 1 [Kw]
P2   = 5;                                     % Power of Pump 2 [Kw]
P3   = 5;                                     % Power of Pump 3 [Kw]
P4   = 5;                                     % Power of Pump 4 [Kw]
Pmss =[P1 P2 P3 P4];                          % 1xM [Kw]

for i=0:9
    
x0=[xmin(1)+(xmax(1)-xmin(1))*i/9; 50 ; 50];           	      % Initial State. V1 V2 V3 [Kl]

%First Stage Call (Linear Programming problem)
[U, Energy, EnergyCost, X_m, U_m, Xf_m] = FirstStageLP( C, Delta_C, x0, A, B, W, Pmss, xmax, xmin)
VisualizeData(xmin,xmax, Xf_m,U_m, Delta_C,C,Energy)
pause
close all
end

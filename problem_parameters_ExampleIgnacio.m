
% PROBLEM PARAMETERS


% Main Dimensions:
% Money: M  ,c€*
% Power: P  ,KW*
% Time: T   ,min*
% Volume: V ,Kl*
% Note the importance on put the same units for each dimension. The program will return
% back results with the set units..
% *We recommend to note the units that will be used, just next to the
% general dimension

C  = [11.87 14.11 82.05 14.11 82.05 11.87]*(1/60);  % Electricity rates [M/(P·T)]
N  = length(C);
Delta_C = 60*[6 1 3 8 4 2];                  % length of the constant price intervals [T]
nAuxiliarTanks = 3;
nMainTanks = 1;
D=nAuxiliarTanks+nMainTanks; %Number of total tanks or deposits

% System Dynamics

A  = zeros(D,D);                                                    % La dinamica es la de un integrador               
B1 = 0.5;                                                           % Flow of Pump 1 [Kl/min] [V/T]
B2 = 0.6;                                                           % Flow of Pump 2 [Kl/min] [V/T]
B3 = 0.3;
B  = [-B1 -B2 0; B1 0 0; 0 B2 B3;0 0 -B3];                          % Control Actions Matrix [Kl/min] [V/T]
Bw = [ 1/4; -1/12; -1/10; 1/15];                                    % Disturbance matrix [Qe Qs1 Qs2] [Kl/min] [V/T]
Disturbance=[];
for i=1:length(Bw)
Disturbance = [Disturbance;Bw(i)*ones(1,N)];                        %DxN [kl/min] [V/T]
end
W = Disturbance.*(kron(ones(D,1),Delta_C));                         % Integral  [Kl][V]


% Initial Conditions and limits

x0=[100; 50; 50; 50];                   % Initial State. V1 V2 V3 [Kl] [V]
xmax = [200; 100; 100; 100];              % Maximum Volume [Kl][V]
xmin = [20; 20; 20; 15];                  % Minimum Volume [Kl][V]  
P1   = 5;                                 % Power of Pump 1 [Kw][P]
P2   = 6;                                 % Power of Pump 2 [Kw][P]
p3   = 3;                                 % Power of Pump 3 [Kw][P]
Pmss =[P1 P2 p3];                         %1xM [Kw][P]

%First Stage Call (Linear Programming problem)
[ U, Energy, EnergyCost ] = FirstStageLP( C, Delta_C, x0, A, B, W, Pmss, xmax, xmin)



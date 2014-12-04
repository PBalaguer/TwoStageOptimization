
% PROBLEM PARAMETERS

clear all
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

% Flow of pump i [V/T]
B1 = 0.5;                                                           
B2 = 0.6;                                                         
B3 = 0.4;

B  = [-B1 -B2 0 ; B1 0 0 ; 0 B2 -B3;0 0 B3];                          % Control Actions Matrix [Kl/min] [V/T]
Bw = [1/6; -1/8; -1/8; -1/6];                              % Disturbance matrix [Qe Qs1 Qs2] [Kl/min] [V/T]
Disturbance=[];
for i=1:length(Bw)
Disturbance = [Disturbance;Bw(i)*ones(1,N)];                        %DxN [kl/min] [V/T]
end
W = Disturbance.*(kron(ones(D,1),Delta_C));                         % Integral  [Kl][V] Matrix DxN


% Initial Conditions and limits

x0=[20; 200; 200; 30];                   % Initial State. V1 V2 V3 [Kl] [V]
xmax = [400; 250; 250; 200];              % Maximum Volume [Kl][V]
xmin =  [20; 20; 20; 20];                  % Minimum Volume [Kl][V]  
% Power of Pump i [P]

P1   = 5;                                 
P2   = 6;                                
p3   = 3;
Pmss =[P1 P2 p3];                         %1xM [Kw][P]


%Solving and plotting results

 [ U, Energy, EnergyCost, X_m, U_m, Xf_m ] = FirstStageLP( C, Delta_C, x0, A, B, W, Pmss, xmax, xmin)
 VisualizeData(xmin,xmax, Xf_m,U_m, Delta_C,C)



% Function that solves the first stage that is the LP of the two steps
% procedure.
%
%
%
% function [U,Energy,Cost]=First_Stage_LP(C, x0, A, B, W, Pmss, xmax, xmin)
%
% Inputs:
%   C:          Electricity price per period. Vector 1xN. [€/kWh]
%   x0:         Initial state value. Vector nx1.[Kl]
%   A:          Dynamic matrix. Matrix nxn.
%   B:          Control Matrix. Matrix nxM.
%   W:          Integral of disturbance per time interval. Matrix DxN.
%   Pmss:       Steady state power consmption of each actuator. Vector 1xM
%   xmax:       Maximum state constraint. Matrix nx1. [Kl]
%   xmin:       Minimum state constraint. Matrix nx1. [Kl]

% Outputs:
%   U:          Integral control action. Matrix MxN. [min]
%   Energy:     Amount of energy consumed. Real.      [kW]
%   Cost:       Energy cost. Real.                    [€]






C=[11.78 14.11 82.05 14.11 82.05 11.87];
x0=[2 1 1]';
A=zeros(3);
F02=1/12;
F03=F02;
F1=1/3;
Fe=1/6;
F=[Fe -F02 -F03];
P1=5;
P2=6;
Pmss=[P1 P2];
B=[-F1 -F02; F1 0;0 F02];
var_t=[6 1 3 8 4 2];
var_t_min=60*var_t;
W=kron(F',var_t_min);

function [ U, Energy, EnergyCost ] = FirstStageLP( C, x0, A, B, W, Pmss, xmax, xmin)


%Optimization call using cvx solver
cvx_begin

variable x(30)


% Alp matrix in Alp*x < blp
Abarra = [ A B ];
At     = kron( tril( ones( N, N), 0 ), Abarra );    % lower diagonal block matrix with A matrix in each one of its entries
Alp    = [ -At; At ];       


Iden = ones( N, 1 );

% W matrix 
Wbar = [];
for i = 1:N
    Wbar = [ Wbar; sum( W( :, 1:i ), 2 ) ];             
end

% blp matrix in Alp * x < blp
blp = [ kron( -Iden, (xmin-x0) ); kron( Iden,(xmax-x0)) ] + [ Wbar; -Wbar ];     % blp Matrix, Alp x <= blp

pos_U1i=zeros(1,length(C));
pU1i=4;
for i=1:6
pos_U1i(i)=pU1i;
pU1i=pU1i+5;
end

pos_U2i=zeros(1,length(C));
pU1i=5;
for i=1:6
pos_U2i(i)=pU2i;
pU2i=pU1i+5;
end


minimize [x(pos_U1i)*Pmss(1,1) x(pos_U2i)*Pmss(1,2)]*transpose[C C]*(1/60) %dimension balance min*(€/kWh)*(kW)*(1h/60min)
subject to
   Alp * x <= blp 
   x >= 0
cvx_end

%Output management

U = double(U);
Energy       = Pmss * ( sum( U' ) )';
EnergyCost   = C * ( Pmss * U )';

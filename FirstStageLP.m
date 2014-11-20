
% Function that solves the first stage that is the LP of the two steps
% procedure.
%
%
%
% function [U,Energy,Cost]=First_Stage_LP(C, x0, A, B, W, Pmss, xmax, xmin)

%Dimensions:
%Money: M; Power: P; Time: T; Volume: V

% Inputs:
%   C:          Electricity price per period. Vector 1xN.  [M/P·T]
%   Delta_C:    Time for each period. Vector 1xN        [T]
%   x0:         Initial state value. Vector nx1.        [V]
%   A:          Dynamic matrix. Matrix nxn. [Adimensional]
%   B:          Control Matrix. Matrix nxM.             [V/T]
%   W:          Integral of disturbance per time interval. Matrix DxN. [V]
%   Pmss:       Steady state power consmption of each actuator. Vector 1xM
%   [P]
%   xmax:       Maximum state constraint. Matrix nx1.   [V]
%   xmin:       Minimum state constraint. Matrix nx1.   [V]
% Outputs:
%   U:          Integral control action. Matrix MxN.    [T]
%   Energy:     Amount of energy consumed. Real.        [P]
%   Cost:       Energy cost. Real.                      [M]



function [ U, Energy, EnergyCost ] = FirstStageLP( C, Delta_C, x0, A, B, W, Pmss, xmax, xmin)

N=length(C);
[D M]=size(B);

                        
A_barra=kron(tril( ones( N, N), 0 ), A );           
B_barra=kron(tril( ones( N, N), 0 ), B );               %[V/T]
 
Ct=kron(C,Pmss);                                        %[M/T]

Iden = ones( N, 1 );

% W matrix 
Wbar = [];
for i = 1:N
    Wbar = [ Wbar; sum( W( :, 1:i ), 2 ) ];          %Wbar D*Nx1 [V]       
end

% blp matrix in Alp * x < blp
blp_min = [ kron( -Iden, (xmin-x0) )+ Wbar];         %blp_min D*Nx1 [V]
blp_max = [kron( Iden,(xmax-x0))- Wbar ];            %blp_max D*Nx1 [V]

% blp Matrix, Alp x <= blp

cvx_begin

variables X(D*N,1) U(M*N,1)                          %X [V] U[T]
minimize Ct*U 
subject to
   -(A_barra*X+B_barra*U) <= blp_min                 %[V]
    A_barra*X+B_barra*U <= blp_max                   %[V]
    Xf=A_barra*X+B_barra*U+Wbar+kron(x0,ones(N,1))   %[V] Xf describes the state vector for each period.
   
    zeros(M*N,1)<=U<=transpose(kron(Delta_C,ones(1,M)))
  
cvx_end

%Output management

% Transformation of vector X(D*N,1) in matrix form X_m(N,D)
X_m=[];
for i= 1:length(C);
    X_m=[X_m;tranpose(X((i-1)*D+1:i*D))];
end

% Transformation of vector Xf(D*N,1) in matrix form Xf_m(N,D)
Xf_m=[];
for i= 1:length(C);
    Xf_m=[Xf_m;tranpose(Xf((i-1)*D+1:i*D))];
end

% Transformation of vector U(M*N,1) in matrix form U_m(N,M)
U_m=[];
for i= 1:length(C);
    U_m=[U_m;tranpose(U((i-1)*M+1:i*M))];
end

Energy       = kron(eye(N),Pmss)*U*(1/60)    %[P·T]
EnergyCost   = Ct*U                          %[M]




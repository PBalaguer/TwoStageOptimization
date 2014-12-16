% Function that solves the second stage that is the BIP of the two steps
% procedure.
%
%
%
% function [u, Umisum, L] = Second_Stage_BIP(Ui,Delta_Ci,L,x0,A,B,Bw,W,xmax,xmin)
%
% Inputs:
%   Ui:         Optimal Energy to be deployed in the BIP. Matrix Mx1 with
%               at least one element greater than zero.
%   Delta_Ci:   Time interval for the BIP. Integer.
%   L:          Indirect sampling time. Integer >=1. Ts=Ui/L.
%   x0:         Initial state value. Vector nx1.
%   A:          Dynamic matrix. Matrix nxn.
%   B:          Control Matrix. Matrix nxM.
%   Bw:         Disturbance Matrix. Matrix nxD.
%   Wi:         Integral of disturbance at fast sampling. Matrix Dx1.
%   xmax:       Maximum state constraint. Real.
%   xmin:       Minimum state constraint. Real.
%	ops:		Yalmip's solver options.
% Outputs:
%   u:          Binary control action. Matrix MxK.
%   Umisum:     Integral control action. Integer.
%   L:          Indirect sampling time. Integer.


function [U_m_K, L, Xf_m_K] = SecondStageBIP( Ui, Delta_Ci, L, x0, A, B, Wi, xmax, xmin)


% 0. - VARIABLE DEFINITION
N  = size(Wi, 2);
[n, M] = size(B);


% 1.- SAMPLING TIME DEFINITION
Ui_dummy = Ui;
if Ui_dummy == 0 
	Umisum = [ 0; 0 ];
	u = zeros(Delta_Ci,2);
    return 
end

Ui_dummy(Ui_dummy == 0) = [];
Ui_dummy=round(Ui_dummy);
Ui_greatest = Ui_dummy(1);
for k = 2:length(Ui_dummy)
    Ui_greatest  = gcd(Ui_greatest,Ui_dummy(k)); % The greatest common divisor. Required for equating rational values.
end

Ts = Ui_greatest/L;
if Ts < 1
	L  = 1;
 	Ts = Ui_greatest;
end
 
K  = Delta_Ci*L/Ui_greatest;       % Number of sampling intervals.
K  = floor(K);                     % It must be an integer. 


% 2.- DISCRETIZATION OF SYSTEM DYNAMICS
Scd = ss( A, B, eye(n), zeros(n, M));
Sdd = c2d(Scd,Ts,'zoh');
Ad  = Sdd.a;
Bd  = Sdd.b;

% 3.- CONSTRAINTS CONSTRUCTION
% Ax<=b
% Abarrad

Bbarrad=[];
aux=[zeros(n,M)];
for i = 1:K
    for j = K:-1:i
        aux=[aux; Ad^(K-j) * Bd];
    end
    Bbarrad=[Bbarrad aux];
    aux = [zeros( n*(i+1), M)];
end

Iden = ones(K+1,1);

Ikk=[eye(n,n)*x0];
for i=1:K
    Ikk=[Ikk; Ad^(i) * x0];
end

wk =  repmat( Wi/K, 1, K);

Wbar_K = [zeros(n,1)];
for i = 1:K
    Wbar_K = [ Wbar_K; sum( wk( :, 1:i ), 2 ) ];             
end


Alp = [ -Bbarrad; Bbarrad ];
blp = [-kron(xmin, Iden) + Ikk + Wbar_K; kron(xmax,Iden) - Ikk - Wbar_K ];

% CALL SOLVER
eye_m=[];
for i=1:K
    eye_m=[eye_m eye(M)];
end

cvx_begin
variable u(M*K,1) binary %Actually u goes from u(0) to u(K-1) and x from x(0) to x(K)

% minimize norm((eye_m*u-Ui),2)
subject to

Alp*u<=blp;

cvx_end

x=Bbarrad*u+Ikk+Wbar_K;

Xf_m_K=[];
for i= 1:(K+1)
    Xf_m_K=[Xf_m_K;transpose(x((i-1)*n+1:i*n))];
end

U_m_K=[];
for i= 1:4:M*K;
    U_m_K=[U_m_K , u(i:i+3)];
%     U_m_K=[U_m_K;transpose(u((i-1)*M+1:i*M))];
end









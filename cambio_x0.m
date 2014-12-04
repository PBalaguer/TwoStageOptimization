function cambio_x0
problem_parameters_ExampleIgnacio
n_variations=10;
X0_VAR=[];
for i=1:D
     x0_i=[xmin(i):((xmax(i)-xmin(i))/(n_variations-1)):xmax(i)];
     X0_VAR=[X0_VAR;x0_i];
end

XF_M=[];

for i=1:n_variations
    x0=X0_VAR(:,i);
   [ X_m, U_m, Xf_m ] = FirstStageLP( C, Delta_C, x0, A, B, W, Pmss, xmax, xmin);
    VisualizeData(xmin,xmax, Xf_m,U_m, Delta_C,C)
    XF_M=[XF_M;Xf_m];
end
    
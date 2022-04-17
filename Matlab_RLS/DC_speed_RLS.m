simul = zeros(1000,1);
ts = 0.01;
sysorder = 2;
lambda = 0.9995;
teta = zeros ( 2*sysorder , 1);
t = sysorder + 1 ;
Yk = zeros (1 , sysorder);
Uk = zeros (1, sysorder);
Pn =0.1* eye (2*sysorder);

file = fopen ('input_DCM.bin');
Input = fread (file, 'float');
[Ni Ci] = size (Input);
U_Data = Input (1+sysorder:Ni,1);

file = fopen ('Speed.bin');
Output = fread (file, 'float');
[No,Co]= size(Output);

Y_Data = Output (1+sysorder :No,1);
[N,C]= size(Y_Data);
  
U = U_Data ( 1:sysorder , :);
Y = Y_Data ( 1:sysorder , :);
i = 1 ;


  while t <= N % N*data test 
  for n = 1: sysorder
      Yk(n) = - Y(t- n);
      Uk(n) = U(t-n);
  end
Xk = [Yk Uk];
K = (Pn * Xk')./(lambda + Xk*(Pn*Xk'));
U = [U ; U_Data(t)];
Y = [Y ; Y_Data(t)];
teta = teta + K*( Y(t,:) - Xk * teta);
Pn = (Pn - K*(Xk*Pn))./lambda;

%num = teta(sysorder+1 : 2*sysorder)';
%denum = horzcat (1  , teta (1 : sysorder)');
%P_Chapeau = tf ( num , denum ,0.01);

simul (i) = teta'*Xk';

i = i+1;
t = t+ 1;
 end
 teta;



tin = 0:ts:9.99;

%Filtre PB%
num = [1];
denum = [0.1 1];
FPB= tf (num , denum);
FPB_z = c2d ( FPB , 0.01 , 'Tustin');
Filtered_Sim =lsim (FPB_z,simul,tin)
Filtered_Data = lsim (FPB_z,Output,tin);

A = [Filtered_Sim Filtered_Data];
plot (tin , A);
    




%%end
    
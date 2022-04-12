ts = 0.01;
sysorder = 1;
lambda = 0.995;
teta = zeros ( 2*sysorder , 1)
t = sysorder + 1 ;
X = zeros (t-sysorder , 2*sysorder);
Yk = zeros (1 , sysorder);
Uk = zeros (1, sysorder);
Pn = eye (2*sysorder);

rng(0,'twister');
a = 0;
b = 3.3;
Input = (b-a).*rand(1000,1) + a;
[Ni Ci] = size (Input)
U_Data = Input (1+sysorder:Ni,1);

tin = 1:ts:10.99;
P = tf ([3] , [1.9, 12])
Pd = c2d (P , 0.01 , methode = 'tustin' );

Output = lsim (Pd,Input,tin);
[No,Co]= size(Output);

Y_Data = Output (1+sysorder :No,1);
[N,C]= size(Y_Data);
  
U = U_Data ( 1:sysorder , :);
Y = Y_Data ( 1:sysorder , :);


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
Pn = Pn./lambda - K*(X*Pn)./lambda;
t = t+ 1;
 end
 teta;
num = teta(sysorder+1 : 2*sysorder)';
denum = horzcat (1  , teta (1 : sysorder)');
P_Chapeau = tf ( num , denum ,0.01);
sys = d2c ( P_Chapeau ,'Tustin') 
    




%%end
    
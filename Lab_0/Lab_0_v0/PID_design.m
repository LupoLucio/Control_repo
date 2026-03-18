Mp = 0.1;
t_s_5 = 0.15;

Req = mot.R+sens.curr.Rs;

Beq = mot.B+((mld.B)/((gbox.N)^2));

Jeq = mot.J + ((mld.J)/((gbox.N)^2));

kdrv = (1+(drv.R3)/(drv.R4))*((drv.R2)/((drv.R1)+(drv.R2)));

km = (kdrv*(mot.Kt))/(Req*Beq+(mot.Kt)*(mot.Ke));

Tm = (Req*Jeq)/(Req*Beq+((mot.Kt)*(mot.Ke)));

G = (tf([km], [Tm 1]))*(tf([1], [(gbox.N) 0]));

delta = (log(1/(0.1)))/(sqrt((pi)^2+(log(1/Mp))^2));

w_gc = 3/(delta*t_s_5);

[mag, phase] = bode(G, w_gc);

Delta_K = (mag(:))^(-1);

phi_m = atan((2*delta)/(sqrt(sqrt(1+4*((4*delta)^4))-2*((delta)^2))));

Delta_phi = -pi+phi_m-phase(:);

Kp = Delta_K*cos(Delta_phi);
clear all;
clc;

% Contains the set of all paramters given of the datasheet
% The last parts includes the call of the PID design function and the
% initialization of the input for simulink simulations

% General parameters and conversion gains
%   conversion gains
rpm2rads = 2*pi/60;                 %   [rpm]   -> [rad/s]
rads2rpm = 60/2/pi;                 %   [rad/s] -> [rpm]

rpm2degs = 360/60;                  %   [rpm]   -> [deg/s]
degs2rpm = 60/360;                  %   [deg/s] -> [rpm]

deg2rad = pi/180;                   %   [deg]   -> [rad]
rad2deg = 180/pi;                   %   [rad]   -> [deg]

ozin2Nm = 0.706e-2;                 %   [oz*inch] -> [N*m]

% DC motor nominal parameters
%   brushed DC-motor Faulhaber 2338S006S
mot.R    = 2.6;                     %   armature resistance
mot.L    = 180e-6;                  %   armature inductance
mot.Kt   = 1.088 * ozin2Nm;         %   torque constant
mot.Ke   = 0.804e-3 * rads2rpm;     %   back-EMF constant
mot.J    = 5.523e-5 * ozin2Nm;      %   rotor inertia
mot.B    = 0.0;                     %   viscous friction coeff (n.a.)      
mot.eta  = 0.69;                    %   motor efficiency
mot.PN   = 3.23/mot.eta;            %   nominal output power
mot.UN   = 6;                       %   nominal voltage
mot.IN   = mot.PN/mot.UN;           %   nominal current
mot.tauN = mot.Kt*mot.IN;           %   nominal torque
mot.taus = 2.42 * ozin2Nm;          %   stall torque
mot.w0   = 7200 * rpm2rads;         %   no-load speed

% Gearbox nominal parameters
%   planetary gearbox Micromotor SA 23/1 
gbox.N1   = 14;                     %   1st reduction ratio (planetary gearbox)
gbox.eta1 = 0.80;                   %   gearbox efficiency 

%   external transmission gears
gbox.N2   = 1;                      %   2nd reduction ratio (external trasmission gears)
gbox.J72  = 1.4e-6;                 %   inertia of a single external 72 tooth gear      
gbox.eta2 = 1;                      %   external trasmission efficiency (n.a.)

%   overall gearbox data
gbox.N   = gbox.N1*gbox.N2;         %   total reduction ratio
gbox.eta = gbox.eta1*gbox.eta2;     %   total efficiency
gbox.J   = 3*gbox.J72;              %   total inertia (at gearbox output)

% Mechanical load nominal parameters
%   inertia disc params
mld.JD = 3e-5;                      %   load disc inertia
mld.BD = 0.0;                       %   load viscous coeff (n.a.)               

%   overall mech load params
mld.J     = mld.JD + gbox.J;        %   total inertia
mld.B     = 2e-6;                   %   total viscous fric coeff (estimated) 
mld.tausf = 1.0e-2;                 %   total static friction (estimated) 



% Voltage driver nominal parameters
%   op-amp circuit params
drv.R1 = 7.5e3;                     %   op-amp input resistor (dac to non-inverting in)
drv.R2 = 1.6e3;                     %   op-amp input resistor (non-inverting in to gnd)
drv.R3 = 1.2e3;                     %   op-amp feedback resistor (output to  inverting in)
drv.R4 = 0.5e3;                     %   op-amp feedback resistor (inverting in to gnd)
drv.C1 = 100e-9;                    %   op-amp input capacitor
drv.outmax = 12;                    %   op-amp max output voltage

%   voltage driver dc-gain      
drv.dcgain = drv.R2/(drv.R1+drv.R2) * (1 + drv.R3/drv.R4);

%   voltage driver time constant
drv.Tc = drv.C1 * drv.R1*drv.R2/(drv.R1+drv.R2);



% Sensors data
%   shunt resistor
sens.curr.Rs = 0.5;   

%   Hewlett-Packard HEDS-5540#A06 optical encoder
sens.enc.ppr = 500*4;                               %   pulses per rotation
sens.enc.pulse2deg = 360/sens.enc.ppr;              %   [pulses] -> [deg]
sens.enc.pulse2rad = 2*pi/sens.enc.ppr;             %   [pulses] -> [rad]
sens.enc.deg2pulse = sens.enc.ppr/360;              %   [deg] -> [pulses]
sens.enc.rad2pulse = sens.enc.ppr/2/pi;             %   [rad] -> [pulses]

%   potentiometer 1 (Spectrol 138-0-0-103) - installed on motor box
sens.pot1.range.R      = 10e3;                                          %   ohmic value range 
sens.pot1.range.V      = 5;                                             %   voltage range
sens.pot1.range.th_deg = 345;                                           %   angle range [deg]
sens.pot1.range.th     = sens.pot1.range.th_deg * deg2rad;              %   angle range [rad]                 
sens.pot1.deg2V        = sens.pot1.range.V / sens.pot1.range.th_deg;    %   sensitivity [V/deg]
sens.pot1.rad2V        = sens.pot1.range.V / sens.pot1.range.th;        %   sensitivity [V/rad]
sens.pot1.V2deg        = 1/sens.pot1.deg2V;                             %   conversion gain [V] -> [deg]
sens.pot1.V2rad        = 1/sens.pot1.rad2V;                             %   conversion gain [V] -> [rad]

% Data acquisition board (daq) data
%   NI PCI-6221 DAC data
daq.dac.bits = 16;                                  %   resolution (bits)
daq.dac.fs   = 10;                                  %   full scale 
daq.dac.q    = 2*daq.dac.fs/(2^daq.dac.bits-1);     %	quantization

%   NI PCI-6221 ADC data
daq.adc.bits = 16;                                  %   resolution (bits)
daq.adc.fs   = 10;                                  %   full scale (as set in SLDRT Analog Input block)
daq.adc.q    = 2*daq.adc.fs/(2^daq.adc.bits-1);     %   quantization 


% Sampling time
Ts = 1e-3;

% Real derivative parameters

rdp.wci = 2*3.14*50;
rdp.di = 1/(sqrt(2));

% Simplified Motor transfer function

km = drv.dcgain / mot.Ke;  
Jl = mld.JD + 3*gbox.J72; 
Req = mot.R + sens.curr.Rs;
Jeq = mot.J + Jl/(gbox.N1^2);  
Tm = (Req * Jeq) / (mot.Kt * mot.Ke); 
s = tf('s');
P = km / (s * (Tm*s + 1) * gbox.N); % Ingresso [V], Uscita [rad] 
[numP, denP] = tfdata(P, 'v');

% PID required parameters

ts_5 = 0.0695;          
Mp = 0.08;            
delta = log(1/Mp) / (sqrt(pi^2 + (log(1/Mp))^2)); 
wgc = 3 / (delta * ts_5);

% Insert here the PID parameters or comment the parameters and uncomment and call the function
% design_pid_controller that apply the bode method to compute kp, kd, ki, tl

% call of the design_pid_controller function

wc_des = 3 / (delta * ts_5);

PM_des = (180/pi) * atan( (2*delta) / sqrt( sqrt(1+4*delta^4) - 2*delta^2 ) );

% Opzioni per imporre il margine di fase
opts = pidtuneOptions('PhaseMargin', PM_des);

% Sintesi PIDF con specifica diretta
C_vector = pidtune(P, 'PIDF', wc_des, opts);

% --- Costruzione del PIDF manuale ---
kp = C_vector.Kp;

kd = C_vector.Kd;

ki = C_vector.Ki;

tl = 1/(2*wgc);

s = tf('s');

C_s = kp + ki/s + kd*s/(1 + tl*s);



% --- Open loop TF ---
L = C_s * P;

[GM, PM, Wcg, Wcp] = margin(L);

fprintf('PIDF sintetizzato:\n');
fprintf('  PM ottenuto = %.2f°\n', PM);
fprintf('  wc ottenuta = %.2f rad/s\n', Wcp);

margin(L)
grid on

%Requirements verification

out = pid_metrics(P,kp,ki,kd,tl);

% Initialization of the input of LaB_0_real.slx (simulink simulations doesnt't
% start if a value is not assigned to these parameters

step_gain = 1;
simul.select2 = 1;
simul.select1 = 1;
simul.stair_gain = 1;


% parameter for confidence intervals. See Lab assignment: point 2.2) equation 12
var.c = 1.96;

% Saturator bounds

u_bar_max =12;

K_W = 15;

Beq = mot.B+(mld.B/((gbox.N)^2));

LSB_DAC = daq.dac.fs/(2^(daq.dac.bits)-1);

LSB_ADC = daq.adc.fs/(2^(daq.adc.bits)-1);

% Save parameters for all simulations. If you change somethings in param.m
% file rerun the file after all saves

save('Lab_2\Generated_files\parameters.mat')

% === CONFIGURAZIONE CARTELLE SIMULINK ===

% Cartella base dove vuoi mettere TUTTI i file temporanei
baseDir = 'Lab_2\Generated_files';

% Sottocartelle dedicate
cacheDir   = fullfile(baseDir, 'cache');    % per file .slxc
codegenDir = fullfile(baseDir, 'codegen');  % per slprj e code generation

% Crea le cartelle se non esistono
if ~exist(cacheDir, 'dir')
    mkdir(cacheDir);
end

if ~exist(codegenDir, 'dir')
    mkdir(codegenDir);
end

% === IMPOSTAZIONE PARAMETRI GLOBALI ===

Simulink.fileGenControl( ...
    'set', ...
    'CacheFolder', cacheDir, ...
    'CodeGenFolder', codegenDir);

%% PARAMETERS_AUTOMATIZED.M - Control Lab 0 System Parameters
%  
%  This file initializes all hardware, sensor, and controller parameters
%  for the SRV02-IL Rotary Servo plant (DC motor + gearbox).
%  
%  HISTORY:
%    - Compiled from datasheet specifications
%    - Motor: Faulhaber 2338S006S brushed DC motor
%    - Gearbox: Micromotor SA 23/1 planetary + external transmission
%
%  USAGE:
%    1. Run this script to initialize all parameters in workspace
%    2. Modify only PID tuning parameters (ts_5, Mp, alfa, wgc_p)
%    3. Variables saved to Generated_files/parameters.mat
%
%===========================================================================

clear all;
clc;

% =========================================================================
% CONVERSION FACTORS (fundamental constants)
% =========================================================================

% Angular velocity conversions
RPM_TO_RAD_S = 2*pi/60;             %   [rpm]   -> [rad/s]
RAD_S_TO_RPM = 60/(2*pi);           %   [rad/s] -> [rpm]

% Angular rate conversions
RPM_TO_DEG_S = 360/60;              %   [rpm]   -> [deg/s]
DEG_S_TO_RPM = 60/360;              %   [deg/s] -> [rpm]

% Angle conversions
DEG_TO_RAD = pi/180;                %   [deg]   -> [rad]
RAD_TO_DEG = 180/pi;                %   [rad]   -> [deg]

% Torque conversions
OZ_IN_TO_NM = 0.706e-2;             %   [oz*inch] -> [N*m]

% Legacy variable names (for compatibility)
rpm2rads = RPM_TO_RAD_S;
rads2rpm = RAD_S_TO_RPM;
rpm2degs = RPM_TO_DEG_S;
degs2rpm = DEG_S_TO_RPM;
deg2rad = DEG_TO_RAD;
rad2deg = RAD_TO_DEG;
ozin2Nm = OZ_IN_TO_NM;

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

% Real derivative parameters (second-order low-pass filter for differentiation)
RAD_INTEGRATOR_FREQ = 2*pi*20;      %   Integrator cutoff frequency [rad/s]
DAMPING_RATIO = 1/sqrt(2);          %   Damping ratio for filter poles [unitless]

rdp.wci = RAD_INTEGRATOR_FREQ;      %   Legacy variable name
rdp.di = DAMPING_RATIO;             %   Legacy variable name

% Simplified Motor transfer function

km = drv.dcgain / mot.Ke;  
Jl = mld.JD + 3*gbox.J72; 
Req = mot.R + sens.curr.Rs;
Jeq = mot.J + Jl/(gbox.N1^2);  
Tm = (Req * Jeq) / (mot.Kt * mot.Ke); 
s = tf('s');
P = km / (s * (Tm*s + 1) * gbox.N); % Ingresso [V], Uscita [rad] 
[numP, denP] = tfdata(P, 'v');

% =========================================================================
% PID CONTROLLER DESIGN SPECIFICATIONS
% =========================================================================
% These parameters define the desired closed-loop performance.
% Modify these only when tuning controller performance.

DESIRED_OVERSHOOT_PERCENT = 5;          %   Maximum overshoot [%]
DESIRED_SETTLING_TIME_S = 0.15;         %   5% settling time [s]
PID_TUNING_RATIO_ALFA = 4;              %   PID ratio parameter [unitless, 4-10 typical]
FREQUENCY_WEIGHT_WGC_P = 2;             %   Frequency weighting [unitless, 1-5 typical]
REFERENCE_SIGNAL_TYPE = "step";         %   Expected input: 'impulse', 'step', 'ramp'

% Convert to normalized form (for design functions)
ts_5 = DESIRED_SETTLING_TIME_S;
Mp = DESIRED_OVERSHOOT_PERCENT / 100;   % Normalize to [0,1] range
alfa = PID_TUNING_RATIO_ALFA;
wgc_p = FREQUENCY_WEIGHT_WGC_P;
signal_type = REFERENCE_SIGNAL_TYPE;

% Insert here the PID parameters or comment the parameters and uncomment and call the function
% design_pid_controller that apply the bode method to compute kp, kd, ki, tl

% call of the design_pid_controller function

[kp, ki, kd, tl, type] = design_pid_controller_automatized(Mp,ts_5,P,alfa,wgc_p,signal_type);

%Requirements verification

out = pid_metrics_automatized(P,kp,ki,kd,tl);

% Initialization of the input of LaB_0_real.slx (simulink simulations doesnt't
% start if a value is not assigned to these parameters

step_gain = 1;
simul.select2 = 1;
simul.select1 = 1;
simul.stair_gain = 1;


% parameter for confidence intervals. See Lab assignment: point 2.2) equation 12
var.c = 1.96;

% Save parameters for all simulations. If you change somethings in param.m
% file rerun the file after all saves

save('Lab_0_automatized\Generated_files\parameters.mat')

% === CONFIGURAZIONE CARTELLE SIMULINK ===

% Cartella base dove vuoi mettere TUTTI i file temporanei
baseDir = 'Lab_0_automatized\Generated_files';

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



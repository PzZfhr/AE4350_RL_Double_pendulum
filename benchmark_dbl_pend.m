%% Choose parameters of initial model
l1 = 0.1; l2 = 0.1; % lengths of first and second arms. [m]
m1 = 0.125; m2 = 0.05; % masses of first and second arms. [kg]
c1 = -0.04; c2 = 0.06; % centers of mass of first and second arms. [m]
I1 = 0.074; I2 = 0.00012; % inertias of arms. [kg m^2]
b1 = 4.8; b2 = 0.0002; % damping of joints. [kg/s]
km = 50; % motor gain. [Nm]
tau_e = 0.03; % motor electrical time constant. [s]

%% Choose parameters of white-box identification
l1 = 0.1; l2 = 0.1;
m1 = 0.125; m2 = 0.0493;
I1 = 0.0888; I2 = 9.5326e-05;
b1 = 7.7210; b2 = 2.337424773114795e-05;
c1 = -0.0320; c2 = 0.053232610186913;
km = 53.049364666321930; tau_e = 0.014999999999983;

%% Setup initial conditions and load agent
init_cond = [0.1, -0.1, 0, 0]; % initial position. (0,0) corresponds to up-up position. [rad] % initial velocity. [rad/s]
g = 9.81; % gravitational acceleration. [m/s^2]

agent = load('testAgent.mat');
agent = agent.saved_agent;
% Now need to run the simulink code directly to generate the data

%% Procesds results
th1exp = out.ScopeData1.signals(1).values;
th2exp = out.ScopeData1.signals(2).values;
th1dexp = out.ScopeData1.signals(3).values;
th2dexp = out.ScopeData1.signals(4).values;

reference_vector = zeros(length(th1exp), 1);

MSE_th1 = immse(th1exp, reference_vector)
MSE_th2 = immse(th2exp, reference_vector)
MSE_th1d = immse(th1dexp, reference_vector)
MSE_th2d = immse(th2dexp, reference_vector)

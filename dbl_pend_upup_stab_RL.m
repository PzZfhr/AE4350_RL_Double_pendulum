%% The goal of this code is for an agent to learn how to stabilize the double pendulum by having the double pendulum already start in the up-up
%% position, just like in the SC42045 Control Systems Lab course.

%% Define system initital conditions and characteristics
clear all
init_cond = [pi, 0, 0, 0]; % initial position. (0,0) corresponds to up-up position. [rad] % initial velocity. [rad/s]

% All of the following system values as well as the double pendulum model
% is taken directly from the TU Delft "SC42045 Control Systems Lab" course
% (given in Q3 of 2023)
g = 9.81; % gravitational acceleration. [m/s^2]
l1 = 0.1; l2 = 0.1; % lengths of first and second arms. [m]
m1 = 0.125; m2 = 0.05; % masses of first and second arms. [kg]
c1 = -0.04; c2 = 0.06; % centers of mass of first and second arms. [m]
I1 = 0.074; I2 = 0.00012; % inertias of arms. [kg m^2]
b1 = 4.8; b2 = 0.0002; % damping of joints. [kg/s]
km = 50; % motor gain. [Nm]
tau_e = 0.03; % motor electrical time constant. [s]

%% Define simulation parameters
T_final = 10; % total simulation time. [s]
T_step = 0.01; % time step of 10 ms. [s]

% weigths of error function
w = [1, 1, 0.05, 0.1]; % weights of squared absolute error (distance to 0) of, in order, th1, th2, th1d, th2d

% bounds not to be exceeded during simulation
bounds_th1 = [-pi, 3*pi];
% bounds_alpha_limit = 10*pi; % angle between second arm and horizontal
bounds_alpha_reward = pi/2 - 0.05;

% boolean parameters
loadPreviouslyTrainedAgent = true; 
explore = false;
doTraining = true; 
saveAgent = true;  

%% Open Simulink model
mdl = "dbl_pend_upup_stab_RL_simulink"; 
open_system(mdl)

%% Setup RL environment
% Create observations object
obsInfo = rlNumericSpec([4 1]);
obsInfo.Name = "observations";
obsInfo.Description = "Observations of the environment. Contains angles and angular velocities in the following order: [th1 th2 th1d th2d].";

% Create action object
actInfo = rlNumericSpec([1 1]);
actInfo.Name = "voltage";
actInfo.Description = "Action of the RL Agent. Corresponds to the motor voltage u. [V]";
actInfo.LowerLimit = -10;   % Minimum and maximum voltage values
actInfo.UpperLimit = 10;    % achievable by the agent. [V]

% Find agent path and create RL environment
agentBlk = mdl + "/Agent";      
env = rlSimulinkEnv(mdl,agentBlk,obsInfo,actInfo);
env.ResetFcn = @(in) setVariable(in, "init_cond", [pi, 0, 0, 0] + 0.05*randn(1,4), "Workspace", mdl); % Define reset function to slightly random initial conditions around up-up position

%% Create DDPG Agent
% Define state path
statePath = [
    featureInputLayer(obsInfo.Dimension(1), Name="obsPathInputLayer") % starts with a feature input layer that takes in the observation information
    fullyConnectedLayer(512)    
    reluLayer   % introduces non-linearity to the network
    fullyConnectedLayer(512)
    reluLayer
    fullyConnectedLayer(512, Name="spOutLayer")
    ];

% Define action path
actionPath = [
    featureInputLayer(actInfo.Dimension(1), Name="actPathInputLayer") % starts with a feature input layer that takes in the action information
    fullyConnectedLayer(512, Name="apOutLayer", BiasLearnRateFactor=0) % biases in the network will not be updated during training.
    ];

% Define common path
commonPath = [
    additionLayer(2, Name="add") % starts with an addition layer that adds the outputs from the state and action paths
    reluLayer
    fullyConnectedLayer(1) % Fully connected layer with 1 output. Represents the estimated Q-value, which evaluates the quality of a particular state-action pair.
    ];

% Create the critic network
criticNetwork = layerGraph();
criticNetwork = addLayers(criticNetwork, statePath);
criticNetwork = addLayers(criticNetwork, actionPath);
criticNetwork = addLayers(criticNetwork, commonPath);
criticNetwork = connectLayers(criticNetwork, "spOutLayer", "add/in1");
criticNetwork = connectLayers(criticNetwork, "apOutLayer", "add/in2");
criticNetwork = dlnetwork(criticNetwork);
summary(criticNetwork)
figure(1)
plot(criticNetwork)

% Increase the number of units in the actor network and add dropout regularization
actorNetwork = [
    featureInputLayer(obsInfo.Dimension(1))
    fullyConnectedLayer(512)
    reluLayer
    fullyConnectedLayer(512)
    reluLayer
    fullyConnectedLayer(512)
    reluLayer
    fullyConnectedLayer(1)
    tanhLayer
    scalingLayer(Scale=max(actInfo.UpperLimit))
    ];

actorNetwork = dlnetwork(actorNetwork);
summary(actorNetwork)
figure(2)
plot(actorNetwork)

% Create the critic and actor objects
critic = rlQValueFunction(criticNetwork, obsInfo, actInfo, ...
    ObservationInputNames="obsPathInputLayer", ...
    ActionInputNames="actPathInputLayer");

% Create the actor using actorNet and the observation and action specifications.
actor = rlContinuousDeterministicActor(actorNetwork,obsInfo,actInfo); % actor for continuous action spaces in reinforcement learning

% Specify options for the critic and actor using rlOptimizerOptions.
criticOpts = rlOptimizerOptions(LearnRate=1e-03,GradientThreshold=1);
actorOpts = rlOptimizerOptions(LearnRate=1e-04,GradientThreshold=1);

% Specify the DDPG agent options using rlDDPGAgentOptions, include the training options for the actor and critic.
agentOpts = rlDDPGAgentOptions(...
    SampleTime=T_step,... % time interval between consecutive observations and actions in the environment
    CriticOptimizerOptions=criticOpts,...
    ActorOptimizerOptions=actorOpts,...
    ExperienceBufferLength=1e6,... % Determines the number of past experiences stored for training
    DiscountFactor=0.975,... % Determines the importance of future rewards compared to immediate rewards. Closer to 1 gives more importance to future rewards.
    MiniBatchSize=128); % Specifies the number of experiences sampled from the replay buffer in each training iteration

%% Create the DDPG agent using the specified actor, critic, and agent options objects.
if loadPreviouslyTrainedAgent
    agent = load('oldAgent.mat');
    agent = agent.saved_agent;
    agent.AgentOptions.ResetExperienceBufferBeforeTraining = false;
else 
    agentOpts.ResetExperienceBufferBeforeTraining = false;
    agent = rlDDPGAgent(actor,critic,agentOpts);
end

if explore
    agent.UseExplorationPolicy = 1;
else
    agent.UseExplorationPolicy = 0;
end

agent.AgentOptions.SampleTime = 0.005; % 'Upgrade' sampling time to 5[ms] to hopefully control more accurately 

%% Train Agent
maxepisodes = 15000; % the total number of episodes to train
maxsteps = ceil(T_final/T_step); % The maximum number of steps (time steps) per episode
trainOpts = rlTrainingOptions(...
    MaxEpisodes=maxepisodes,...
    MaxStepsPerEpisode=maxsteps,...
    ScoreAveragingWindowLength=50,... % length of the window for averaging the episode scores in the training progress plot.
    Verbose=false,...
    Plots="training-progress",... % shows the training progress plot
    StopTrainingCriteria="AverageReward",... % criteria for stopping the training
    StopTrainingValue=-500,...
    SaveAgentCriteria="EpisodeCount",... % agent should be saved if training finished
    SaveAgentValue=1); % for now save any agent that trains for at least 1 episodes

if doTraining
    % Train the agent.
    trainingStats = train(agent,env,trainOpts);
end

if saveAgent
    save("newAgent.mat", "agent")
    save("newTrainingData.mat", "trainingStats")
end
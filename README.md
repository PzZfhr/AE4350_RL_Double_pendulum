# AE4350_RL_Double_pendulum
Code used in the TU Delft AE4350 course project (Q4 2023) to train an agent to control a double-pendulum system.

Description of files:

- dbl_pend_upup_stab_RL.m: Matlab code in which the RL process is implemented. Contains the creation of the agent, the call of the simulink model, the definititon of the training hyperparameters, and the training itself.
  
- dbl_pend_upup_stab_RL_simulink.slx: Simulink model containing the physical double pendulum model, the RL agent and RL circuit, the reward and penalty circuits, and the real-time vizualisation of the double pendulum.
  
- double_pendulum.slx: Simulink model of the double pendulum.
  
- plotDoublePend.m: Matlab function used to display the double-pendulum for a given set of angles.
  
- testAgent.mat: Final version of the trained agent.
  
- benchmark_dbl_pend.m: Matlab code used to simulate the time response of the agent to the 'benchmark' test described in the report, as well as to calculate the MSE of the agent.

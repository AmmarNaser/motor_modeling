%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  SCRIPT : Import DC motor log file to MATLAB workspace              %%
%%  BY     : Waleed El-Badry                                           %%
%%  DATE   : 13/08/2021      D:\laby\teams\modeling\Simulation 4\serialplot\                                          %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Read log csv file
file_path = 'D:\laby\teams\modeling\Simulation 4\serialplot\DATA_MOTOR_4.csv';
motor_log = readtable(file_path);

%% Store each column in a variable
t = motor_log.t;
speed = motor_log.SPEED;
pwm = motor_log.PWM;

%% export to mat file
save('motor_log','t','pwm','speed');
R= 2.0 % Ohms
L= 0.5 % Henrys
K = .015 % torque constant 
B = 0.2 % Nms
J= 0.02 % kg.m^2

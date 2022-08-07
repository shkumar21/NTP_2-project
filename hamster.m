% This code is written as a part of NTP 
%=========Author: Mohamed Soliman ========%
%=========Year :2021==========%
clc;
clear all;  %#ok<CLALL>
close all;

% Condition for compiling
EXPORT = 1;        

DifferentialState x1;                     % Position 
DifferentialState x2;                     % Speed  
Control u;                                % Control input


    
%% Diferential Equation
f = dot([x1; x2]) == [x2; u];
   

%define the step length
simTs = 0.4;           % time interval              
Num = 50;
numSteps = 2;
%define the objective function ( stage and end costs)
h = [diffStates; controls];
hN = (diffStates);
% Define number of sates and control signal
n_XD = length(diffStates);
n_U = length(controls);
 
%% SIMexport (Integrator Setting)
acadoSet('problemname', 'sim');

sim = acado.SIMexport( simTs );
sim.setModel(f);
sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_RIIA5' );
sim.set( 'NUM_INTEGRATOR_STEPS',        numSteps        );

if EXPORT
    sim.exportCode( 'export_SIM' ); %#ok<UNRCH>
    
    cd export_SIM
    make_acado_integrator('./../integrate_hamster')
    cd ..
end

%% MPCexport (optimal control setting)
acadoSet('problemname', 'NMPC');

ocp = acado.OCP( 0.0, simTs*Num, Num );
W_mat = eye(n_XD+n_U,n_XD+n_U);
WN_mat = eye(n_XD,n_XD);
W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);
ocp.minimizeLSQ( W, h);
ocp.minimizeLSQEndTerm( WN, hN );

ocp.subjectTo( (0 <= x1) <= 20 );     % Constraints on position
ocp.subjectTo( (0 <= x2)   <= 1.2 );  % Constraints on speed
%Hard constraints for obstacle avoidance 
% ocp.subjectTo( sqrt((x-m1x)*(x-m1x)  + (y-m1y)*(y-m1y)) - cylinder1_r  >= 1 )
% ocp.subjectTo( sqrt((x-m2x)*(x-m2x)  + (y-m2y)*(y-m2y)) - cylinder2_r  >= 1)

ocp.setModel(f);
mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'PRINTLEVEL',                  'NONE'              );
mpc.set( 'INTEGRATOR_TYPE',             'INT_EX_EULER'      );
mpc.set( 'CG_USE_OPENMP ',              'YES');
mpc.set( 'NUM_INTEGRATOR_STEPS',         2*Num                  );
mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-4				);

if EXPORT
    mpc.exportCode( 'export_MPC' );                   
    copyfile('../../../../../../../acado-master/external_packages/qpoases', 'export_MPC/qpoases', 'f')
%             copyfile('C:/Users/leeck/Desktop/ACADO/external_packages/qpoases','export_MPC/qpoases', 'f')
    cd export_MPC
    make_acado_solver('../acado_hamster')
    cd ..
end
%=========================================================================%
% Simulation Parameters 
N=50;
dt =0.2;
input.N =50;      % predicition Horizon
X0 = [0 0 ] ;
Xref = [20 0];
input.x = repmat(Xref,N+1,1);
Xref = repmat(Xref,N,1);

Uref = zeros(N,n_U);
input.u = Uref;

input.y = [Xref(1:N,:) Uref];
input.yN = Xref(N,:);
% Weightening Matrices 
input.W  = diag([0.02,   0.02,  0.02]);     
input.WN  = diag([3, 3]);       

 %=============================================================================%
disp('-----------------------HAMSTER --------------------------------')
disp('                  Simulation Loop'                                )
disp('---------------        Soliman ---------------------------------')

iter = 0; time = 0;
Tf = 30;
KKT_MPC = []; INFO_MPC = [];
controls_MPC = [];
state_sim = X0;
while time(end) < Tf
    input.x0 = state_sim(end,:);
    output = acado_hamster(input); % Solve our OCP
    
    input.x = output.x;
    input.u = output.u; 
    input_sim.x = state_sim(end,:)';
    input_sim.u = output.u(1,:)';                   % NMPC control
    states = integrate_hamster(input_sim);       % Simulate our system
    state_sim = [state_sim; states.value'];      
    % Save the MPC step
    INFO_MPC = [INFO_MPC; output.info]; 
    KKT_MPC = [KKT_MPC; output.info.kktValue];
    controls_MPC = [controls_MPC; output.u(1,:)];
    controls = [controls; output.u(1,:)];
    
    iter = iter+1;
    nextTime = iter*dt; 
    time = [time nextTime];
    result.CalTime = output.info.cpuTime;
    
end
 %=========Plot the results==========%
 figure(1)
 plot(time',state_sim(:,1),'k','LineWidth',2)
 grid on
 xlabel('Time [sec]')
 ylabel('Position [m]')
 
 %matlab2tikz('plot_states.tex');
 
 constV(1:1:length(time))=1.2;
 figure(2)
 plot(time',state_sim(:,2),'k','LineWidth',2)
 hold on
 plot(time,constV,'r','LineWidth',1)
 xlabel('Time [sec]')
 ylabel('Velocity [m/sec]')
 legend('Velocity','Constraints')
 %matlab2tikz('plot_states.tex');
 grid on


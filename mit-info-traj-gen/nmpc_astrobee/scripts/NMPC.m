clear all

%% Setup
ACADO_PATH = '/usr/local/MATLAB/ACADOtoolkit';  % set this to your ACADOtoolkit folder
SIM = 0;  % 0 for code export only, 1 for sim only, 2 for both

addpath(genpath(ACADO_PATH)); 
acadoSet('results_to_file', false); 

%% Declare all differential states, controls, parameters
DifferentialState r(3) v(3) q(4) w(3) psi(9)
Control u(6)
OnlineData mass(1) ixx(1) iyy(1) izz(1) r_des(3) v_des(3) q_des(4) w_des(3)  % these variables can be changed online
n_XD = length(diffStates);
n_U = length(controls);

I = diag([ixx, iyy, izz]); 

roff = [0;0;0];

%% Dynamic equations
f = [ dot(r); dot(v); dot(q); dot(w); dot(psi)] == ...
    [ v;               ...
      Rotmat(q)*u(1:3)/mass;     ...   % forces
      0.5*H_bar_T(q)*w; ...
      dot_w(I,w,u(4:6));
      Calc_psidot(r,v,q,w,mass,I,izz,u,psi)];  % torques  
  
%% Export NMPC solver
if SIM ~= 1
    time = 0; Ts = 1/62.5;
    N =40;  % number of iterations forward
    W_mat = eye(20);  % info gain(1) state error(12) inputs(6)
    WN_mat = eye(12);  % terminal cost, 13 DoF
    W = acado.BMatrix(W_mat);
    WN = acado.BMatrix(WN_mat);

    [FIM1, FIM2] = CalcFIM(psi)
    %---Optimization Problem---
    % Create NMPC object
    ocp = acado.OCP(0.0, N*Ts, N); % start time 0, end time, number of intervals. OCP(tStart, tEnd, N)

    % Set cost function
    ocp.minimizeLSQ( W, [1/(trace(FIM1)+1);1/(trace(FIM2)+1);r-r_des;  v-v_des;err_q(q,q_des); w-w_des;u]);% r-r_des; err_q(q,q_des); v-v_des; w-w_des]);  % Minimize inverse FIM + error between states, 1 to avoid singularity
    ocp.minimizeLSQEndTerm( WN, [r-r_des;  v-v_des;err_q(q,q_des); w-w_des]);

    % Set constraints
    ocp.subjectTo(-0.35 <= u(1:3) <= 0.35);  % force constraints
    ocp.subjectTo(-0.035 <= u(4:6) <= 0.035);  % torque control constraints
        
    % Obstacle constraints
    ocp.subjectTo(-1.0 <= r(1) <= 1.0);  % x position
    ocp.subjectTo(-1.0 <= r(2) <= 1.0);  % y position
    
    % An elliptical obstacle
%     eps_len_x = 0.1;
%     eps_len_y = 0.8;
%     eps_centroid = [0; .2];
%     
%     P = [1/((eps_len_x/2)^2), 0;
%          0, 1/((eps_len_y/2)^2)];  % shaping matrix
%     ocp.subjectTo( (r(1:2) - eps_centroid)'*P*(r(1:2) - eps_centroid) >= 1);  % ellipsoidal obstacle

    % Add dynamics constraints
    ocp.setModel(f);  % dynamics
    %---Optimization Problem---
    
    
    % Export NMPC code
    mpc = acado.OCPexport( ocp );
    mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
    mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
    mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
    mpc.set( 'INTEGRATOR_TYPE',              'INT_IRK_GL8'    );
    mpc.set( 'NUM_INTEGRATOR_STEPS',         10                 );
    mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
    mpc.set( 'HOTSTART_QP',                 'YES'             	);
    mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-5				);  % steps over horizon
    mpc.exportCode('NMPC_export');  % export code and compile
    copyfile(join([ACADO_PATH, '/external_packages/qpoases']), 'NMPC_export/qpoases', 'f'); % path to the acado folders, might very depending on the folder location
%     cd 'NMPC_export'
end
disp("Code generated!")

% Everything hereon is only needed for MATLAB simulation
if SIM ~= 0
     cd 'NMPC_export'
    make_acado_solver('../acado_NMPC_6DoF')  % place in subdirectory
    cd ..;

    %
    %export integrator
    acadoSet('problemname', 'sim');
    sim = acado.SIMexport(Ts); % sampling time
    sim.setModel(f);
    sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_RIIA5' );
    sim.set( 'NUM_INTEGRATOR_STEPS',        10      );
    sim.exportCode( 'SIM_export' );
    cd SIM_export
    make_acado_integrator('../acado_system')
    cd ..


    % orientation at start
    Q_begin = [0.0;0.0;0.0;1];

    % orientation desired at end
    q_des = [0.0, 0.0, 0.7071, 0.7071];
    %q_des = [0.0, 0.0, -0.288, 0.958];

    %initialize state and control trajectories
    X0 = [10.0 -7.2 4.95 0 0 0 Q_begin(1) Q_begin(2) Q_begin(3) Q_begin(4) 0.0 0.0 0.0 zeros(1,9)]; % initial values of states
    Xref = [10.0 -7.2 4.5 0.0 0.0 0.0 q_des(1) q_des(2) q_des(3) q_des(4) 0.0 0.0 0.0 zeros(1,9) ]; % reference values of states

    % current values
    input.x = repmat(X0, N+1, 1);
    % Xref rows are replicated N times. Column is one, so once(13 cols).

    %Uref = zeros(N, n_U); % matrix of horizonx number of control inputs
    input.u = repmat(ones(1,6)*0.1,N,1);
    input.y = repmat(zeros(1,20), N, 1);
    input.yN = zeros(1,12); % reference for the minimizing function, is always zero
    % 
    %% online data has the current values
    Mass = 9.583788668;
    ixx = 0.153427988291;
    iyy = 0.142714053392;
    izz = 0.16230276227;

    input.od(:,1:4) = repmat([Mass ixx iyy izz], N+1, 1); % inertial parameters which are set online
    input.od(:,5:17) = repmat([ 10.0 -7.2 4.95 0.0 0.0 0.0 q_des(1) q_des(2) q_des(3) q_des(4) 0.0 0.0 0.0 ], N+1, 1); % desired states

    % testing cost function values
    input.W = eye(20)*0.01; 
    input.W(1,1) = 0;
    input.W(2,2) = 0; input.W(3,3) =0.1; input.W(4,4) = 0.1; input.W(5,5) = 0.1;
    input.W(6,6) = 1; input.W(7,7) =1; input.W(8,8) = 1;
    for i = 9:11
        input.W(i,i) = 1;
    end
    for i = 12:14
        input.W(i,i) = 0.001;
    end

    input.WN = eye(12)*10;

    %% SIMULATION LOOP
    display('------------------------------------------------------------------')
    display('               Simulation Loop'                                    )
    display('------------------------------------------------------------------')

    r_err = []; v_err = []; q_err = []; w_err = [];
    time = 0;
    Tf = 30;
    state_sim = X0;
    time_store = [0];
    control_ip = [];
    while time < Tf
        tic
        % solv NMPC OCP
        input.x0 = [state_sim(end,:)];
        output = acado_NMPC_6DoF(input);
        output.info;
        % shift state and control trajectories
          input.x = [output.x(2:end,:); output.x(end,:)]; %forward shifting strategy is to take elements from second row and repeat last row.

         input.u = [output.u(2:end,:); output.u(end,:)];

        % to normalize the quaternions
        den = norm(state_sim(end,7:10));
        for i  = 7:10
            state_sim(end,i) = state_sim(end,i)/den;
        end
        input_sim.x = state_sim(end,:).';% current state of simulation
        input_sim.u = [output.u(1,1);output.u(1,2);0;0;0;output.u(1,6)];%first input from MPC optimization
        control_ip = [control_ip;output.u(1,1) output.u(1,2) 0 0 0 output.u(1,6)];
        input_sim.od = [Mass ixx iyy izz]';
        output_sim = acado_system(input_sim);
        %simulated system's output found by 
        state_sim = [state_sim ; output_sim.value'];

        info{1} = output_sim.value;
        info{2} = time;
        %W = gamma_shifter(input.W, info);
        %input.W = W;  % update gamma   
        time = time + Ts;
        time_store = [time_store; time];
        disp(['current time: ' num2str(time) '   ' char(9) ' (RTI step: ' num2str(output.info.cpuTime*1e6) ' µs)'])
    end
    %plotting func
    visualize;
end

%% Supporting functions

% quaternion metric for cost function
function [error_quat] = err_q2(q,q_des)
%     error_quat = sqrt((q'*q_des)^2);
    error_quat = sqrt(2*(1 - sqrt((q'*q_des)^2)));
end

% quaternion error angle calculation for cost function
function [error_quat] = err_q(q,q_des)
    error_quat = invskew(Rotmat(q_des)'*Rotmat(q) - Rotmat(q)'*Rotmat(q_des))/(2*sqrt(1+tr(Rotmat(q_des)'*Rotmat(q))));
end
function [skew_v] = skew(vec)
    skew_v = [0 -vec(3) vec(2) ; vec(3) 0 -vec(1) ; -vec(2) vec(1) 0 ];  % Cross product matrix
end

function [vec] = invskew(A)
    vec = [A(3,2); A(1,3); A(2,1)];
end
function [Trace] = tr(A)
    Trace =  A(1,1) + A(2,2) + A(3,3);  % Trace
end

% rotation matrix for R_I_B: quaternion uses scalar last convention
function [R] = Rotmat(q)
    R = [q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)),          2*(q(1)*q(3)-q(2)*q(4));...
             2*(q(1)*q(2)-q(3)*q(4)),        -q(1)^2+q(2)^2-q(3)^2 + q(4)^2, 2*(q(2)*q(3)+q(1)*q(4));...
             2*(q(1)*q(3)+q(2)*q(4)),        2*(q(2)*q(3)-q(1)*q(4)),          -q(1)^2-q(2)^2+q(3)^2+q(4)^2]';
end

% quaternion conversion matrix: quaternion uses scalar last convention,
% w_IB expressed in body frame
function out = H_bar_T(q)
    out = [ q(4) q(3) -q(2) -q(1);
              -q(3) q(4) q(1) -q(2);
              q(2) -q(1) q(4) -q(3)]';
end

% dot_w calculation from xyz torque
function out = dot_w(I, w, tau)
    inv_I = [1/I(1,1)   0         0    ;
                0      1/I(2,2)   0    ;
                0       0        1/I(3,3)];
    out = (-inv_I * cross(w, I*w) + inv_I*tau);
end

function[psi_dot] = Calc_psidot(x,v,q,w,Mass,I,izz,u,psi)

    f1 = [v(1:2);               ...
      Rotmat(q)*u(1:3)/Mass;]  % forces                    
    f2 = [0.5*(w(3)*q(2)); 0.5*(w(3)*q(1)); 0.5*(w(3)*q(4)); 0.5*(w(3)*q(3)); ...
        u(6)/izz]; % attitude


    df_x1 = simplify(jacobian([f1(1:4)],[x(1:2);v(1:2)])); % Jacobian (df/dx)
    df_x2 =  simplify(jacobian(f2,[q;w(3)]));       % attitude


    df_theta1 =  simplify(jacobian(f1(1:4),Mass)); % Jacobian (df/dtheta)
    df_theta2 =  simplify(jacobian(f2,izz))


    psi_dot(1:4,1) = df_x1*psi(1:4)+df_theta1; %df/dMass  % d/dt [ dx/dtheta ]
    psi_dot(5:9,1) = df_x2*psi(5:9)+df_theta2; %df/dI_xx  % d/dt [ dx/dtheta ]

    psi_dot;
end

function [FIM1, FIM2] = CalcFIM(psi)

% measurements = v_x, v_y,v_z
    dh_x1 = [ 0 0 1 0;
              0 0 0 1];% dh/dx == dy/dx

% measurements w_x, w_y, w_z
    dh_x2 = [0 0 0 0 1];  % dh/dx == dy/dx

    sigma  = 0.01*ones(1,1);
    inv_R = inv(diag([sigma]));


    psi_matrix1 = [psi(1:4)];
    psi_matrix2 = [ psi(5:9)];

    FIM1 = ((dh_x1*psi_matrix1)'*inv_R*(dh_x1*psi_matrix1))   % H' * inv_R * H = [1 x 1]
    FIM2 = ((dh_x2*psi_matrix2)'*inv_R*(dh_x2*psi_matrix2))    % H' * inv_R * H = [1 x 1]

%     FIM =[ FIM1 0;
%             0      FIM2     ];
end

function df_dx = quat_df_dx(q, w)
    df_dx = [ 0     w(3) -w(2) w(1)  q(4) -q(3)  q(2);
             -w(3)   0    w(1) w(2)  q(3)  q(4) -q(1);
              w(2) -w(1)  0    w(3) -q(2)  q(1)  q(4);
             -w(1) -w(2) -w(3)  0   -q(1) -q(2) -q(3)];
end

function W_out = gamma_shifter(W, info)
    % gamma is the first term
    gamma = W(1, 1);
    x = info{1};
    t = info{2};
    tau = 20;  % time constant for decay by a factor of e

    % adjust gamma for decent tracking
    % options: distance to goal; quality of tracking; change in parameter
    % estimates; hard cutoff after enough time

    % for now, it just ramps down as you get closer to x_g
    gamma = 20*exp(1)^(-1/tau*t);
%     gamma = gamma + info{3}; % quaternion scalar element does not let this go to zero
    if gamma > 20
        gamma = 20;
    end  % limit max
    W(1, 1) = gamma;
    W_out = W;
end


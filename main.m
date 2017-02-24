clc
close all
clear

%% flags
plot_flag = 1;
animate_flag = 1;

%% system parameters
m = 1;                   % mass
b = 0;                   % external damping constant
k = 0.01;                 % spring constant
r = 0.2;                 % internal damping
l = 5;                   % desired distance between vehicles

N = 3;                   % number or vehicles
I = eye(N);              % identity matrix
S = diag(ones(N,1),0) - ...
    diag(ones(N-1,1),-1);% connectivity matrix
M = m*I;                 % mass matrix
K = k*I;                 % spring matrix
B = b*I;                 % drag matrix
D = r*ones(1,N);         % dissipation
Ru = diag(D)- diag(D(:,1:N-1),-1); % unidirectional dissipation matrix
R = diag(D) - diag(D(:,2:N),1) - ...
    diag(D(:,1:N-1),-1) + ...
    diag([D(:,2:N),0]);  % bidirectional dissipation matrix
Ref.A = -0.1;            % reference velocity amplitude (first vehicle)
Ref.f = 0.0628;          % reference velocity frequency
Ref.phi = 0;             % reference velocity phase

%% simulation parameters
t_end = 200;             % end time
t_step = .1;             % time steps
t_lsim = 0:t_step:t_end; % simulation time
v0 = zeros(1,N);         % initial velocities
p0 = M*v0';              % initial generalized momenta
n = 0:1:N-1;             % vector number vehicle in the string
q0 = l*n;                % absolute initial positions
delta0 = [-l,q0(1:N-1)] - q0 + l; % initial relative positions
energy0 = 1/2*(M'*p0.^2)+1/2*K*(delta0'.^2);

%% simulate
f_handle = @(t,x)simulate(t,x,N,B,Ru,S,K,M,Ref); %unidirectional
[t,x] = ode15s(f_handle,t_lsim,[p0;delta0';energy0]);

%% plot
if plot_flag
    figure(1);
    %s = scatter(q0,zeros(1,N),40,'filled');
    xlabel('x [m]');
    
    figure(2)
    hold on
    plot(t,x(:,N+1:2*N));
    xlabel('time [s]');
    ylabel('deltas [m]');
    
    figure(3)
    hold on
    plot(t,x(:,1:N)/m);
    xlabel('time [s]');
    ylabel('velocities [m/s]');
    
    figure(4)
    hold on
    for j = 1:N
        pos(:,j) = q0(j)+cumsum(x(:,j).*t_step);
    end
    plot(t,pos);
    xlabel('time [s]');
    ylabel('positions [m]');
    
    figure(5)
    hold on
    energy = 1/2*(x(:,1:N).^2/m)+1/2*k*(x(:,N+1:2*N).^2);
    plot(t,energy);
    xlabel('time [s]');
    ylabel('energy [J]');
    
    figure(6)
    hold on
    plot(x(:,N+1:2*N),x(:,1:N));
    xlabel('delta [m]');
    ylabel('velocity [m/s]');
    
    figure(7)
    hold on
    plot(t,x(:,2*N+1:end));
    xlabel('time [s]');
    ylabel('power integrated [J]');
    
    if animate_flag
        figure(1);
        xlim([min(min(pos)),max(max(pos))])
        hold on
        for t = 1:size(t_lsim,2)
            s = scatter(pos(t,:),zeros(1,N),[],'b','filled');
            drawnow limitrate;
            delete(s);
        end
    end
end
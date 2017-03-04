function xdot = simulate(t,x,N,B,R,S,K,M,F,epsilon)
%SIMULATE simulates a unidirectional string of vehicles
% change S'*K to eye(N)*K and R = diag(D)- diag(D(:,2:N),-1)
% to switch to unidirectional architecture

x1 = x(1:N);
x2 = x(N+1:2*N);
x3 = x(2*N+1:3*N);

if (F.f ==0)
    Mv0 = M(1,1)*F.A;
else
    Mv0 = M(1,1)*F.A*sin(F.f*t + F.phi);
end

alpha = (x3>epsilon);
beta = (x3<epsilon);

%alpha = ones(size(N));
Alpha = diag(alpha);
B = 0.021*B*diag(beta);

x1 = x1 - Mv0;

xdot1 = -B/M*(x1+Mv0) + Alpha*K*x2;
xdot2 = -S/M*x1;
xdot3 = 1./x3.*(B/M*(x1+Mv0).^2)-1./x3.*(Alpha*K*x2).*x1;
for i = 1:N
    if x3(i)>1.12 && xdot3(i)>0
        xdot3(i) = 0;
    end
end
xdot=[xdot1;xdot2;xdot3];

%xdot = [-Alpha*R/M-B/M, Alpha*K, zeros(size(S));
%        -S/M, zeros(size(S)), zeros(size(S));
%        zeros(size(S)), zeros(size(S)), zeros(size(S))]*x;
end
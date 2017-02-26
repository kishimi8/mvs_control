function xdot = simulate(t,x,N,B,R,S,K,M,F,epsilon)
%SIMULATE simulates a unidirectional string of vehicles
% change S'*K to eye(N)*K and R = diag(D)- diag(D(:,2:N),-1)
% to switch to unidirectional architecture

if (F.f ==0)
    Mv0 = M(1,1)*F.A;
else
    Mv0 = M(1,1)*F.A*sin(F.f*t + F.phi);
end

alpha = ones(size(N));%(x(2*N+1:3*N)>epsilon);
Alpha = diag(alpha);

x(1:size(S,1),1) = x(1:size(S,1),1) - Mv0;
xdot = [-R/M, Alpha*K, zeros(size(S));
        -S/M, zeros(size(S)), zeros(size(S));
        zeros(size(S)), zeros(size(S)), zeros(size(S))]*x;
xdot(2*N+1:3*N)= 1./x(2*N+1:3*N).*(R/M*x(1:N).^2) ...
    -1./x(2*N+1:3*N).*(K*x(N+1:2*N)).*x(1:N);        

end
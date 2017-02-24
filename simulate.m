function xdot = simulate(t,x,N,B,R,S,K,M,F)
%SIMULATE simulates a unidirectional string of vehicles
% change S'*K to eye(N)*K and R = diag(D)- diag(D(:,2:N),-1)
% to switch to unidirectional architecture

if (F.f ==0)
    Mv0 = M(1,1)*F.A;
else
    Mv0 = M(1,1)*F.A*sin(F.f*t + F.phi);
end

x(1:size(S,1),1) = x(1:size(S,1),1) - Mv0;
xdot = [-R/M, eye(size(S))'*K, zeros(size(S));
        -S/M, zeros(size(S)), zeros(size(S));
        zeros(size(S)), zeros(size(S)), zeros(size(S))]*x;

xdot(2*N+1:end,1) = ((x(1:N,1)+Mv0)).*(K*x(N+1:2*N,1) - R/M*(x(1:N,1)));

end
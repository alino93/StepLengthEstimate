N = 100000;
x = sdpvar(2,N);
optimize(cone([ones(1,N);x]),sum(sum(x)),sdpsettings('solver', 'mosek'))
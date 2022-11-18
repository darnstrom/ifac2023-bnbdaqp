%% Init
% Add path to DIR/daqp/interfaces/daqp-matlab/
% where dir is the save path to daqp
addpath ~/Projects/Research/code/daqp/interfaces/daqp-matlab/
%% Run random examples 
rng(1);
dims = 2:2:40;
nruns = 50;
times = zeros(length(dims),nruns,3);
iters = zeros(length(dims),nruns,3);
nodes = zeros(length(dims),nruns,3);
flags = zeros(length(dims),nruns,1);
xdiff= zeros(length(dims),nruns,1);
fvaldiff= zeros(length(dims),nruns,1);
grb_params.Threads=1;
grb_params.OutputFlag=0;
for nrun = 1:nruns
    i_dim = 0;
    for dim = dims 
        [nrun,dim]
        i_dim = i_dim+1;
        % generate problem
        n = 5*dim;
        m = 10*dim;
        ms = dim;

        H = full(sprandsym(n,1,1e-4,2));
        f = 100*randn(n,1); 
        f(1:ms) = -sign(f(1:ms)).*f(1:ms);
        A = randn(m,n);
        bupper = 20*rand(m,1);
        blower = -20*rand(m,1);
        bupper_tot = [ones(ms,1);bupper];
        blower_tot = [zeros(ms,1);blower];
        sense = int32(zeros(m+ms,1));
        sense(1:ms) = sense(1:ms)+16;
        d = daqp();
        [~,setup_time] = d.setup(H,f,A,bupper_tot,blower_tot,sense);
        d.settings('iter_limit',1e4);
        [xstar,fval,exitflag,info] = d.solve();
        fval = 0.5*xstar'*H*xstar+f'*xstar;
        times(i_dim,nrun,1)=setup_time+info.solve_time;
        iters(i_dim,nrun,1)=info.iter;
        nodes(i_dim,nrun,1)=info.nodes;
        flags(i_dim,nrun,1)=exitflag;

        % Solve with Gurobi
        model.Q = 0.5*sparse(H);
        model.A = sparse([A;-A]);
        model.rhs = [bupper;-blower];
        model.obj = f;
        model.sense='<';
        model.vtype=repelem('BC',[ms,n-ms]);
        model.lb = -inf(n,1);
        results = gurobi(model,grb_params);
        times(i_dim,nrun,2)=results.runtime;
        iters(i_dim,nrun,2)=results.itercount;
        nodes(i_dim,nrun,2)=results.nodecount;

        xgrb = results.x;
        fgrb = 0.5*xgrb'*H*xgrb+f'*xgrb;

        xdiff(i_dim,nrun,1) = norm(xstar-xgrb);
        fvaldiff(i_dim,nrun,1) = fgrb-fval;  
    end
end


%% Compare median times
close all;
semilogy(median(times(:,:,1),2))
hold on; 
semilogy(median(times(:,:,2),2))
hold off;
legend('DAQP','Gurobi')
title('Median')

%% Compare median iters 
close all;
semilogy(median(iters(:,:,1),2))
hold on; 
semilogy(median(iters(:,:,2),2))
hold off;
legend('DAQP','Gurobi')
title('Median')

%% Compare median nodes 
close all;
semilogy(median(nodes(:,:,1),2))
hold on; 
semilogy(median(nodes(:,:,2),2))
hold off;
legend('DAQP','Gurobi')
title('Median')

%% Compare worst-case times 
close all;
semilogy(max(times(:,:,1),[],2))
hold on; 
semilogy(max(times(:,:,2),[],2))
hold off;
legend('DAQP','Gurobi')
title('Max')

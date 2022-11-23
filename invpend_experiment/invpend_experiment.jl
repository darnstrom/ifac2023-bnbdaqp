## Init 
using LinearMPC
using UnicodePlots
using LinearAlgebra
using MatrixEquations
Np,Nc = 6,6

## Formulate MPC problem
mc,mp,g,l,d=1,1,10,1,0.5;
κ,ν = 100,10;

A = [0 0 1 0;
     0 0 0 1; 
     0 (mp*g/mc) 0 0; 
     0 (mc+mp)*g/(mc*l) 0 0];
B = [0 0;
     0 0;
     1/mc 0;
     1/(mc*l) -1/(mp*l);]
B = [B zeros(4,2)]; 

C = I(4) 
D = zeros(4,1);
Ts = 0.1;

F,G = LinearMPC.zoh(A,B,Ts);

# State constraints 
uby = [d;pi/10;2;1];
lby = -uby

# MPC
mpc = LinearMPC.MPC(F,G,C,Np);
mpc.Nc = Nc;

mpc.weights.Q = diagm(1*[1.0,1,1,1]); 
mpc.weights.R = 1*diagm([1.0;1e-4;zeros(2)])
mpc.weights.Rr = diagm([0;zeros(3)])
Qf,~ = ared(mpc.F,mpc.G[:,1],mpc.weights.R[1:1,1:1],mpc.weights.Q)
mpc.weights.Qf= Qf 

mpc.constraints.lb = [-1;0;zeros(2)];
mpc.constraints.ub = [1;1e30;ones(2)];
mpc.constraints.Ncc = mpc.Nc;
mpc.constraints.binary_controls = collect(3:4);

mpc.settings.QP_double_sided= true;
mpc.settings.reference_tracking=false;
mpc.settings.soft_constraints=false;

Cy = C
mpc.constraints.Cy = [Cy];
mpc.constraints.lby = [lby];
mpc.constraints.uby = [uby];
mpc.constraints.Ncy = [1:mpc.Nc]

δ2l, δ2u = -uby[1]+l*lby[2]-d, -lby[1]+l*uby[2]-d
dotδ2l, dotδ2u = -uby[3]+l*lby[4], -lby[3]+l*uby[4] 

u2l,u2u = κ*δ2l+ν*dotδ2l, κ*δ2u+ν*dotδ2u

Ax = [-1 l 0 0;
      1 -l 0 0;
      -κ κ*l -ν ν*l;
      κ -κ*l ν -ν*l;
      zeros(2,4);
      κ -κ*l ν -ν*l;
      -κ κ*l -ν ν*l;
     ]
Au = [0 0 -δ2u 0;
       0 0 -δ2l 0;
       0 0 0 -u2u;
       0 0 0 -u2l;
       0 1 -u2u 0;
       0 1 0 -u2u;
       0 1 0 -u2l;
       0 -1 dotδ2u 0;
      ]
bg = [d; 
       -δ2l-d;
       κ*d;
       -κ*d-u2l;
       0;
       0;
       -u2l-κ*d;
       dotδ2u+κ*d]

mpc.constraints.Au = Au;
mpc.constraints.Ax = Ax;
mpc.constraints.bg = bg;
mpc.constraints.Ncg = mpc.Nc;

mpQP = LinearMPC.mpc2mpqp(mpc);
mpLDP = LinearMPC.dualize(mpQP,1)

## Simulate
Nsim = 50 
x0 = [0.0, 0*pi/180, -1, 0.0];

function invpend_impulse(x,u,k)
    if(k==20)
        x[3] = -0.7
    end
end
mpc.settings.solver_opts =  Dict(:iter_limit=>1e4,
                                 :cycle_tol=>25,
                                 :primal_tol=>1e-6,
                                 :progress_tol=>1e-4,
                                 :pivot_tol=>1e-3)
xs,us,rs = LinearMPC.simulate(mpc,x0,Nsim;callback=invpend_impulse)

## Code generation
#opts = Dict(:primal_tol => 1e-4)
opts = Dict(:zero_tol => 1e-7,
            :primal_tol => 1e-4,
            :dual_tol => 1e-4,
            :cycle_tol => 25,
            :iter_limit => 1e4)
LinearMPC.codegen(mpc,opt_settings=opts);
cp("mpc_workspace.c","bnb_invpend_STM32F11RE/Core/Src/mpc_workspace.c";force=true);
cp("mpc_workspace.h","bnb_invpend_STM32F11RE/Core/Inc/mpc_workspace.h";force=true);
rm("mpc_workspace.c");
rm("mpc_workspace.h");

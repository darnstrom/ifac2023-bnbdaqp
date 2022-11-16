## Connect to Nucleo
using LibSerialPort
nucleo = LibSerialPort.open("/dev/ttyACM0",115200) 
set_flow_control(nucleo; xonxoff = SP_XONXOFF_DISABLED, dtr = SP_DTR_OFF, rts = SP_RTS_OFF)
get_port_settings(nucleo)
#close(nucleo)
## Define simulation model 
function invpend_step(F,G,x,u;κ=100,ν=10,d=0.5,l=1,Nsteps=10)
    for i = 1:Nsteps
        δ = -x[1]+l*x[2]-d
        δdot =  -x[3]+l*x[4]
        if(δ >= 0 && κ*δ+ν*δdot>=0)
            u2 = κ*δ+ν*δdot
        else
            u2 = 0
        end
        x = F*x + G[:,1:2]*[u[1]; u2]
        println(" $u2 & $(u[2])")
        #readline()
    end
    return x
end
## Initialize
data = read(nucleo)
u = zeros(4);
x = [0.0, 0, -1, 0];
Nsteps = 50;
exitflags = zeros(Nsteps);
cycles = zeros(Nsteps);
n_nodes = zeros(Nsteps);
n_iters = zeros(Nsteps);
us = zeros(mpc.nu,Nsteps)
xs = zeros(mpc.nx,Nsteps)
## Step on Nucleo
Fsim,Gsim = LinearMPC.zoh(A,B,Ts/2);
for k = 1:Nsteps
    xs[:,k] = x;
    display(k)
    write(nucleo,Cfloat.(x));
    flush(nucleo)
    #write(nucleo,Cfloat.(x));
    #raw_data=read(nucleo)
    wait_count = 0
    while (bytesavailable(nucleo) < 32)
        wait_count +=1
        @assert(wait_count < 20)
        sleep(0.1)
    end
    for i = 1:4
        u[i] = read(nucleo,Cfloat)
    end
    cycles[k] = read(nucleo,Cfloat)
    exitflags[k] = read(nucleo,Cfloat)
    n_nodes[k] = read(nucleo,Cfloat)
    n_iters[k] = read(nucleo,Cfloat)
    us[:,k] = u
    @assert(exitflags[k]==1)

    # step
    #x = mpc.F*x+mpc.G*u
    x = invpend_step(F,G,x,u;Nsteps=1)
    if(k==20)
        x[3] = -0.7
    end
#    sleep(0.25)
end

## Write to file 
using DelimitedFiles
open("result.dat"; write=true) do f
  write(f, "step time u1 u2 u3 u4 x1 x2 x3 x4 delta\n")
  writedlm(f, [collect(1:Nsteps) cycles/1e2*2 us' xs' delta])
end


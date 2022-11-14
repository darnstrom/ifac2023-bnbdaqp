## Connect to Nucleo
using LibSerialPort
nucleo = LibSerialPort.open("/dev/ttyACM0",115200) 
#close(nucleo)
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
check = 0
Fsim,Gsim = LinearMPC.zoh(A,B,Ts/2);
for k = 1:Nsteps
    xs[:,k] = x;
    display(k)
    write(nucleo,Cfloat.([x;0]));
    #write(nucleo,Cfloat.(x));
    #raw_data=read(nucleo)
    wait_count = 0
    while (bytesavailable(nucleo) < 36)
        if(wait_count > 10)
            println("trying to resend!")
            data_flush = read(nucleo)
            write(nucleo,Cfloat.([x.+1e-4;0]));
            wait_count = 0
        end
        sleep(0.25)
        wait_count +=1
    end
    check = read(nucleo,Cfloat);
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
    x = LinearMPC.invpend_step(F,G,x,u;Nsteps=1)
    if(k==20)
        x[3] = -0.75
    end
#    sleep(0.25)
end

## Write to file 
using DelimitedFiles
open("result.dat"; write=true) do f
  write(f, "step time u1 u2 u3 u4 x1 x2 x3 x4 delta\n")
  writedlm(f, [collect(1:Nsteps) cycles/1e2*2 us' xs' delta])
end


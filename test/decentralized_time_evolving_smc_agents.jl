using ErgodicControl
using ErgodicControlPlots

# Generate the distribution
N = 80
d = Domain([1,1], [100,100])
cov = 0.010 * eye(2)
phi = zeros(100,100,N+1)
for i = 1:N+1
    mui = (.7*(i-1)/N + .15) * ones(2)
    phi[:,:,i] = gaussian(d, mui, cov)
end
ErgodicControl.normalize!(phi, d.cell_size / (N+1))

# Now let's create the ergodic manager in R3
K = 5
em = ErgodicManagerR2T(d, phi, K)

# Set up first trajectory manager
x0 = [0.49,0.01]
h = 0.05
ci = ConstantInitializer([0.0, 0.0])
tm1 = TrajectoryManager(em, x0, h, N, ci)
tm1.xy = [0.49, 0.01]
dynamics!(tm1, SingleIntegrator(2,h))

# second tm is like the first, but different starting point
tm2 = deepcopy(tm1)
tm2.x0= [.79,.99]
tm2.xy= [.79,.99]


# tm3 = deepcopy(tm1)
# tm3.x0 = [.10,.24]
# tm3.xy = [.10,.24]
#
# tm4 = deepcopy(tm1)
# tm4.x0 = [.5,.02]
# tm4.xy = [.5,.02]
#
# tm5 = deepcopy(tm1)
# tm5.x0 = [.23,.056]
# tm5.xy = [.23,.056]
#
# tm6 = deepcopy(tm1)
# tm6.x0 = [.06,.1]
# tm6.xy = [.06,.1]
#
# tm7 = deepcopy(tm1)
# tm7.x0 = [.23,.65]
# tm7.xy = [.23,.65]
#
# tm8 = deepcopy(tm1)
# tm8.x0 = [.7,.11]
# tm8.xy = [.7,.11]
#
# tm9 = deepcopy(tm1)
# tm9.x0 = [.8,.8]
# tm9.xy = [.8,.8]
#
# tm10 = deepcopy(tm1)
# tm10.x0 = [.1,.41]
# tm10.xy = [.1,.41]

# array of trajectory managers
vtm = [tm1, tm2]

# # Generate the trajectories
# ddc = 1e-4
# tm1.barrier_cost = 1
# tm2.barrier_cost = 1
xd, ud = smc_trajectory_st(em, vtm, 0.2)

# # trajectory generation and plotting
# v = true
# xd,ud = smc_trajectory_s(em, vtm, 0.3, verbose=v)

# generating the gif

gif(em, xd, vtm)

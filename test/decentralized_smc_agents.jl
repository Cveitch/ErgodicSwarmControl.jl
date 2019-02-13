using ErgodicControl
using ErgodicControlPlots

# Set up different domains with different discretizations
d = Domain([1,1], 100)
num_agents = 10

# Set up distribution and ergodic manager
K = 5
means = [[.3,.7], [.7,.3]]
Sigmas = [.025*eye(2), .025*eye(2)]
 phi = gaussian(d, means, Sigmas)
#phi = circle(d, [0.5, 0.5], 0.25)
em = ErgodicManagerR2(d, phi, K)

# Set up first trajectory manager
x0 = [0.49,0.01]
N = 80
h = 0.2
ci = ConstantInitializer([0.0, 0.0])
tm1 = TrajectoryManager(em, x0, h, N, ci)
dynamics!(tm1, SingleIntegrator(2,h))

tm2 = deepcopy(tm1)
tm2.x0= [.79,.99]
tm2.xy= [.79,.99]


tm3 = deepcopy(tm1)
tm3.x0 = [.10,.24]
tm3.xy = [.10,.24]

tm4 = deepcopy(tm1)
tm4.x0 = [.5,.02]
tm4.xy = [.5,.02]

tm5 = deepcopy(tm1)
tm5.x0 = [.23,.056]
tm5.xy = [.23,.056]

tm6 = deepcopy(tm1)
tm6.x0 = [.06,.1]
tm6.xy = [.06,.1]

tm7 = deepcopy(tm1)
tm7.x0 = [.23,.65]
tm7.xy = [.23,.65]

tm8 = deepcopy(tm1)
tm8.x0 = [.7,.11]
tm8.xy = [.7,.11]

tm9 = deepcopy(tm1)
tm9.x0 = [.8,.8]
tm9.xy = [.8,.8]

tm10 = deepcopy(tm1)
tm10.x0 = [.1,.41]
tm10.xy = [.1,.41]

tm11 = deepcopy(tm1)
tm11.x0 = [rand(), rand()]
tm11.xy = tm11.x0

tm12 = deepcopy(tm1)
tm12.x0 = [rand(), rand()]
tm12.xy = tm12.x0

tm13 = deepcopy(tm1)
tm13.x0 = [rand(), rand()]
tm13.xy = tm13.x0

tm14 = deepcopy(tm1)
tm14.x0 = [rand(), rand()]
tm14.xy = tm14.x0

tm15 = deepcopy(tm1)
tm15.x0 = [rand(), rand()]
tm15.xy = tm15.x0

tm16 = deepcopy(tm1)
tm16.x0 = [rand(), rand()]
tm16.xy = tm16.x0

tm17 = deepcopy(tm1)
tm17.x0 = [rand(), rand()]
tm17.xy = tm17.x0

tm18 = deepcopy(tm1)
tm18.x0 = [rand(), rand()]
tm18.xy = tm18.x0

tm19 = deepcopy(tm1)
tm19.x0 = [rand(), rand()]
tm19.xy = tm19.x0

tm20 = deepcopy(tm1)
tm20.x0 = [rand(), rand()]
tm20.xy = tm20.x0

tm21 = deepcopy(tm1)
tm21.x0 = [rand(), rand()]
tm21.xy = tm21.x0

tm22 = deepcopy(tm1)
tm22.x0 = [rand(), rand()]
tm22.xy = tm22.x0

tm23 = deepcopy(tm1)
tm23.x0 = [rand(), rand()]
tm23.xy = tm23.x0

tm24 = deepcopy(tm1)
tm24.x0 = [rand(), rand()]
tm24.xy = tm24.x0

tm25 = deepcopy(tm1)
tm25.x0 = [rand(), rand()]
tm25.xy = tm25.x0

tm26 = deepcopy(tm1)
tm26.x0 = [rand(), rand()]
tm26.xy = tm26.x0

tm27 = deepcopy(tm1)
tm27.x0 = [rand(), rand()]
tm27.xy = tm27.x0

tm28 = deepcopy(tm1)
tm28.x0 = [rand(), rand()]
tm28.xy = tm28.x0

tm29 = deepcopy(tm1)
tm29.x0 = [rand(), rand()]
tm29.xy = tm29.x0

tm30 = deepcopy(tm1)
tm30.x0 = [rand(), rand()]
tm30.xy = tm30.x0

tm31 = deepcopy(tm1)
tm31.x0 = [rand(), rand()]
tm31.xy = tm31.x0

tm32 = deepcopy(tm1)
tm32.x0 = [rand(), rand()]
tm32.xy = tm32.x0

tm33 = deepcopy(tm1)
tm33.x0 = [rand(), rand()]
tm33.xy = tm33.x0

tm34 = deepcopy(tm1)
tm34.x0 = [rand(), rand()]
tm34.xy = tm34.x0

tm35 = deepcopy(tm1)
tm35.x0 = [rand(), rand()]
tm35.xy = tm35.x0

tm36 = deepcopy(tm1)
tm36.x0 = [rand(), rand()]
tm36.xy = tm36.x0

tm37 = deepcopy(tm1)
tm37.x0 = [rand(), rand()]
tm37.xy = tm37.x0

tm38 = deepcopy(tm1)
tm38.x0 = [rand(), rand()]
tm38.xy = tm38.x0

tm39 = deepcopy(tm1)
tm39.x0 = [rand(), rand()]
tm39.xy = tm39.x0

tm40 = deepcopy(tm1)
tm40.x0 = [rand(), rand()]
tm40.xy = tm40.x0

tm41 = deepcopy(tm1)
tm41.x0 = [rand(), rand()]
tm41.xy = tm41.x0

tm42 = deepcopy(tm1)
tm42.x0 = [rand(), rand()]
tm42.xy = tm42.x0

tm44 = deepcopy(tm1)
tm44.x0 = [rand(), rand()]
tm44.xy = tm44.x0

tm43 = deepcopy(tm1)
tm43.x0 = [rand(), rand()]
tm43.xy = tm43.x0

tm45 = deepcopy(tm1)
tm45.x0 = [rand(), rand()]
tm45.xy = tm45.x0

tm46 = deepcopy(tm1)
tm46.x0 = [rand(), rand()]
tm46.xy = tm46.x0

tm47 = deepcopy(tm1)
tm47.x0 = [rand(), rand()]
tm47.xy = tm47.x0

tm48 = deepcopy(tm1)
tm48.x0 = [rand(), rand()]
tm48.xy = tm48.x0

tm49 = deepcopy(tm1)
tm49.x0 = [rand(), rand()]
tm49.xy = tm49.x0

tm50 = deepcopy(tm1)
tm50.x0 = [rand(), rand()]
tm50.xy = tm50.x0

tm51 = deepcopy(tm1)
tm51.x0 = [rand(), rand()]
tm51.xy = tm51.x0

tm52 = deepcopy(tm1)
tm52.x0 = [rand(), rand()]
tm52.xy = tm52.x0

tm53 = deepcopy(tm1)
tm53.x0 = [rand(), rand()]
tm53.xy = tm53.x0

tm54 = deepcopy(tm1)
tm54.x0 = [rand(), rand()]
tm54.xy = tm54.x0

tm55 = deepcopy(tm1)
tm55.x0 = [rand(), rand()]
tm55.xy = tm55.x0

tm56 = deepcopy(tm1)
tm56.x0 = [rand(), rand()]
tm56.xy = tm56.x0

tm57 = deepcopy(tm1)
tm57.x0 = [rand(), rand()]
tm57.xy = tm57.x0

tm58 = deepcopy(tm1)
tm58.x0 = [rand(), rand()]
tm58.xy = tm58.x0

tm59 = deepcopy(tm1)
tm59.x0 = [rand(), rand()]
tm59.xy = tm59.x0

tm60 = deepcopy(tm1)
tm60.x0 = [rand(), rand()]
tm60.xy = tm60.x0

tm61 = deepcopy(tm1)
tm61.x0 = [rand(), rand()]
tm61.xy = tm61.x0

tm62 = deepcopy(tm1)
tm62.x0 = [rand(), rand()]
tm62.xy = tm62.x0

tm63 = deepcopy(tm1)
tm63.x0 = [rand(), rand()]
tm63.xy = tm63.x0

tm64 = deepcopy(tm1)
tm64.x0 = [rand(), rand()]
tm64.xy = tm64.x0

tm65 = deepcopy(tm1)
tm65.x0 = [rand(), rand()]
tm65.xy = tm65.x0

tm66 = deepcopy(tm1)
tm66.x0 = [rand(), rand()]
tm66.xy = tm66.x0

tm67 = deepcopy(tm1)
tm67.x0 = [rand(), rand()]
tm67.xy = tm67.x0

tm68 = deepcopy(tm1)
tm68.x0 = [rand(), rand()]
tm68.xy = tm68.x0

tm69 = deepcopy(tm1)
tm69.x0 = [rand(), rand()]
tm69.xy = tm69.x0

tm70 = deepcopy(tm1)
tm70.x0 = [rand(), rand()]
tm70.xy = tm70.x0





# array of trajectory managers
# vtm = [tm1, tm2, tm3, tm4, tm5, tm6, tm7, tm8, tm9, tm10, tm11, tm12, tm13, tm14, tm15, tm16, tm17,tm18, tm19, tm20,tm21,tm22, tm23,tm24, tm25, tm26, tm27, tm28, tm29, tm30,tm31, tm32, tm33, tm34, tm35, tm36, tm37, tm38, tm39, tm40, tm41,tm42, tm43, tm44, tm45, tm46,tm47,tm48, tm49, tm50,tm51,tm52,tm53, tm54,tm55,tm56,tm57,tm58,tm59,tm60,tm61,tm62,tm63, tm64, tm65,tm66,tm67,tm68,tm69,tm70]
vtm = [tm1, tm2, tm3, tm4]

# Generate the trajectories
ddc = 1e-4
tm1.barrier_cost = 1
tm2.barrier_cost = 1
 # xd, ud = smc_trajectory_s(em, vtm, 0.2)
xd, ud = smc_trajectory(em, vtm)

# plotting
plot(em, xd, vtm)

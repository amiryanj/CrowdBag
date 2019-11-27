import crowdsim

sim = crowdsim.CrowdSim("helbing")
N = 10
sim.init(N)
sim.setTime(0)

for i in range(N):
   sim.setPosition(i, 0, i)
   sim.setGoal(i, 10, i)

dt = 0.01
for k in range(100):
   t = dt * k
   sim.doStep(dt)
   for i in range(N):
      px_i = sim.getCenterxNext(i)
      py_i = sim.getCenteryNext(i)
      vx_i = sim.getCenterVelocityxNext(i)
      vy_i = sim.getCenterVelocityyNext(i)
      print(t, px_i, py_i)

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Python examples 
# https://github.com/CoppeliaRobotics/zmqRemoteApi/tree/master/clients/python 

client = RemoteAPIClient()
sim = client.require('sim')

# https://manual.coppeliarobotics.com/en/simulation.htm#stepped
sim.setStepping(True)

target = sim.getObject('/Target')

panda_joint = []
for i in range(7):
    panda_joint.append(sim.getObject(':/Franka_joint' + str(i+1)))

sim.startSimulation()
while (t := sim.getSimulationTime()) < 15:
    print(f'Simulation time: {t:.2f} [s]')
    position = sim.getObjectPosition(target,-1)
    # print(position)

    if t > 8:
        # sim.setObjectPosition(target,[0.4,0,0.4],-1)
        # sim.setObjectQuaternion(target,[-1.0,0,0.0,0.0],-1)


        sim.setJointTargetPosition(panda_joint[0], 0.0)
        sim.setJointTargetPosition(panda_joint[1], 0.0)
        sim.setJointTargetPosition(panda_joint[2], 0.0)
        sim.setJointTargetPosition(panda_joint[3], 0.0)
        sim.setJointTargetPosition(panda_joint[4], 0.0)
        sim.setJointTargetPosition(panda_joint[5], 0.0)
        sim.setJointTargetPosition(panda_joint[6], 0.0)


    position = []
    for i in range(7):
        position.append(sim.getJointPosition(panda_joint[i]))
    print(position)
    sim.step()

sim.stopSimulation()
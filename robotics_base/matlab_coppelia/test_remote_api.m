client = RemoteAPIClient();
sim = client.require('sim');

% https://github.com/CoppeliaRobotics/zmqRemoteApi/tree/master/clients/matlab
% https://manual.coppeliarobotics.com/en/simulation.htm#stepped
sim.setStepping(true)

target = sim.getObject('/Target');

j_num = 7;
panda_joint = zeros(j_num, 1);
for i=1:j_num
    panda_joint(i) = sim.getObject(strcat (':/Franka_joint', num2str(i)));
end


sim.startSimulation();
while true
    t = sim.getSimulationTime();
    if t >= 15; break; end
    fprintf('Simulation time: %.2f [s]\n', t);
    position = sim.getObjectPosition(target,-1);


    if t >= 8.0
        sim.setObjectPosition(target,[0.4,0,0.4],-1);
        sim.setObjectQuaternion(target,[-1.0,0,0.0,0.0],-1);
    end
    sim.step();
end
sim.stopSimulation();
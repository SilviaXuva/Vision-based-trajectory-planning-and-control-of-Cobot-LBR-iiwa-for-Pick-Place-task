--lua

sim=require('sim')
rad = 180/math.pi
deg = math.pi/180

function sysCall_init()
    joint = 1
    mode = 'pos'

    targetPos = 90*deg
    targetVel = 90*deg
    stopSim = 5

    -- Take a few handles from the dynamic robot:
    simJoints={}
    for i=1,7,1 do
        simJoints[i]=sim.getObject('./joint'..(i-1))
        sim.setJointTargetPosition(simJoints[i],0)
    end

    for i=1,7,1 do
        if mode == 'pos' then
            sim.setObjectInt32Param(simJoints[i], sim.jointintparam_dynctrlmode, sim.jointdynctrl_position)
        elseif mode == 'vel' then 
            sim.setObjectInt32Param(simJoints[i], sim.jointintparam_dynctrlmode, sim.jointdynctrl_velocity)
        end
    end

    -- Graphs:
    -- pos
    graphPosition=sim.getObject('/position')
    sim.destroyGraphCurve(graphPosition,-1)
    pos = sim.addGraphStream(graphPosition,'joint '..joint..' position','deg',0,{1,0,0})
    sim.setGraphStreamValue(graphPosition,pos,sim.getJointPosition(simJoints[joint]))
    -- vel
    graphVelocity=sim.getObject('/velocity')
    sim.destroyGraphCurve(graphVelocity,-1)
    vel = sim.addGraphStream(graphVelocity,'joint '..joint..' velocity','deg',0,{0,1,0})
    sim.setGraphStreamValue(graphVelocity,vel,sim.getJointVelocity(simJoints[joint]))
end

function sysCall_sensing()
    if sim.getSimulationTime() >= stopSim then
        sim.stopSimulation()
    end
    -- Populate graphs with points:
    sim.setGraphStreamValue(graphPosition,pos,sim.getJointPosition(simJoints[joint])) --pos
    sim.setGraphStreamValue(graphVelocity,vel,sim.getJointVelocity(simJoints[joint])) --vel
end

function sysCall_thread()
    sim.setJointTargetPosition(simJoints[joint], targetPos)
    if mode == 'pos' then
        sim.setJointTargetPosition(simJoints[joint], targetPos)
    elseif mode == 'vel' then 
        sim.setJointTargetVelocity(simJoints[joint], targetVel)
    end
end

function sysCall_cleanup()

end

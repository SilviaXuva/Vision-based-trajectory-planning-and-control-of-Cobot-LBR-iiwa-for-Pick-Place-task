-- See the end of the script for instructions on how to do efficient grasping

function sysCall_init() 
    j1=sim.getObject('./active1')
    j2=sim.getObject('./active2')

    modelHandle=sim.getObject('.')
    closing=false
    
    -- Prepare an 2 ik groups, using the convenience function 'simIK.addElementFromScene':
    ikEnv=simIK.createEnvironment()
    ikGroup1=simIK.createGroup(ikEnv)
    local simBase=sim.getObject('.')
    local simTip1=sim.getObject('./LclosureDummyA')
    local simTarget1=sim.getObject('./LclosureDummyB')
    simIK.addElementFromScene(ikEnv,ikGroup1,simBase,simTip1,simTarget1,simIK.constraint_x+simIK.constraint_z)
    ikGroup2=simIK.createGroup(ikEnv)
    local simTip2=sim.getObject('./RclosureDummyA')
    local simTarget2=sim.getObject('./RclosureDummyB')
    simIK.addElementFromScene(ikEnv,ikGroup2,simBase,simTip2,simTarget2,simIK.constraint_x+simIK.constraint_z)
    
end

function sysCall_cleanup() 
    simIK.eraseEnvironment(ikEnv)
end 

function sysCall_sensing()
    local s=sim.getObjectSel()
    local show=(s and #s==1 and s[1]==modelHandle)
    if show then
        if not ui then
            local xml =[[<ui title="xxxx" closeable="false" placement="relative" layout="form">
                    <button id="1" text="open" checkable="true" checked="true" auto-exclusive="true" on-click="openClicked"/>
                    <button id="2" text="close" checkable="true" auto-exclusive="true" on-click="closeClicked"/>
            </ui>]]
            ui=simUI.create(xml)
            if uiPos then
                simUI.setPosition(ui,uiPos[1],uiPos[2])
            else
                uiPos={}
                uiPos[1],uiPos[2]=simUI.getPosition(ui)
            end
            simUI.setTitle(ui,sim.getObjectAlias(modelHandle,1))
            simUI.setButtonPressed(ui,1,not closing)
            simUI.setButtonPressed(ui,2,closing)
        end
    else
        if ui then
            uiPos[1],uiPos[2]=simUI.getPosition(ui)
            simUI.destroy(ui)
            ui=nil
        end
    end
end

function openClicked(ui,id)
    closing=false
end

function closeClicked(ui,id)
    closing=true
end

function Actuation(close)
    closing = close
    sysCall_actuation()
end

function sysCall_actuation() 
    p1=sim.getJointPosition(j1)
    p2=sim.getJointPosition(j2)
    
    if (closing) then
        if (p1<p2-0.008) then
            sim.setJointTargetVelocity(j1,-0.01)
            sim.setJointTargetVelocity(j2,-0.04)
        else
            sim.setJointTargetVelocity(j1,-0.04)
            sim.setJointTargetVelocity(j2,-0.04)
        end
    else
        if (p1<p2) then
            sim.setJointTargetVelocity(j1,0.04)
            sim.setJointTargetVelocity(j2,0.02)
        else
            sim.setJointTargetVelocity(j1,0.02)
            sim.setJointTargetVelocity(j2,0.04)
        end
    --    sim.setJointTargetVelocity(j1,0.04)
    --    sim.setJointTargetVelocity(j2,0.04)
    end
    
    simIK.applyIkEnvironmentToScene(ikEnv,ikGroup1)
    simIK.applyIkEnvironmentToScene(ikEnv,ikGroup2)
end
    
-- You have basically 2 alternatives to grasp an object:
--
-- 1. You try to grasp it in a realistic way. This is quite delicate and sometimes requires
--    to carefully adjust several parameters (e.g. motor forces/torques/velocities, friction
--    coefficients, object masses and inertias)
--
-- 2. You fake the grasping by attaching the object to the gripper via a connector. This is
--    much easier and offers very stable results.
--
-- Alternative 2 is explained hereafter:
--
--
-- a) In the initialization phase, retrieve some handles:
-- 
-- connector=sim.getObject('./attachPoint')
-- objectSensor=sim.getObject('./attachProxSensor')

-- b) Before closing the gripper, check which dynamically non-static and respondable object is
--    in-between the fingers. Then attach the object to the gripper:
--
-- index=0
-- while true do
--     shape=sim.getObjects(index,sim.object_shape_type)
--     if (shape==-1) then
--         break
--     end
--     if (sim.getObjectInt32Param(shape,sim.shapeintparam_static)==0) and (sim.getObjectInt32Param(shape,sim.shapeintparam_respondable)~=0) and (sim.checkProximitySensor(objectSensor,shape)==1) then
--         -- Ok, we found a non-static respondable shape that was detected
--         attachedShape=shape
--         -- Do the connection:
--         sim.setObjectParent(attachedShape,connector,true)
--         break
--     end
--     index=index+1
-- end

-- c) And just before opening the gripper again, detach the previously attached shape:
--
-- sim.setObjectParent(attachedShape,-1,true)

function sysCall_init()
    sim.addLog(sim.verbosity_scriptinfos,'Computing reachable points. This may take a few seconds...')
    a=0
end 

doCalculation=function()
    local divisions=5 -- the divisions used for each joint. The total nb of pts becomes (divisions+1)^7
    local usePointCloud=true -- if false, 3D spheres will be used
    local extractConvexHull=true
    local generateRandom=true -- Use random configurations instead of joint divisions
    local randomSteps=46000 -- Number of random configs to test


    local jointHandles={}
    local jointLimitLows={}
    local jointLimitRanges={}
    local initialJointPositions={}
    for i=1,7,1 do
        jointHandles[i]=sim.getObject('./joint'..i-1)
        local cyclic,interv=sim.getJointInterval(jointHandles[i])
        if cyclic then
            jointLimitLows[i]=-180*math.pi/180
            jointLimitRanges[i]=360*math.pi/180
        else
            jointLimitLows[i]=interv[1]
            jointLimitRanges[i]=interv[2]
        end
        initialJointPositions[i]=sim.getJointPosition(jointHandles[i])
    end
    local tip=sim.getObject('./tip')
    local base=sim.getObject('.')
    local robotCollection=sim.createCollection(0)
    sim.addItemToCollection(robotCollection,sim.handle_tree,base,0)
    local points={}

    if not usePointCloud then
        pointContainer=sim.addDrawingObject(sim.drawing_spherepts,0.01,0.01,base,99999,{1,1,1})
    end
    
    if generateRandom then
        for cnt=1,randomSteps,1 do
            local p1=jointLimitLows[1]+math.random()*jointLimitRanges[1]
            sim.setJointPosition(jointHandles[1],p1)
            local p2=jointLimitLows[2]+math.random()*jointLimitRanges[2]
            sim.setJointPosition(jointHandles[2],p2)
            local p3=jointLimitLows[3]+math.random()*jointLimitRanges[3]
            sim.setJointPosition(jointHandles[3],p3)
            local p4=jointLimitLows[4]+math.random()*jointLimitRanges[4]
            sim.setJointPosition(jointHandles[4],p4)
            local p5=jointLimitLows[5]+math.random()*jointLimitRanges[5]
            sim.setJointPosition(jointHandles[5],p5)
            local p6=jointLimitLows[6]+math.random()*jointLimitRanges[6]
            sim.setJointPosition(jointHandles[6],p6)
            local p7=jointLimitLows[7]+math.random()*jointLimitRanges[7]
            sim.setJointPosition(jointHandles[7],p7)

            local colliding=false
            local matrix=sim.getObjectMatrix(tip,sim.handle_world)
            -- local pos=sim.getObjectPosition(tip,sim.handle_world)
            if not colliding then
                points[#points+1]=matrix[4]
                points[#points+1]=matrix[8]
                points[#points+1]=matrix[12]
                if not usePointCloud then
                    sim.addDrawingObjectItem(pointContainer,{matrix[4],matrix[8],matrix[12]})
                end
            end
        end
    else
        for i1=1,divisions+1,1 do
            local p1=jointLimitLows[1]+(i1-1)*jointLimitRanges[1]/divisions
            sim.setJointPosition(jointHandles[1],p1)
            for i2=1,divisions+1,1 do
                local p2=jointLimitLows[2]+(i2-1)*jointLimitRanges[2]/divisions
                sim.setJointPosition(jointHandles[2],p2)
                for i3=1,divisions+1,1 do
                    local p3=jointLimitLows[3]+(i3-1)*jointLimitRanges[3]/divisions
                    sim.setJointPosition(jointHandles[3],p3)
                    for i4=1,divisions+1,1 do
                        local p4=jointLimitLows[4]+(i4-1)*jointLimitRanges[4]/divisions
                        sim.setJointPosition(jointHandles[4],p4)
                        for i5=1,divisions+1,1 do
                            local p5=jointLimitLows[5]+(i5-1)*jointLimitRanges[5]/divisions
                            sim.setJointPosition(jointHandles[5],p5)
                            for i6=1,divisions+1,1 do
                                local p6=jointLimitLows[6]+(i6-1)*jointLimitRanges[6]/divisions
                                sim.setJointPosition(jointHandles[6],p6)
                                for i7=1,divisions+1,1 do
                                    local p7=jointLimitLows[7]+(i7-1)*jointLimitRanges[7]/divisions
                                    sim.setJointPosition(jointHandles[7],p7)
                                
                                    local colliding=false
                                    local matrix=sim.getObjectMatrix(tip,sim.handle_world)
                                    -- local pos=sim.getObjectPosition(tip,sim.handle_world)
                                    if checkCollision then
                                        if sim.checkCollision(robotCollection,sim.handle_all)~=0 then
                                            colliding=true
                                        end
                                    end
                                    if not colliding then
                                        points[#points+1]=matrix[4]
                                        points[#points+1]=matrix[8]
                                        points[#points+1]=matrix[12]
                                        if not usePointCloud then
                                            sim.addDrawingObjectItem(pointContainer,{matrix[4],matrix[8],matrix[12]})
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end


    for i=1,7,1 do
        sim.setJointPosition(jointHandles[i],initialJointPositions[i])
    end

    if usePointCloud then
        local ptcld=sim.createPointCloud(0.05,1,0,2)
        sim.insertPointsIntoPointCloud(ptcld,0,points)
    end

    if extractConvexHull then
        local vertices,indices=sim.getQHull(points)
        local shape=sim.createShape(3,0,vertices,indices)
        sim.alignShapeBB(shape,{0,0,0,0,0,0,1})
        sim.setShapeColor(shape,nil,0,{1,0,1})
        sim.setShapeColor(shape,nil,4,{0.2})
    end
end



function sysCall_actuation()
    a=a+1
    if a==2 then
        doCalculation()
    end
end

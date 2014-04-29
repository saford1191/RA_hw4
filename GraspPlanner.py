import logging, numpy, openravepy, math

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner

        self.isFirstAttempt = True

            
    def GetBasePoseForObjectGrasp(self, obj):

        base_pose = None
        grasp_config = None
       
        ###################################################################
        # TODO: Here you will fill in the function to compute
        #  a base pose and associated grasp config for the 
        #  grasping the bottle
        ###################################################################        

        if self.isFirstAttempt:
            # Load grasp database
            self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
            if not self.gmodel.load():
                self.gmodel.autogenerate()

            # Order grasps
            self.grasps = self.gmodel.grasps
            self.graspindices = self.gmodel.graspindices
            self.order_grasps()

            # Load Inverse reachability model
            self.irmodel = openravepy.databases.inversereachability.InverseReachabilityModel(self.robot)
            print "Loading Inverse Reachability model..."
            if not self.irmodel.load():
                self.irmodel.autogenerate()

            self.isFirstAttempt = False

        # Record initial pose of herb
        start_transform = self.robot.GetTransform()
        start_joints    = self.robot.GetActiveDOFValues()
        
        manip   = self.robot.GetActiveManipulator()
        env     = self.robot.GetEnv()

        # Iterate through the sorted grasps and find out the base and arm configuration to get to the grasp
        baseFound = False
        for grasp in self.grasps_ordered:
            # Location and orientation of the hand for current grasp in global co-ordinates
            T_grasp = self.gmodel.getGlobalGraspTransform(grasp, collisionfree = True)
            if T_grasp == None:
                continue
            
            # Come up with 10 randmly sampled base poses that can reach towards given grasp
            desnityfn,samplerfn,bounds = self.irmodel.computeBaseDistribution(T_grasp)
            pose, jointstate = samplerfn(10)
            
            # Iterate through sampled base positions to see if they are collision free 
            #    and if they can genrate collision free arm configuratinos to get to the grasp
            for i in range(10):
                # Snap the transform to current the nearest grid location and test IK and collision
                self.robot.SetTransform(pose[i])
                temp_transform      = self.robot.GetTransform()
                temp_transform_SE2  = [temp_transform[0][3], temp_transform[1][3], numpy.arctan2(temp_transform[1][0], temp_transform[0][0])]
                temp_grid_coord     = self.base_planner.planning_env.discrete_env.ConfigurationToGridCoord(temp_transform_SE2)
                discrete_transform  = self.base_planner.planning_env.discrete_env.GridCoordToConfiguration(temp_grid_coord)
                x       = discrete_transform[0]
                y       = discrete_transform[1]
                theta   = discrete_transform[2]
                robot_pose = numpy.array([[numpy.cos(theta), -numpy.sin(theta), 0,  x],
                                          [numpy.sin(theta),  numpy.cos(theta), 0,  y],
                                          [0.              ,  0.              , 1,  0.  ],
                                          [0.              ,  0.              , 0,  1.  ]])
                self.robot.SetTransform(robot_pose)

                jointAngles = manip.FindIKSolution(T_grasp, filteroptions=1)  # Second parameter reference - http://tinyurl.com/k88yqob
                if jointAngles == None:
                    continue
                self.robot.SetActiveDOFValues(jointAngles)

                # Check for collision and break if collision-free
                inCollision = env.CheckCollision(env.GetBodies()[0], env.GetBodies()[1]) or self.robot.CheckSelfCollision()
                if not inCollision:
                    baseFound = True
                    break
            
            if baseFound:
                break

        if not baseFound:       # Can't find a collision-free target
            # Return herb to its initial position
            self.robot.SetActiveDOFValues(start_joints)
            self.robot.SetTransform(start_transform)

            return base_pose, grasp_config
        
        else:                   # Return the base position and correspondign grasp configuation for grasping
            # Come up with target base configuration in terms of [x, y, orientation]
            goal_transform = self.robot.GetTransform()
            base_pose = [goal_transform[0][3], goal_transform[1][3], numpy.arctan2(goal_transform[1][0], goal_transform[0][0])]
            base_pose = numpy.array(base_pose)

            # Manipulator configurations corresponding to the base configuration
            grasp_config = jointAngles

            # Return herb to its initial pose
            self.robot.SetActiveDOFValues(start_joints)
            self.robot.SetTransform(start_transform)

            print "\nGoals found..."
            print "Target base configuration: " + str(base_pose)
            print "Target manipulator configuration: " + str(grasp_config) + "\n"

            return base_pose, grasp_config

        #######################################
        # grasp_transform = self.gmodel.getGlobalGraspTransform(grasp, collisionfree = True)
        # desnityfn,samplerfn,bounds = self.irmodel.computeBaseDistribution(grasp_transform)
        # pose, joinstate = samplerfn(1)
        # self.robot.SetTransform(pose[0])
        # self.robot.GetTransform()
        # ik=manip.FindIKSolution(grasp_transform,0 )     # Check second parameter. Something to do with collision
        # self.robot.SetActiveDOFValues(ik)
        #######################################
        # for grasp in self.grasps_ordered:
        #     contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=True)
        #     self.robot.SetTransform(finalconfig[1])
        #     T_grasp = manip.GetTransform()
        #     self.robot.SetTransform(start_transform)
        #     desnityfn,samplerfn,bounds = self.irmodel.computeBaseDistribution(T_grasp)
        #######################################        
        # import IPython
        # IPython.embed()
        # return base_pose, grasp_config
        #######################################

    def order_grasps(self):
        self.grasps_ordered = self.grasps.copy()

        graspMatrixMinVal = []
        graspMatrixRatio = []
        graspMatrixVolume = []

        print "Evaluating grasps..."

        # Find parameters for comparison of each grasp
        for grasp in self.grasps_ordered:
            #sys.stdout.write("\n%2d : " %(i)
            temp = self.eval_grasp(grasp)
            graspMatrixMinVal.append(temp[0])
            graspMatrixRatio.append(temp[1])
            graspMatrixVolume.append(temp[2])

        # Rescale the minimum eigen values to [0,1]
        minMin = min(graspMatrixMinVal)
        maxMin = max(graspMatrixMinVal)
        rangeMin = maxMin - minMin
        graspMatrixMinValNorm = [ (e-minMin)/(rangeMin) for e in graspMatrixMinVal ]

        # Rescale the voulme to [0,1]
        minVol = min(graspMatrixVolume)
        maxVol = max(graspMatrixVolume)
        rangeVol = maxVol - minVol
        graspMatrixVolumeNorm = [ (e-minVol)/(rangeVol) for e in graspMatrixVolume ]

        # Add up the performance martices to get the final matrix for comparison
        i = 0
        for grasp in self.grasps_ordered:  
            grasp[self.graspindices.get('performance')] = graspMatrixVolumeNorm[i] + graspMatrixMinValNorm[i] + graspMatrixRatio[i]
            i+=1

        # sort!
        order = numpy.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
        order = order[::-1]
        print "Grasps ordered..."
        # print "Graps ordered. Order of original indices: "
        # print order
        self.grasps_ordered = self.grasps_ordered[order]

    def eval_grasp(self, grasp):
        with self.robot:
            #contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
            try:
                contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)

                obj_position = self.gmodel.target.GetTransform()[0:3,3]
                # for each contact
                G = numpy.array([]) #the wrench matrix
                for c in contacts:
                    pos = c[0:3] - obj_position
                    drc = -c[3:] #this is already a unit vector
                    # Fill G
                    drc=numpy.array(drc)
                    W=numpy.array([drc,numpy.cross(pos,drc)])
                    G=numpy.append(G,W)

                n=len(G)
                G=G.reshape(n/6,6)
                G=G.transpose()

                # Use G to compute scrores as discussed in class
                try:
                    s,v,d=numpy.linalg.svd(G)
                    min_value=min(v)
                    max_value=max(v)
                    ratio=min_value/max_value
                    volume = math.sqrt(abs(numpy.linalg.det(numpy.dot(G,numpy.transpose(G)))))
                    answerL = (min_value, ratio, volume)
                    return answerL
                
                except numpy.linalg.linalg.LinAlgError:
                    # print "\n\nThis G caused an error: "
                    # print G
                    return (0.0, 0.0, 0.0)

            except openravepy.planning_error,e:
                #you get here if there is a failure in planning
                #example: if the hand is already intersecting the object at the initial position/orientation
                return  (0.00, 0.00, 0.00) 

    def PlanToGrasp(self, obj):

        pipelineComplete    = False
        global_start_base   = self.robot.GetTransform()
        global_start_arm    = self.robot.GetActiveDOFValues()

        print global_start_arm

        numIter = 0

        while not pipelineComplete:
            # First select a pose for the base and an associated ik for the arm
            print '[Grasp Planner] Find base position and arm configuration for a grasp'
            base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

            if base_pose is None or grasp_config is None:
                print 'Failed to find solution'
                exit()

            # Now, get the arm to a position which will make the job of RRT easier later on...
            print '\n\n\n[Grasp Planner] Move the arm to initial position to faciliate later plannign stages'
            if self.robot.GetActiveManipulator() == self.robot.GetManipulator('left_wam'):
                good_arm_pose = numpy.array([numpy.pi,  0.0 , -numpy.pi/2,  3*numpy.pi/4,  0.0 , 0.0,  0.0])
            else:
                good_arm_pose = numpy.array([numpy.pi,  0.0 ,  numpy.pi/2,  3*numpy.pi/4,  0.0 , 0.0,  0.0])

            start_config_0 = self.arm_planner.planning_env.herb.GetCurrentConfiguration()
            arm_plan_0 = self.arm_planner.Plan(start_config_0, good_arm_pose)
            if arm_plan_0 == []:
                self.robot.SetTransform(global_start_base)
                self.robot.SetActiveDOFValues(global_start_arm)
                numIter += 1
                if numIter < 10:
                    continue
                else:
                    break

            arm_traj_0 = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan_0)

            print '[Grasp Planner] Executing arm trajectory'
            self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj_0)

            # Now plan to the base pose
            # print '\n\n\n[Grasp Planner] Move the robot base to the desired location'
            # start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
            # base_plan = self.base_planner.Plan(start_pose, base_pose)
            # if base_plan == []:
            #     self.robot.SetTransform(global_start_base)
            #     self.robot.SetActiveDOFValues(global_start_arm)
            #     numIter += 1
            #     if numIter < 10:
            #         continue
            #     else:
            #         break
            # base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

            # print 'Executing base trajectory'
            # self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

            print '\n\n\n[Grasp Planner] Teleporting to the final base position'
            x = base_pose[0]
            y = base_pose[1]
            theta = base_pose[2]
            robot_pose = numpy.array([[numpy.cos(theta), -numpy.sin(theta), 0,  x],
                                      [numpy.sin(theta),  numpy.cos(theta), 0,  y],
                                      [0.              ,  0.              , 1,  0.  ],
                                      [0.              ,  0.              , 0,  1.  ]])
            self.robot.SetTransform(robot_pose)

            # Now plan the arm to the grasp configuration
            print '\n\n\n[Grasp Planner] Planning for final arm path'
            start_config = self.arm_planner.planning_env.herb.GetCurrentConfiguration()
            arm_plan = self.arm_planner.Plan(start_config, grasp_config)
            print arm_plan
            if arm_plan == []:
                self.robot.SetTransform(global_start_base)
                self.robot.SetActiveDOFValues(global_start_arm)
                numIter += 1
                if numIter < 10:
                    continue
                else:
                    break
            arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

            print 'Executing arm trajectory'
            self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

            pipelineComplete = True

        if pipelineComplete:
            # Grasp the bottle
            self.robot.SetTransform(global_start_base)
            self.robot.SetActiveDOFValues(global_start_arm)
            print global_start_arm
            print global_start_base

            raw_input('Press Any Key to execute entire sequence of actions...')
            self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj_0)

            raw_input('Press any key to continue to next step')
            # self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)
            self.robot.SetTransform(robot_pose)

            raw_input('Press any key to continue to next step')
            self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

            task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
            task_manipulation.CloseFingers()
        else:
            print "\n\n\n[Grasp Planner] Can not find complete solution after 10 attempts... Exiting..."
    

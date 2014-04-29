import numpy
import time

class HerbEnvironment(object):

    def __init__(self, herb):

        self.herb = herb
        self.robot = herb.robot
        self.env = self.robot.GetEnv()
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
 
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)

    def hRRT_SetGoalParameters(self, goal_config):
        self.hrrt_goal_config = goal_config

    def hRRT_GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())

        # Get limits for all active degrees of freedom
        minLimits, maxLimits = self.robot.GetActiveDOFLimits()
        # Set environment variable. Will be used for collision check
        env = self.robot.GetEnv()

        # Generate a random and collision free configuration
        inCollision = True
        with env:
            while inCollision == True:
                # Generate a random configuration
                for i in range(len(config)):
                    config[i] = (maxLimits[i] - minLimits[i]) * numpy.random.random_sample() + minLimits[i]
                # Check if the configuration that was generated is in collision
                self.robot.SetActiveDOFValues(numpy.array(config))
                inCollision = env.CheckCollision(env.GetBodies()[0],env.GetBodies()[1]) or self.robot.CheckSelfCollision()

        return numpy.array(config)

    def hRRT_ComputeDistance(self, start_config, end_config):

        # L2 distance between two configurations
        return numpy.linalg.norm(start_config-end_config)

    def hRRT_ComputeCostToGo(self, config):

        return self.hRRT_ComputeDistance(config, self.hrrt_goal_config)

    def hRRT_Extend(self, start_config, end_config):

        # Find out minimum number of steps required for linearly extending each active DOF
        #   based on start configuration, end configuration and resolution of the DOFs
        minNumSteps = []
        for i in range(len(start_config)):
            minNumSteps.append(abs(start_config[i]-end_config[i])/self.robot.GetActiveDOFResolutions()[i])
        # Select the maximum among the minimum number of steps
        numSteps = int(numpy.ceil(max(minNumSteps)))

        # Compute trajectory based on linear interpolation
        trajectorySteps = []
        for i in range(len(start_config)):
            curStepSize = (end_config[i] - start_config[i]) / numSteps
            trajectorySteps.append(numpy.arange(start_config[i], end_config[i], curStepSize))

        # Environment variable for collision check
        env = self.robot.GetEnv()

        # If the entire path is collision free, end configuration will be returned
        returnConfig = end_config
        # Test all the intermediate steps in the trajectory for collision
        with env:
            for i in range(numSteps):
                # Get current configuration for testing
                testConfig = []
                for j in range(len(start_config)):
                    testConfig.append(trajectorySteps[j][i])
                
                # Set the ARM to the current configuration
                self.robot.SetActiveDOFValues(numpy.array(testConfig))
                
                # Check for collision. If there is collision, set the previous configuration as last collision free config
                inCollision = env.CheckCollision(env.GetBodies()[0],env.GetBodies()[1]) or env.CheckCollision(env.GetBodies()[0], env.GetBodies()[2]) or self.robot.CheckSelfCollision()
                if inCollision == True:
                    testConfig = []
                    for j in range(len(start_config)):
                        testConfig.append(trajectorySteps[j][i-1])
                    returnConfig = numpy.array(testConfig)
                    break

        if inCollision:
            return []
        else:
            return returnConfig

    def hRRT_ShortenPath(self, path, timeout=5.0):
        
        print "Beginning path shortening..."
        # Store the goal configuration 
        goal_config = path[-1]
        # Initialize the time
        starttime = time.clock()
        # Step size
        s = 2
        
        # Run the path shortening for arbitrarily large number of times
        for k in range(500):
            # Lower limit on number of nodes in path. Do not shorten if 4 or less nodes in the path
            if len(path) < 4:
                break
            
            # Initialize new path with starting configuration
            i = 0
            newpath = []
            newpath.append(path[0])
            
            # Repeat for the entire path. This will be executed completely only if the path can not be shortened any more. Will be broken otherwise  
            for j in range(len(path)-s):
                # Try to extend from current configuration to the one after step size
                extension = self.hRRT_Extend(path[i], path[i+s])
                
                # Check if the path was obstacle free. Path is obstacle free if and only if complete extension was possible                 
                # If there is no obstacle, add the configuration after step size to new path.
                if extension != []:
                    newpath.append(path[i+s])
                    i = i+s
                # Otherwise, add the immediate next configuration to the path
                else:
                    newpath.append(path[i+1])
                    i = i+1
                
                # If the next step goes beyond the available configurations in the path, break the loop after adding goal configuration if necessary
                if i > len(path)-(s+1):
                    # Check if last element of new path is goal configuration
                    # If last element is not goal configuration, append it at the end of new path
                    if not (newpath[-1] == goal_config).all():
                        newpath.append(goal_config)
                    
                    # New path is complete. Break out of the loop
                    break
            
            # Stop path shortening if there is no improvement
            if len(newpath) == len(path):
                break
            
            # Copy the new path into the path variable             
            path = []
            for r in range(len(newpath)):
                path.append(newpath[r])
                
            # Stop path shortening if time exceeds.
            currenttime = time.clock() - starttime
            if currenttime > timeout:
                break
        
        print "Path shortening completed..."
        
        # Compute the distance of shortened path
        path_length = 0
        for j in range(len(path)-1):
            dist = self.hRRT_ComputeDistance(path[j], path[j+1])
            path_length = path_length + dist
        print "Shortened Path Length = ", round(path_length, 2)
        
        # Return shortened path
        return path

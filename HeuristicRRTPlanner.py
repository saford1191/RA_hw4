import numpy
from RRTTree import RRTTree
import time

class HeuristicRRTPlanner(object):

    def __init__(self, planning_env):
        self.planning_env = planning_env        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        self.planning_env.hRRT_SetGoalParameters(goal_config)

        pGoal = 0.2
        minNodeQuality = 0.3

        print "Staring Heuristic RRT Planner"
        start_time = time.clock()

        tree = RRTTree(self.planning_env, start_config)
        plan = []

        # if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
        #     self.planning_env.InitializePlot(goal_config)
        #     showPath = True
        # else:
        #     showPath = False

        goalFound = False
        maxIterations = 1000
        numNodes = 0

        for i in range(maxIterations):
            # print "\nIteration " + str(i)

            goalSampler = numpy.random.random()

            if goalSampler < pGoal:
                new_config = goal_config
                # print "Goal Sampled."
                # nearest_vertex = tree.GetNearestVertex(new_config)
                # print "Goal Sampled. " + "Nearest vertex: " + str(nearest_vertex[1]) 
            else:
                new_config = self.planning_env.hRRT_GenerateRandomConfiguration()

            if numNodes < 10:       # Since there are not too many members in tree, can't get meaningful k-NN
                nearest_vertex = tree.GetNearestVertex(new_config)

                nodeQuality = tree.getNodeQuality(nearest_vertex[0])
                # print "New random sample: " + str(new_config) + "\tNearest vertex: " + str(nearest_vertex[1]) + "\tNode quality: " + str(nodeQuality)
                
                nearest_config = nearest_vertex[1]
                nearest_vertex_id = nearest_vertex[0]
                extended_config = self.planning_env.hRRT_Extend(nearest_config, new_config)
            
            else:                   # Get k nearest neighbours. Acts as jailbreaker in a case where hRRT always gets 
                                    #  a nearest neigbour for the goal which can not be extended towards the goal.
                k_nearest_vertices = tree.GetkNearestVertices(new_config)

                to_delete = []
                for v in range(len(k_nearest_vertices)):
                    nodeQuality = tree.getNodeQuality(k_nearest_vertices[v][0])
                    # print "New random sample: " + str(new_config) + "\tNearest vertex: " + str(k_nearest_vertices[v][1]) + "\tNode quality: " + str(nodeQuality)

                    if nodeQuality < minNodeQuality:
                        nodeQuality = minNodeQuality

                    nodeRejector = numpy.random.random()

                    if nodeQuality < nodeRejector:
                        # print "Node Rejected"
                        to_delete.append(v)
                        
                if to_delete != []:
                    to_delete = to_delete[::-1]
                    if to_delete[-1] == 0 or len(to_delete) > 2:        # Gets rid of bad nodes.
                        continue
                    for v in to_delete:
                        del k_nearest_vertices[v]

                if k_nearest_vertices == []:
                    continue

                for v in k_nearest_vertices:
                    nearest_config = v[1]
                    nearest_vertex_id = v[0]
                    # print str(nearest_config) + str(nearest_vertex_id)
                    extended_config = self.planning_env.hRRT_Extend(nearest_config, new_config)
                    if extended_config != []:
                        break
            
            if extended_config == []:
                # print "Path not extended."
                continue

            numNodes = numNodes + 1

            # print "New node to be added: " + str(extended_config)
            tree.AddEdge(nearest_vertex_id, tree.AddVertex(extended_config))

            # if showPath:
            #     self.planning_env.PlotEdge(nearest_config, extended_config)

            testGoal = (extended_config == goal_config).all()
            
            if testGoal == True:
                goalFound = True
                break

        if goalFound:
            print "Goal found.\n\tNumber of vertices: %d\n\tTime Elapsed: %0.3f seconds" %(numNodes, time.clock()-start_time)
            
            x = len(tree.edges.items())
            while x != 0:
                plan.append(tree.vertices[x])
                x = tree.edges[x]
            plan.append(start_config)
            plan = plan[::-1]
            plan.append(goal_config)

            path_length = tree.getPathLength()
            print "\tPath Length: %0.3f" %(path_length)
        
        else:
            print "Goal not found.\n\tVertices Explored: %d\n\tTime Elapsed: %0.3f seconds"%(numNodes,time.clock()-start_time)

            plan.append(start_config)
            plan.append(goal_config)
        
        return plan
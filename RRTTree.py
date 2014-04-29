import operator
import numpy

class RRTTree(object):
    
    def __init__(self, planning_env, start_config):
        
        self.planning_env = planning_env
        self.vertices = []

        self.edges = dict()
        self.cost_to_come = dict()
        self.heuristic_cost_to_go = dict()
        self.total_node_cost = dict()
        
        self.vertices.append(start_config)
        self.cost_to_come[0] = 0
        self.heuristic_cost_to_go[0] = self.planning_env.hRRT_ComputeCostToGo(start_config)
        self.total_node_cost[0] = 0.001 + self.heuristic_cost_to_go[0] # To ensure that we do not get a nan

        # print "%.4f %.4f %.4f %.4f" %(0,self.cost_to_come[0],self.heuristic_cost_to_go[0],self.total_node_cost[0])

    def GetRootId(self):
        return 0

    def GetNearestVertex(self, config):
        
        dists = []
        for v in self.vertices:
            dists.append(self.planning_env.hRRT_ComputeDistance(config, v))

        vid, vdist = min(enumerate(dists), key=operator.itemgetter(1))

        return vid, self.vertices[vid]
            
    def GetkNearestVertices(self, config):

        dists = []
        for v in self.vertices:
            dists.append(self.planning_env.hRRT_ComputeDistance(config, v))

        kNearestVertices = []
        
        maxval = 1 + max(dists)
        for i in range(5):
            vid, vdist = min(enumerate(dists), key=operator.itemgetter(1))
            kNearestVertices.append((vid, self.vertices[vid]))
            dists[vid] = maxval

        return kNearestVertices

    def AddVertex(self, config):
        vid = len(self.vertices)
        self.vertices.append(config)
        return vid

    def AddEdge(self, sid, eid):
        self.edges[eid] = sid
        
        edge_cost = self.planning_env.hRRT_ComputeDistance(self.vertices[sid], self.vertices[eid])
        self.cost_to_come[eid] = self.cost_to_come[sid] + edge_cost
        self.heuristic_cost_to_go[eid] = self.planning_env.hRRT_ComputeCostToGo(self.vertices[eid])
        self.total_node_cost[eid] = self.cost_to_come[eid] + self.heuristic_cost_to_go[eid]

        # print "\tEdge: %.4f  c2come: %.4f  c2go: %.4f  cost: %.4f" %(edge_cost,self.cost_to_come[eid],self.heuristic_cost_to_go[eid],self.total_node_cost[eid])

    def getNodeQuality(self, node_id):
        cOpt = self.heuristic_cost_to_go[0]
        cMax = max(self.total_node_cost.values())
        cQ = self.total_node_cost[node_id]

        nodeQuality = 1 - ( (cQ - cOpt) / (cMax - cOpt) )

        return nodeQuality

    def getPathLength(self):
        return self.cost_to_come[len(self.vertices)-1]
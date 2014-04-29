import numpy
import math

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        self.vol_dimension = self.dimension*[0]
        prev_volume = 1
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx])/resolution[idx])
            self.vol_dimension[idx] = self.num_cells[idx]*prev_volume
            prev_volume = self.vol_dimension[idx]
        self.vol_dimension = [1]+self.vol_dimension

    def ConfigurationToNodeId(self, config):
        
        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
        node_id = self.GridCoordToNodeId(self.ConfigurationToGridCoord(config))

        return int(node_id)

    def NodeIdToConfiguration(self, nid):
        
        # TODO:
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        config = self.GridCoordToConfiguration(self.NodeIdToGridCoord(nid))
        return numpy.array(config)
        
    def ConfigurationToGridCoord(self, config):
        
        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        coord = [0] * self.dimension
        for idx in range(self.dimension):
            coord [idx]= math.floor(((config[idx] - self.lower_limits[idx]) / self.resolution[idx]))
        return numpy.array(coord)

    def GridCoordToConfiguration(self, coord):
        
        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [0] * self.dimension
        res_middle = 0.0

        for idx in range(self.dimension):
            # res_middle = self.resolution[idx]/2
            # if idx == 2:            # Don't snap the angle to the middle of two resoultions
            #     res_middle = 0      # Because it is more intuitive to visualize 0,45,90 as compared to -22.5,22.5,67.5 etc
            config[idx] = coord[idx] * self.resolution[idx] + res_middle + self.lower_limits[idx]
        
        return numpy.array(config)

    def GridCoordToNodeId(self,coord):
        node_id = 0
        
        for idx in range(self.dimension):
            node_id = node_id + coord[idx] * self.vol_dimension[idx]
        
        return int(node_id)

    def NodeIdToGridCoord(self, node_id):
        # TODO:
        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension
        for idx in reversed(range(self.dimension)):
            #coord[idx] = math.floor(node_id / self.vol_dimension[idx])
            coord[idx] = node_id // self.vol_dimension[idx]     # Equivalent to divide and floor
            node_id = node_id % self.vol_dimension[idx]
        
        return numpy.array(coord)
        
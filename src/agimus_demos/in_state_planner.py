# Copyright 2021 CNRS - Airbus SAS
# Author: Florent Lamiraux, Joseph Mirabel
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from CORBA import Any, ORB_init, TC_long
from hpp.corbaserver import wrap_delete
from hpp_idl.hpp import Error as HppError

class InStatePlanner:
    # Default path planner
    parameters = {'kPRM*/numberOfNodes': Any(TC_long,2000)}
    pathProjectorType = "Progressive"
    pathProjectorParam = 0.02

    def wd(self, o):
        return o
        return wrap_delete(o, self.ps.client.basic._tools)

    def __init__(self, ps, graph):
        self.orb = ORB_init()
        self.ps = ps
        self.graph = graph
        self.plannerType = "BiRRT*"
        self.optimizerTypes = []
        self.maxIterPathPlanning = None
        self.timeOutPathPlanning = None
        self.manipulationProblem = \
            self.wd(self.ps.hppcorba.problem.getProblem())
        self.crobot = self.wd(self.manipulationProblem.robot())
        # create a new problem with the robot
        self.cproblem = self.wd(self.ps.hppcorba.problem.createProblem\
                                (self.crobot))
        # Set parameters
        for k, v in self.parameters.items():
            self.cproblem.setParameter(k, v)
        # Set config validation
        self.cproblem.clearConfigValidations()
        for cfgval in [ "CollisionValidation", "JointBoundValidation"]:
            self.cproblem.addConfigValidation\
                (self.wd(self.ps.hppcorba.problem.createConfigValidation
                         (cfgval, self.crobot)))
        # Set steering method in problem
        self.cproblem.setSteeringMethod\
            (self.ps.hppcorba.problem.createSteeringMethod\
             ("Straight", self.cproblem))
        # Set path projector in problem
        if self.pathProjectorType:
            self.cproblem.setPathProjector\
                (self.ps.hppcorba.problem.createPathProjector\
                 (self.pathProjectorType, self.cproblem,
                  self.pathProjectorParam))
        # Add obstacles to new problem
        for obs in self.ps.getObstacleNames(True,False):
            self.cproblem.addObstacle\
                (self.wd(self.ps.hppcorba.problem.getObstacle(obs)))

    def setEdge(self, edge):
        # get reference to constraint graph
        cgraph = self.wd(self.manipulationProblem.getConstraintGraph())
        self.cedge = self.wd(cgraph.get(self.graph.edges[edge]))
        self.cconstraints = self.wd(self.cedge.pathConstraint())
        self.cproblem.setPathValidation(self.cedge.getPathValidation())
        self.cproblem.setConstraints(self.cconstraints)
        self.cproblem.setSteeringMethod\
            (self.wd(self.cedge.getSteeringMethod()))
        self.cproblem.filterCollisionPairs()

    def setReedsAndSheppSteeringMethod(self):
        sm = self.wd(self.ps.hppcorba.problem.createSteeringMethod\
            ("ReedsShepp", self.cproblem))
        self.cproblem.setSteeringMethod(sm)
        dist = self.ps.hppcorba.problem.createDistance("ReedsShepp",
                                                       self.cproblem)
        self.cproblem.setDistance(dist)

    def buildRoadmap(self, qinit):
        self.wd(self.cconstraints.getConfigProjector()).\
            setRightHandSideFromConfig(qinit)
        self.cproblem.setInitConfig(qinit)
        self.cproblem.resetGoalConfigs()
        self.croadmap = self.wd(self.ps.client.manipulation.problem.\
                               createRoadmap(
                self.wd(self.cproblem.getDistance()),
                self.crobot))
        cgraph = self.wd(self.manipulationProblem.getConstraintGraph())
        self.croadmap.constraintGraph(cgraph)
        self.planner = self.wd(self.ps.hppcorba.problem.createPathPlanner(
            self.plannerType,
            self.cproblem,
            self.croadmap))
        if self.maxIterPathPlanning:
            self.planner.maxIterations(self.maxIterPathPlanning)
        if self.timeOutPathPlanning:
            self.planner.timeOut(self.timeOutPathPlanning)
        path = self.wd(self.planner.solve())

    def createEmptyRoadmap(self):
        self.croadmap = self.wd(self.ps.hppcorba.problem.createRoadmap(
                self.wd(self.cproblem.getDistance()),
                self.crobot))

    ## Call steering method of selected edge
    #
    #  \param qinit, qgoal initial and end configurations
    #  \param validate whether the path should ba validated
    #
    #  \todo call path projector
    def directPath(self, qinit, qgoal, validate):
        sm = self.wd(self.cproblem.getSteeringMethod())
        p = sm.call(qinit, qgoal)
        if not p: return False, None, "Steering method failed"
        res = p is not None; p1 = p; msg = ""
        if validate:
            pv = self.wd(self.cedge.getPathValidation())
            res, p1, msg = pv.validate(p, False)
        return res, p1, msg

    def computePath(self, qinit, qgoals, resetRoadmap = False):
        self.wd(self.cconstraints.getConfigProjector()).\
            setRightHandSideFromConfig(qinit)
        self.cproblem.setInitConfig(qinit)
        self.cproblem.resetGoalConfigs()
        for q in qgoals:
            if (type(q) != list and type(q) != tuple) or \
               len(q) != self.ps.robot.getConfigSize():
                raise TypeError\
                    ('Expected a list of configurations as argument qgoals.' +
                     ' One element turns out to be {};'.format(q))
            res, q2 = self.cconstraints.apply(q)
            self.cproblem.addGoalConfig(q2)

        if resetRoadmap or not hasattr(self, 'roadmap'):
            self.createEmptyRoadmap()
        self.planner = self.wd(self.ps.hppcorba.problem.createPathPlanner(
            self.plannerType,
            self.cproblem,
            self.croadmap))
        if self.maxIterPathPlanning:
            self.planner.maxIterations(self.maxIterPathPlanning)
        if self.timeOutPathPlanning:
            self.planner.timeOut(self.timeOutPathPlanning)
        if self.maxIterPathPlanning is None and self.timeOutPathPlanning is \
           None:
            self.planner.stopWhenProblemIsSolved(True)
        path = self.wd(self.planner.solve())
        path = self.optimizePath(path)
        return path

    def optimizePath(self, path):
        for optType in self.optimizerTypes:
            optimizer = self.wd(self.ps.hppcorba.problem.createPathOptimizer\
                (optType, self.manipulationProblem))
            try:
                optpath = optimizer.optimize(path)
                # optimize can return path if it couldn't find a better one.
                # In this case, we have too different client refering to the
                # same servant.
                # thus the following code deletes the old client, which
                # triggers deletion of the servant and the new path points to
                # nothing...
                # path = self.wd(optimizer.optimize(path))
                from hpp.corbaserver.tools import equals
                if not equals(path, optpath):
                    path = self.wd(optpath)
            except HppError as e:
                print("could not optimize", e)
        return path

    def timeParameterization(self, path):
        for optType in [ "EnforceTransitionSemantic",
                         "SimpleTimeParameterization" ]:
            optimizer = self.wd(self.ps.hppcorba.problem.createPathOptimizer\
                (optType, self.manipulationProblem))
            try:
                optpath = optimizer.optimize(path)
                # optimize can return path if it couldn't find a better one.
                # In this case, we have too different client refering to the
                # same servant. thus the following code deletes the old client,
                # which triggers deletion of the servant and the new path points
                # to nothing...
                # path = self.wd(optimizer.optimize(path))
                from hpp.corbaserver.tools import equals
                if not equals(path, optpath):
                    path = self.wd(optpath)
            except HppError as e:
                print("could not time parameterize", e)
        return path

    ## Solve traveling salesman problem between configurations
    # \param configs list of configurations.
    #
    # \li Compute matrix of paths between all configurations by calling method
    #     computePath,
    # \li compute matrix of distances as path lengths,
    # \li call TSP solver in module agimus_demos.pytsp
    def solveTSP(self, configs, resetRoadmapEachTime, pb_kwargs={}):
        # Compute matrix of paths
        l = len(configs)
        paths = [ [ None, ] * l for _ in range(l) ]
        distances = np.zeros((l,l))
        from itertools import combinations
        pbar = progressbar_object(desc="Computing distance matrix",
                                  total=l*(l-1)/2, **pb_kwargs)
        for i, j in combinations(range(l), 2):
            try:
                path = paths[i][j] = self.computePath\
                    (configs[i],configs[j], resetRoadmap=resetRoadmapEachTime)
                distances[j,i] = distances[i,j] = path.length()
            except Exception as e:
                pbar.write("Failed to connect {} to {}: {}".format(i, j,e))
                paths[i][j] = None
                distances[j,i] = distances[i,j] = 1e8
            pbar.update()
        if l > 15:
            from agimus_demos.pytsp.approximative_kopt import solve_3opt as \
                solve_tsp
        else:
            from agimus_demos.pytsp.dynamic_programming import \
                solve_with_heuristic as solve_tsp
        distance, permutation = solve_tsp(distances)
        # rotate permutation so that 0 is the first index.
        solution = []
        permutation = [0,] + permutation + [0,]
        assert permutation[0] == 0, str(permutation[0]) + " should be 0"
        for i,j in zip(permutation,permutation[1:]):
            p = paths[i][j] if i < j else self.wd(paths[j][i].reverse())
            solution.append(p)
        return permutation, solution

    def writeRoadmap(self, filename):
        cgraph = self.wd(self.manipulationProblem.getConstraintGraph())
        self.ps.client.manipulation.problem.writeRoadmap\
                       (filename, self.croadmap, self.crobot, cgraph)

    def readRoadmap(self, filename):
        cgraph = self.wd(self.manipulationProblem.getConstraintGraph())
        self.croadmap = self.ps.client.manipulation.problem.readRoadmap\
                       (filename, self.crobot, cgraph)

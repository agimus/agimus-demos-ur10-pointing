import numpy as np

class SecurityMargins(object):
    def __init__(self, jointNames):
        self.jointNames = jointNames
        n = len(jointNames)+1
        self.margins = np.zeros((n,n))
    def jid(self, j):
        if not isinstance(j, str): return j
        if j == "universe": return 0
        return self.jointNames.index(j) + 1
    def set(self, j1, j2, margin):
        i1 = self.jid(j1)
        i2 = self.jid(j2)
        self.margins[j1, j2] = self.margins[j2, j1] = margin
    def apply(self, cproblem):
        import hpp_idl
        try:
            if cproblem._is_a(hpp_idl.hpp.core_idl._objref_Problem._NP_RepositoryId):
                cproblem.setSecurityMargins(self.margins.tolist())
                return
        except:
            pass
        #TODO apply it to ProblemSolver when exposed.
        raise ValueError('expects a instance of hpp_idl.hpp.core_idl._objref_Problem._NP_RepositoryId')

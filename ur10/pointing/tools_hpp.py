# BSD 2-Clause License

# Copyright (c) 2021, Florent Lamiraux
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.

# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

tool_gripper = 'ur10/gripper'

class ConfigGenerator(object):
    def __init__(self, graph):
        self.robot = graph.robot
        self.graph = graph

    def generateValidConfigForHandle(self, handle, qinit, qguesses = [],
                                     NrandomConfig=10):
        edge = tool_gripper + " > " + handle
        ok = False
        from itertools import chain
        def project_and_validate(e, qrhs, q):
            res, qres, err = self.graph.generateTargetConfig (e, qrhs, q)
            return res and self.robot.configIsValid(qres), qres
        qpg, qg = None, None
        for qrand in chain(qguesses, (self.robot.shootRandomConfig()
                                      for _ in range(NrandomConfig))):
            res, qpg = project_and_validate (edge+" | f_01", qinit, qrand)
            if res:
                ok, qg = project_and_validate(edge+" | f_12", qpg, qpg)
                if ok: break
        return ok, qpg, qg

    def generateValidConfig(self, constraint, qguesses = [], NrandomConfig=10):
        from itertools import chain
        for qrand in chain(qguesses, (self.robot.shootRandomConfig()
                                      for _ in range(NrandomConfig))):
            res, qres = constraint.apply (qrand)
            if res and self.robot.configIsValid(qres):
                return True, qres
        return False, None

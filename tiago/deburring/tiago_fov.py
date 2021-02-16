from __future__ import print_function
import pinocchio, hpp.rostools, hppfcl, numpy as np

def _add_fov_to_gui(gui, name, filename_or_pts, group=None, color=None):
    if isinstance(filename_or_pts, str):
        gui.addMesh(name, filename_or_pts)
        if color is not None: gui.setColor(name, [0.1, 0.1, 0.9, 0.2])
    else:
        assert color is not None
        gui.addCurve(name, filename_or_pts, color)
        gui.setCurveMode(name, "TRIANGLE_FAN")
    if group is not None: gui.addToGroup(name, group)
    gui.setBoolProperty(name, "Selectable", False)

_tetahedron_tris = hppfcl.StdVec_Triangle()
_tetahedron_tris.append(hppfcl.Triangle(0, 1, 2))
_tetahedron_tris.append(hppfcl.Triangle(0, 2, 3))
_tetahedron_tris.append(hppfcl.Triangle(0, 3, 4))
_tetahedron_tris.append(hppfcl.Triangle(0, 4, 1))

class TiagoFOV:
    def __init__(self,
            urdfString=None,
            urdfFilename=None,
            fov = np.radians((49.5, 60)),
            geoms = [ "arm_3_link_0" ],
            ):
        if isinstance(fov, str):
            self.fov = fov
            fov_fcl = hppfcl.MeshLoader().load(hpp.rostools.retrieve_resource(fov))
        else:
            # build FOV tetahedron
            dx, dy = np.sin(fov)
            pts = hppfcl.StdVec_Vec3f()
            self.fov = [(0,0,0),
                    ( dx,  dy, 1),
                    (-dx,  dy, 1),
                    (-dx, -dy, 1),
                    ( dx, -dy, 1),]
            pts.extend([ np.array(e) for e in self.fov ])
            self.fov.append(self.fov[1])
            fov_fcl = hppfcl.BVHModelOBBRSS()
            fov_fcl.beginModel(4, 5)
            fov_fcl.addSubModel(pts, _tetahedron_tris)
            fov_fcl.endModel()

        self.cos_angle_thr = np.cos(np.radians(70))
        if urdfFilename is None:
            assert urdfString is not None
            # Pinocchio does not allow to build a GeometryModel from a XML string.
            urdfFilename = '/tmp/tmp.urdf'
            with open(urdfFilename, 'w') as f:
                f.write(urdfString)
        self.model, self.gmodel = pinocchio.buildModelsFromUrdf(
                hpp.rostools.retrieve_resource(urdfFilename),
                root_joint = pinocchio.JointModelPlanar(),
                geometry_types=pinocchio.GeometryType.COLLISION)

        id_parent_frame = self.model.getFrameId("xtion_rgb_optical_frame")
        parent_frame = self.model.frames[id_parent_frame]
        go = pinocchio.GeometryObject("field_of_view", id_parent_frame, parent_frame.parent,
                fov_fcl, parent_frame.placement)

        self.gid_field_of_view = self.gmodel.addGeometryObject(go, self.model)
        for n in geoms:
            assert self.gmodel.existGeometryName(n)
            self.gmodel.addCollisionPair(pinocchio.CollisionPair(self.gmodel.getGeometryId(n), self.gid_field_of_view))

        self.data = pinocchio.Data(self.model)
        self.gdata = pinocchio.GeometryData(self.gmodel)

    def reduceModel(self, jointsToRemove, config, len_prefix=0):
        jointIds = [ self.model.getJointId(jn[len_prefix:]) for jn in jointsToRemove ]
        self.model, self.gmodel = pinocchio.buildReducedModel(self.model, self.gmodel, jointIds, np.array(config))
        self.data = pinocchio.Data(self.model)
        self.gdata = pinocchio.GeometryData(self.gmodel)

    def updateGeometryPlacements(self, q):
        pinocchio.framesForwardKinematics(self.model, self.data, np.array(q))
        pinocchio.updateGeometryPlacements(self.model, self.data, self.gmodel, self.gdata)

    def tagToTetahedronPts(self, oMt, size):
        """ It assumes that updateGeometryPlacements has been called """
        if not isinstance(oMt, pinocchio.SE3):
            oMt = pinocchio.XYZQUATToSE3(oMt)

        pts = hppfcl.StdVec_Vec3f()
        idc = self.model.getFrameId("xtion_rgb_optical_frame")

        C = self.data.oMf[idc].translation + 0.002*self.data.oMf[idc].rotation[:,2]
        pts.append(C)
        for pt in [
                np.array(( size / 2,  size / 2, 0)),
                np.array((-size / 2,  size / 2, 0)),
                np.array((-size / 2, -size / 2, 0)),
                np.array(( size / 2, -size / 2, 0)), ]:
            P = oMt * pt
            u = (C-P)
            u /= np.linalg.norm(u)
            pts.append(P + 0.002 * u)
        return pts

    def tagVisible(self, oMt, size):
        """ It assumes that updateGeometryPlacements has been called """
        idc = self.model.getFrameId("xtion_rgb_optical_frame")
        camera = self.model.frames[idc]
        oMc = self.data.oMf[idc]

        # oMc.rotation[:,2] view axis
        # oMt.rotation[:,2] normal of the tag, on the tag side.
        if not isinstance(oMt, pinocchio.SE3):
            oMt = pinocchio.XYZQUATToSE3(oMt)
        cos_theta = - np.dot(oMc.rotation[:,2], oMt.rotation[:,2])
        if cos_theta < self.cos_angle_thr:
            return False

        pts = self.tagToTetahedronPts(oMt, size)

        tetahedron = hppfcl.BVHModelOBBRSS()
        tetahedron.beginModel(4, 5)
        tetahedron.addSubModel(pts, _tetahedron_tris)
        tetahedron.endModel()

        request = hppfcl.CollisionRequest()
        Id = hppfcl.Transform3f()
        for go, oMg in zip(self.gmodel.geometryObjects, self.gdata.oMg):
            # Don't check for collision with the camera, except with the field_of_view
            if go.parentJoint == camera.parent and go.name != "field_of_view": continue
            result = hppfcl.CollisionResult()
            hppfcl.collide(go.geometry, hppfcl.Transform3f(oMg), tetahedron, Id, request, result)
            if result.isCollision():
                return False
        return True

    def robotClogsFieldOfView(self):
        """whether the camera is clogged by the selected robot bodies. It assumes that updateGeometryPlacements has been called """
        return pinocchio.computeCollisions(self.gmodel, self.gdata, True)

    def clogged (self, q, robot, tagss, sizess, verbose=False):
        def _print(*args):
            if verbose:
                print(*args)
        # should see at least on tag per tagss
        self.updateGeometryPlacements(q[:-14])
        for tags, sizes in zip(tagss, sizess):
            nvisible = 0
            for oMt, size in zip(robot.hppcorba.robot.getJointsPosition(q, tags), sizes):
                if self.tagVisible(oMt, size):
                    nvisible+=1
            if nvisible == 0:
                _print("No tags visible among ", tags)
                return True
        return False

    def addDriller(self, mesh, frame_name, fMm):
        if not isinstance(fMm, pinocchio.SE3):
            fMm = pinocchio.XYZQUATToSE3(fMm)
        id_parent_frame = self.model.getFrameId(frame_name)
        parent_frame = self.model.frames[id_parent_frame]

        go = pinocchio.GeometryObject("driller", id_parent_frame, parent_frame.parent,
                hppfcl.MeshLoader().load(hpp.rostools.retrieve_resource(mesh)),
                parent_frame.placement * fMm)

        self.gmodel.addGeometryObject(go, self.model)
        self.gmodel.addCollisionPair(pinocchio.CollisionPair(self.gid_field_of_view, self.gmodel.ngeoms-1))
        self.gdata = pinocchio.GeometryData(self.gmodel)

    def appendUrdfModel(self, urdfFilename, frame_name, fMm, prefix = None):
        if not isinstance(fMm, pinocchio.SE3):
            fMm = pinocchio.XYZQUATToSE3(fMm)
        id_parent_frame = self.model.getFrameId(frame_name)

        model, gmodel = pinocchio.buildModelsFromUrdf(
                hpp.rostools.retrieve_resource(urdfFilename),
                geometry_types=pinocchio.GeometryType.COLLISION)

        if prefix is not None:
            for i in range(1, len(model.names)):
                model.names[i] = prefix + model.names[i]
            for f in model.frames:
                f.name = prefix + f.name
            for go in gmodel.geometryObjects:
                go.name = prefix + go.name

        igeom = self.gmodel.ngeoms

        nmodel, ngmodel = pinocchio.appendModel(self.model, model, self.gmodel, gmodel, id_parent_frame, fMm)

        self.gid_field_of_view = ngmodel.getGeometryId("field_of_view")
        for go in gmodel.geometryObjects:
            ngmodel.addCollisionPair(pinocchio.CollisionPair(self.gid_field_of_view,
                ngmodel.getGeometryId(go.name)))

        self.model, self.gmodel = nmodel, ngmodel
        self.data = pinocchio.Data(self.model)
        self.gdata = pinocchio.GeometryData(self.gmodel)

    def loadInGui(self, v):
        gui = v.client.gui
        _add_fov_to_gui(gui, "field_of_view", self.fov,
                color = [0.1, 0.1, 0.9, 0.2],
                group = "robot/tiago/head_2_link")
        idl = self.model.getFrameId("head_2_link")
        idc = self.model.getFrameId("xtion_rgb_optical_frame")
        assert self.model.frames[idl].parent == self.model.frames[idc].parent
        gui.applyConfiguration("field_of_view", pinocchio.SE3ToXYZQUATtuple(
            self.model.frames[idl].placement.inverse() * self.model.frames[idc].placement))
        gui.refresh()

class TiagoFOVGuiCallback:
    def __init__(self, robot, tiago_fov, tags, sizes):
        self.robot = robot
        self.fov = tiago_fov
        self.names = [ "fov_" + n for n in tags ]
        self.tags = tags
        self.sizes = sizes
        self.initialized = False

    def initialize(self, viewer):
        self.initialized = True
        gui = viewer.client.gui
        for name, tag in zip(self.names, self.tags):
            _add_fov_to_gui(gui, name, [ (0,0,0), ] * 6,
                    color = [ 0.1, 0.9, 0.1, 0.2 ],
                    group = "gepetto-gui")

    def __call__(self, viewer, q):
        if not self.initialized:
            self.initialize(viewer)
        gui = viewer.client.gui

        self.fov.updateGeometryPlacements(q[:-14])
        for name, oMt, size in zip (self.names, self.robot.hppcorba.robot.getJointsPosition(q, self.tags), self.sizes):
            pts = [ pt.tolist() for pt in self.fov.tagToTetahedronPts(oMt, size) ]
            gui.setCurvePoints(name, pts + pts[1:2])
            if self.fov.tagVisible(oMt, size):
                gui.setColor(name, [ 0.1, 0.9, 0.1, 0.2 ])
            else:
                gui.setColor(name, [ 0.9, 0.1, 0.1, 0.2 ])

if __name__ == "__main__":
    urdfString = hpp.rostools.process_xacro("package://tiago_description/robots/tiago.urdf.xacro", "robot:=steel", "end_effector:=pal-hey5", "ft_sensor:=schunk-ft")
    tiago_fov = TiagoFOV(urdfString=urdfString)

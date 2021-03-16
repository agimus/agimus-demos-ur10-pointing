from __future__ import print_function
from agimus_vision import py_agimus_vision as av
import numpy as np, pinocchio, sys, os, pymannumopt as mno

def toURDFTag(M):
    x,y,z = M.translation
    ori = '<origin xyz="{} {} {}" '.format(x,y,z)
    r,p,y = pinocchio.rpy.matrixToRpy(M.rotation)
    ori += 'rpy="{} {} {}"/>'.format(r,p,y)
    return ori

def vpToSE3(M):
    return pinocchio.SE3(np.array(M))
def se3toVP(M):
    return av.HomogeneousMatrix(M.homogeneous)

def cam_proj(x):
    return x[:2] / x[2]
def Jcam_proj(x):
    d = 1/x[2]
    return np.array([ [ d, 0, -x[0]*d**2],
                      [ 0, d, -x[1]*d**2] ])

def J_M_times_P(M,P):
    return np.hstack((M.rotation, -np.dot(M.rotation, pinocchio.skew(P))))

def image_point_residual(pij, cMi, oPij):
    return pij - cam_proj(cMi * oPij)
def Jimage_point_residual(pij, cMi, oPij):
    return - np.dot(Jcam_proj(cMi * oPij), J_M_times_P(cMi, oPij))

def image_point_residuals(pijss, cMw, wMis, oPijss):
    return [ pij - cam_proj(cMw * wMi * oPij)
            for pijs, wMi, oPijs in zip(pijss, wMis, oPijss)
            for pij, oPij in zip(pijs, oPijs) ]
def Jimage_point_residuals(pijss, cMw, wMis, oPijss):
    nrows = 0
    for pijs, wMi, oPijs in zip(pijss, wMis, oPijss):
        nrows += 2*len(pijs)

    JcMw = np.zeros((nrows, 6))
    JwMis = []

    JcMi = np.zeros((2,6))

    r = 0
    for pijs, wMi, oPijs in zip(pijss, wMis, oPijss):
        XiMw = wMi.toActionMatrixInverse()
        JwMi = np.zeros((nrows, 6))
        for pij, oPij in zip(pijs, oPijs):
            JcMi = Jimage_point_residual(pij, cMw * wMi, oPij)
            JcMw[r:r+2] = np.dot(JcMi, XiMw)
            JwMi[r:r+2] = JcMi
            r+=2
        JwMis.append(JwMi)
    return JcMw, JwMis

def plane_residual(pi, wMis, oPijss):
    #TODO should be this but we add the 4 mm that we measured on the real model
    return np.array([ (np.dot(pi[:3], wMi * oPij) + pi[3] ) for wMi, oPijs in zip(wMis, oPijss) for oPij in oPijs ])
def Jplane_residual(pi, wMis, oPijss):
    # derivative wrt to pi
    Jpi = [ (wMi*oPij).tolist() + [1,] for wMi, oPijs in zip(wMis, oPijss) for oPij in oPijs ]
    # derivative wrt to wMis
    JwMis = []
    row = 0
    for wMi, oPijs in zip(wMis, oPijss) :
        JwMi = np.zeros((len(Jpi), 6))
        for oPij in oPijs:
            JwMi[row,:] = np.dot(pi[:3], J_M_times_P(wMi, oPij))
            row += 1
        JwMis.append(JwMi)
    return Jpi, JwMis

class Image:
    def __init__(self, tags, guess_cMis, pijss, oPijss, coplanar_tags_s):
        self.tags = tags
        self.guess_cMis = guess_cMis
        self.pijss = pijss
        self.oPijss = oPijss
        # list of tuple (plane id, list of visible tags)
        self.coplanar_tags_s = []

        tags_set = set(tags)
        for ip, coplanar_tags in enumerate(coplanar_tags_s):
            coplanar_tags_set = tags_set.intersection(coplanar_tags)
            if len(coplanar_tags_set) > 0:
                tag_indices = [ tags.index(tag) for tag in coplanar_tags_set ]
                self.coplanar_tags_s.append( (ip, tag_indices) )

def makeImage(Img, tag_defs, coplanar_tags_s):
    tags = []
    pijss = []
    oPijss = []
    cMis = []
    first = True
    for tag, (id, size) in enumerate(tag_defs):
        aprilTag = av.makeAprilTag()
        if first:
            aprilTag.detector().imageReady = False
            first = False
        aprilTag.cameraParameters(cam)
        aprilTag.addTag(id, size, av.HomogeneousMatrix())
        if aprilTag.detect(Img):
            aprilTag.drawDebug(Img)
            oMt = np.array(aprilTag.getPose())
            # TODO initial guess
            cMis.append(vpToSE3(oMt))
            tags.append(tag)
            pijss.append([ np.array(v) for v in aprilTag.getPoints(cam, id) ])
            oPijss.append([ np.array(v) for v in av.aprilTagPoints(size) ])
        del aprilTag
    return Image(tags, cMis, pijss, oPijss, coplanar_tags_s)

class Variables:
    def __init__(self, ncameras, ntags, nplanes):
        self.ncameras = ncameras
        self.ntags = ntags
        self.nplanes = nplanes

        from pinocchio import liegroups
        spaces = [ liegroups.Rn(4), ] * nplanes \
                + [ liegroups.SE3() ] * ncameras \
                + [ liegroups.SE3() ] * ntags
        self.space = spaces[0]
        for space in spaces[1:]:
            self.space *= space

    def plane(self, X, i):
        assert i < self.nplanes
        return X[4*i:4*(i+1)]
    def camera(self, X, i, asSE3=True):
        assert i < self.ncameras
        s = self.nplanes*4
        if asSE3:
            return pinocchio.XYZQUATToSE3(X[s+7*i:s+7*(i+1)])
        else:
            return X[s+7*i:s+7*(i+1)]
    def tag(self, X, i, asSE3=True):
        assert i < self.ntags
        s = self.nplanes*4+self.ncameras*7
        if asSE3:
            return pinocchio.XYZQUATToSE3(X[s+7*i:s+7*(i+1)])
        else:
            return X[s+7*i:s+7*(i+1)]

    def Jplane(self, Jx, i):
        assert i < self.nplanes
        return Jx[:,4*i:4*(i+1)]
    def Jcamera(self, Jx, i):
        assert i < self.ncameras
        s = self.nplanes*4
        return Jx[:,s+6*i:s+6*(i+1)]
    def Jtag(self, Jx, i):
        assert i < self.ntags
        s = self.nplanes*4+self.ncameras*6
        return Jx[:,s+6*i:s+6*(i+1)]

class ImageResiduals(mno.VectorFunction):
    def __init__(self, variables, images):
        mno.VectorFunction.__init__(self)
        self.images = images
        self.variables = variables

    def dimension(self):
        nb_pts_per_image = [ sum([ len(pijs) for pijs in image.pijss ]) for image in self.images ]
        nb_pts = sum(nb_pts_per_image)
        return 2 * nb_pts

    def f(self, X, f):
        r = 0
        for ic, image in enumerate(self.images):
            cMw = self.variables.camera(X, ic)
            # build cMis
            wMis = [ self.variables.tag(X, it) for it in image.tags ]
            residuals = image_point_residuals(image.pijss, cMw, wMis, image.oPijss)
            for residual in residuals:
                f[r:r+2] = residual
                r+=2
        assert r == f.shape[0]

    def f_fx(self, X, f, fx):
        fx[:] = np.zeros_like(fx)

        r = 0
        for ic, image in enumerate(self.images):
            cMw = self.variables.camera(X, ic)
            # build cMis
            wMis = [ self.variables.tag(X, it) for it in image.tags ]
            residuals = image_point_residuals(image.pijss, cMw, wMis, image.oPijss)
            JcMw, JwMis = Jimage_point_residuals(image.pijss, cMw, wMis, image.oPijss)
            nr = JcMw.shape[0]
            self.variables.Jcamera(fx[r:r+nr,:], ic)[:] = JcMw
            for it, JwMi in zip(image.tags, JwMis):
                self.variables.Jtag(fx[r:r+nr,:], it)[:] = JwMi
            for residual in residuals:
                f[r:r+2] = residual
                r+=2

class PlaneResiduals(mno.VectorFunction):
    def __init__(self, variables, images):
        mno.VectorFunction.__init__(self)
        self.images = images
        self.variables = variables

    def dimension(self):
        nb_pts_per_plane = [
                sum([ len(image.oPijss[tagid]) for tagid in tag_indices ])
                for image in self.images
                for _, tag_indices in image.coplanar_tags_s ]
        assert sum(nb_pts_per_plane) == 4 * sum([ len(tag_indices) for image in self.images for _, tag_indices in image.coplanar_tags_s ])
        return sum(nb_pts_per_plane)

    def f(self, X, f):
        r = 0
        for ic, image in enumerate(self.images):
            for ip, tag_indices in image.coplanar_tags_s:
                pi = self.variables.plane(X, ip)
                wMis = [ self.variables.tag(X, image.tags[k]) for k in tag_indices ]
                oPijss = [ image.oPijss[k] for k in tag_indices ]
                n = sum( [ len(oPijs) for oPijs in oPijss ] )
                f[r:r+n] = plane_residual(pi, wMis, oPijss)
                r += n
        assert r == f.shape[0]
    def f_fx(self, X, f, fx):
        r = 0
        fx[:] = np.zeros_like(fx)
        for ic, image in enumerate(self.images):
            for ip, tag_indices in image.coplanar_tags_s:
                pi = self.variables.plane(X, ip)
                wMis = [ self.variables.tag(X, image.tags[k]) for k in tag_indices ]
                oPijss = [ image.oPijss[k] for k in tag_indices ]
                n = sum( [ len(oPijs) for oPijs in oPijss ] )
                f[r:r+n] = plane_residual(pi, wMis, oPijss)
                Jpi, JwMis = Jplane_residual(pi, wMis, oPijss)
                self.variables.Jplane(fx[r:r+n,:], ip)[:] = Jpi
                for k, JwMi in zip(tag_indices, JwMis):
                    self.variables.Jtag(fx[r:r+n,:], image.tags[k])[:] = JwMi
                r += n
        assert r == f.shape[0]

class PlaneUnitNormal(mno.VectorFunction):
    def __init__(self, variables):
        mno.VectorFunction.__init__(self)
        self.variables = variables

    def dimension(self):
        return self.variables.nplanes

    def f(self, X, f):
        for ip in range(self.variables.nplanes):
            pi = self.variables.plane(X, ip)
            normal = pi[:3]
            f[ip] = np.sum(normal**2) - 1

    def f_fx(self, X, f, fx):
        fx[:] = np.zeros_like(fx)
        for ip in range(self.variables.nplanes):
            pi = self.variables.plane(X, ip)
            normal = pi[:3]
            f[ip] = np.sum(normal**2) - 1
            J = self.variables.Jplane(fx[ip:ip+1,:], ip)
            J[:,:3] = 2 * normal
            J[:,3] = 0

class LockTag(mno.VectorFunction):
    def __init__(self, variables, itag, wMtd):
        mno.VectorFunction.__init__(self)
        self.variables = variables
        self.itag = itag
        self.tdMw = wMtd.inverse()

    def dimension(self):
        return 6

    def f(self, X, f):
        wMt = self.variables.tag(X, self.itag)
        f[:] = pinocchio.log6(self.tdMw * wMt).vector

    def f_fx(self, X, f, fx):
        wMt = self.variables.tag(X, self.itag)
        tdMt = self.tdMw * wMt
        f[:] = pinocchio.log6(tdMt).vector
        fx[:] = np.zeros_like(fx)
        self.variables.Jtag(fx, self.itag)[:] = pinocchio.Jlog6(tdMt)

class Stack(mno.VectorFunction):
    def __init__(self, functions):
        mno.VectorFunction.__init__(self)
        self.functions = functions
        self.dims = [ func.dimension() for func in self.functions ]

    def dimension(self):
        return sum(self.dims)

    def f(self, X, f):
        r = 0
        for d, func in zip(self.dims, self.functions):
            func.f(X, f[r:r+d])
            r += d

    def f_fx(self, X, f, fx):
        r = 0
        for d, func in zip(self.dims, self.functions):
            func.f_fx(X, f[r:r+d], fx[r:r+d,:])
            r += d

datadir="/home/jmirabel/devel/tiago/catkin_ws/src/tiago_calibration/part2/"

cam = av.makeTiagoCameraParameters()

pi0 = [1, 0, 0, 0]

tag_defs = (
        (15, 0.0845), (6, 0.0845), (1, 0.0845),
        # (100, 0.041), (101, 0.041),
        (2, 0.053), (3, 0.053), (4, 0.053),
        )
def idx(tag_id):
    for k, (id, _) in enumerate(tag_defs):
        if id == tag_id: return k
def indices(tag_ids): return [ idx(tag_id) for tag_id in tag_ids ]
coplanar_tags_s = (
        indices(( 15, 6, 1 )), # corresponds to tag ids (15, 6, 1)
        #( 0, 1, 2, 3, 4 ), # corresponds to tag ids (15, 6, 1, 100, 101)
        indices( (2,3,4) ), # corresponds to tag ids (2, 3, 4)
        )

images = []
I = av.Image()
for fn in [ "001.png", "002.png", "003.png", "005.png" ]:
    I.read(datadir + "base/" + fn)
    #I.initDisplay()
    I.display()
    images.append(makeImage(I, tag_defs, coplanar_tags_s))
    I.flush()
    I.getClick()

# Assume w is the object base link
# We fix the pose of tag with ID 15
wMt0 = pinocchio.SE3(pinocchio.rpy.rpyToMatrix(1.63558443485, -0.0100636021863, -0.0123677826844),
        np.array([ -0.132714992453, -0.0338492746834, 0.546713277339 ]))

variables = Variables(len(images), len(tag_defs), len(coplanar_tags_s))
image_residuals = ImageResiduals(variables, images)
plane_residuals = PlaneResiduals(variables, images)
plane_unit_normal = PlaneUnitNormal(variables)
lock_tag_0 = LockTag(variables, 0, wMt0)
constraints = Stack([
    plane_residuals,
    plane_unit_normal,
    lock_tag_0,
    ])

# Solve optimization problem
gn = mno.GaussNewton(variables.space.nq, variables.space.nv)
gn.xtol = 1e-10
gn.fxtol2 = 1e-12
gn.maxIter = 100
gn.verbose = False

penalty = mno.Penalty(variables.space.nq, variables.space.nv, constraints.dimension())
penalty.etol2 = 1e-12;
penalty.fxtol2 = 1e-10;
penalty.maxIter = 40;
penalty.verbose = True

# Compute initial guess
X0 = variables.space.neutral
for k in range(variables.nplanes):
    variables.plane(X0, k)[0] = 1
# Assume w is the object base link
variables.tag(X0, 0, False)[:] = pinocchio.SE3ToXYZQUAT(wMt0)
unknown_tags = set(range(1, variables.ntags))
unknown_cameras = set(range(variables.ncameras))
unhandled_images = images[:]
while len(unknown_tags) > 0:
    if len(unhandled_images) == 0:
        raise RuntimeError("some tags were not seen")
    for im, image in enumerate(images):
        if image not in unhandled_images: continue
        cMw = None
        for tag, cMi in zip(image.tags, image.guess_cMis):
            if tag in unknown_tags: continue
            # compute camera pose
            cMw = cMi * variables.tag(X0, tag).inverse()
            variables.camera(X0, im, False)[:] = pinocchio.SE3ToXYZQUAT(cMw)
            unhandled_images.remove(image)
            break
        if cMw is None: continue
        # compute pose of wMi
        for tag, cMi in zip(image.tags, image.guess_cMis):
            if tag not in unknown_tags: continue
            # compute tag pose
            wMi = cMw.inverse() * cMi
            variables.tag(X0, tag, False)[:] = pinocchio.SE3ToXYZQUAT(wMi)
            unknown_tags.remove(tag)

res, X = penalty.minimize(image_residuals, constraints, X0, gn, integrate=variables.space.integrate)

# Compute position of holes
aprilTagPart = av.makeAprilTag()
aprilTagPart.cameraParameters(cam)
for k, (id, size) in enumerate(tag_defs):
    wMi = variables.tag(X, k)
    aprilTagPart.addTag(id, size, se3toVP(wMi))

aprilTagHole = av.makeAprilTag()
aprilTagHole.cameraParameters(cam)
aprilTagHole.addTag(230, 0.040, av.HomogeneousMatrix())
detector = aprilTagHole.detector()
J = av.Image()
# Set this to False to enable visualization
initDisplay = True
bMhs = []
for i in range(40):
    img = datadir + "hole/{:03}.png".format(i)
    tagCenters = []
    print("reading " + img)
    J.read(img)
    if not initDisplay:
        initDisplay = True
        J.initDisplay()
    J.display()
    detector.imageReady = False

    partDetected = aprilTagPart.detect(J)
    holeDetected = aprilTagHole.detect(J)
    if partDetected and holeDetected:
        cMb = vpToSE3(aprilTagPart.getPose())

        aprilTagPart.drawDebug(J)
        aprilTagHole.drawDebug(J)
        cMb = vpToSE3(aprilTagPart.getPose())
        print(cMb.translation, pinocchio.rpy.matrixToRpy(cMb.rotation))
        cMh = vpToSE3(aprilTagHole.getPose())
        bMhs.append(cMb.inverse() * cMh)
    else:
        print("Could not detect part or hole. Detected tags {}".format(detector.getTagsId()))

    J.flush()
    J.getClick()

def tagsInUrdfFormat(tags, bMis):
    tag_fmt="""
      <link name="tag36_11_{id:05}">
        <visual>
          <geometry>
            <mesh filename="package://gerard_bauzil/meshes/apriltag_36h11/tag36_11_{id:05}.dae" scale="{size} {size} 1."/>
          </geometry>
        </visual>
      </link>
      <joint name="to_tag_{id:05}" type="fixed">
        <parent link="base_link"/>
        <child link="tag36_11_{id:05}"/>
        {origin}
      </joint>"""
    urdf_str = ""
    for (id, size), bMi in zip(tags, bMis):
        urdf_str += tag_fmt.format(id=id, size=size, origin=toURDFTag(bMi))
    return urdf_str

def holesInUrdfFormat(bMhs):
    hole_fmt="""
      <link name="hole_{id:02}_link">
        <visual>
          <geometry>
            <sphere radius="0.005" />
          </geometry>
        </visual>
      </link>
      <joint name="to_hole_{id:02}" type="fixed">
        <parent link="base_link"/>
        <child link="hole_{id:02}_link"/>
        {origin}
      </joint>"""
    hole_str = ""
    for i, bMh in enumerate(bMhs):
        hole_str += hole_fmt.format(id=i, origin = toURDFTag(bMh))
    return hole_str

def generate_urdf(tags, bMis, bMhs):
    return tagsInUrdfFormat(tags, bMis) + "\n" + holesInUrdfFormat(bMhs)

# Generate URDF with
print("""

# You may generate a URDF with
with open('/tmp/foo.urdf', 'w') as f:
    f.write(generate_urdf(tag_defs, [ variables.tag(X, i) for i in range(variables.ntags) ], bMhs))
""")


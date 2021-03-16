import hppfcl, random, numpy as np, eigenpy

# Algo to generate points
# select a face (i.e a flat convex polyhedron)
# select a point on this face
# compute the translation and rotation

def generate_srdf(bvh_file, N, output):
    loader = hppfcl.MeshLoader ()
    bvh = loader.load(bvh_file)

    handle = """
    <handle name="{name}" clearance="{clearance}">
        <position xyz="{xyz}" xyzw="{xyzw}"/>
        <link name="{link}" />
        <mask>{mask}</mask>
    </handle>"""

    mask = "1 1 1 1 1 1"
    clearance = 0.01
    link = "cylinder"

    output.write("""<robot name="cylinder">""")
    for ih in range(N):
        it = random.randint(0,bvh.num_tris-1)
        tri = bvh.tri_indices(it)

        ws = [ random.random() for _ in range(3) ]
        wt = sum(ws)

        ps = [ bvh.vertices(i) for i in tri ]
        p = sum((wi/wt*pi for wi,pi in zip(ws,ps)))

        x = -np.cross(ps[1]-ps[0], ps[2]-ps[0])
        x /= np.linalg.norm(x)
        p -= 0.05*x
        quat = eigenpy.Quaternion.FromTwoVectors(np.array([1,0,0]), x)

        output.write(handle.format(
            name="handle_"+str(ih),
            clearance=clearance,
            xyz = " ".join([ str(v) for v in p ]),
            xyzw = " ".join([ str(quat[i]) for i in range(4) ]),
            link = link,
            mask=mask))
    output.write("""</robot>""")

if __name__ == "__main__":
    bvh_file = "/home/jmirabel/devel/hpp/src/agimus-demos/meshes/cylinder.stl"
    N = 10
    import sys
    generate_srdf(bvh_file, N, sys.stdout)

import mujoco_py
from dm_control import mjcf, mujoco
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-n", "--num_links", help="Specifies number of links in chain", default="10")
parser.add_argument("-s", "--scale", help="Specifies scale of geoms", default=".001")
parser.add_argument("-b", "--buffer", help="Specifies buffer", default="1.5")

args = parser.parse_args()

def create_n_links(n, base_file='base.xml'):
    # Parse from path
    mjcf_model = mjcf.from_path(base_file)
    even_link_quat = "0.707107 0 0 0.707107"
    odd_link_quat = "1 0 0 0"
    body = ""
    ypos = (n * (54.3 - 2 / 2) / 1000 + 4.45 / 1000 + 64.3 / 2000)
    m = 91 / 23
    assets = mjcf_model.asset
    assets.add('mesh', file="meshes/convex_0_lab.stl", name="torus0", scale=args.scale + " " + args.scale + " " + args.scale)
    assets.add('mesh', file="meshes/convex_1_lab.stl", name="torus1", scale=args.scale + " " + args.scale + " " + args.scale)
    assets.add('mesh', file="meshes/convex_2_lab.stl", name="torus2", scale=args.scale + " " + args.scale + " " + args.scale)
    assets.add('mesh', file="meshes/convex_3_lab.stl", name="torus3", scale=args.scale + " " + args.scale + " " + args.scale)
    assets.add('mesh', file="meshes/convex_4_lab.stl", name="torus4", scale=args.scale + " " + args.scale + " " + args.scale)
    assets.add('mesh', file="meshes/convex_5_lab.stl", name="torus5", scale=args.scale + " " + args.scale + " " + args.scale)
    assets.add('mesh', file="meshes/convex_6_lab.stl", name="torus6", scale=args.scale + " " + args.scale + " " + args.scale)
    assets.add('mesh', file="meshes/convex_7_lab.stl", name="torus7", scale=args.scale + " " + args.scale + " " + args.scale)
    assets.add('mesh', file="meshes/convex_8_lab.stl", name="torus8", scale=args.scale + " " + args.scale + " " + args.scale)
    assets.add('mesh', file="meshes/convex_9_lab.stl", name="torus9", scale=args.scale + " " + args.scale + " " + args.scale)
    assets.add('mesh', file="meshes/convex_10_lab.stl", name="torus10", scale=args.scale + " " + args.scale + " " + args.scale)
    assets.add('mesh', file="meshes/convex_11_lab.stl", name="torus11", scale=args.scale + " " + args.scale + " " + args.scale)
    assets.add('mesh', file="meshes/convex_12_lab.stl", name="torus12", scale=args.scale + " " + args.scale + " " + args.scale)
    assets.add('mesh', file="meshes/convex_13_lab.stl", name="torus13", scale=args.scale + " " + args.scale + " " + args.scale)
    assets.add('mesh', file="meshes/convex_14_lab.stl", name="torus14", scale=args.scale + " " + args.scale + " " + args.scale)
    assets.add('mesh', file="meshes/convex_15_lab.stl", name="torus15", scale=args.scale + " " + args.scale + " " + args.scale)
    assets.add('mesh', file="meshes/convex_16_lab.stl", name="torus16", scale=args.scale + " " + args.scale + " " + args.scale)
    assets.add('mesh', file="meshes/convex_17_lab.stl", name="torus17", scale=args.scale + " " + args.scale + " " + args.scale)
    for i in range(n):
        print(mjcf_model.asset)
        ypos -= (54.3 - float(args.buffer)) / 1000
        if i % 2 == 0:
            body = mjcf_model.worldbody.add('body', name="body" + str(i), pos="-0 0 " + str(ypos), quat=even_link_quat)
        else:
            body = mjcf_model.worldbody.add('body', name="body" + str(i), pos="-0 0 " + str(ypos), quat=odd_link_quat)
        body.add('geom', mesh='torus0', type='mesh')
        body.add('geom', mesh='torus1', type='mesh')
        body.add('geom', mesh='torus2', type='mesh')
        body.add('geom', mesh='torus3', type='mesh')
        body.add('geom', mesh='torus4', type='mesh')
        body.add('geom', mesh='torus5', type='mesh')
        body.add('geom', mesh='torus6', type='mesh')
        body.add('geom', mesh='torus7', type='mesh')
        body.add('geom', mesh='torus8', type='mesh')
        body.add('geom', mesh='torus9', type='mesh')
        body.add('geom', mesh='torus10', type='mesh')
        body.add('geom', mesh='torus11', type='mesh')
        body.add('geom', mesh='torus12', type='mesh')
        body.add('geom', mesh='torus13', type='mesh')
        body.add('geom', mesh='torus14', type='mesh')
        body.add('geom', mesh='torus15', type='mesh')
        body.add('geom', mesh='torus16', type='mesh')
        body.add('geom', mesh='torus17', type='mesh')
        # body.add('geom', mesh='torus12', type='mesh')
        if i == 0:
            body.add('inertial', pos="-0 0 " + str(ypos), mass=0)
            # body.add('joint', name="chain_x", axis="1 0 0")
            # body.add('joint', name="chain_y", axis="0 1 0")
        else:
            body.add('freejoint')
            # body.add('inertial', pos="-0 0 " + str(ypos), mass=m)
    # mjcf_model.actuator.add('velocity', joint="chain_x")
    # mjcf_model.actuator.add('velocity', joint="chain_y")
    mjcf.export_with_assets(mjcf_model, 'xml', out_file_name='chain_' + str(n) + '.xml')

create_n_links(int(args.num_links))
xml_path = "xml/chain_" + str(args.num_links) + ".xml"
physics = mujoco.Physics.from_xml_path(xml_path)
d = []
for pos in physics.named.data.xpos:
    d.append({'mesh': 'link_lab_vhcad.obj', 'pos': pos})
print(d)

model = mujoco_py.load_model_from_path(xml_path)
sim = mujoco_py.MjSim(model)
viewer = mujoco_py.MjViewer(sim)

while True:
    sim.step()
    viewer.render()

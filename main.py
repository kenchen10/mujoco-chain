import mujoco_py
from dm_control import mjcf, mujoco
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-n", "--num_links", help="Specifies number of links in chain", default="10")

args = parser.parse_args()

def create_n_links(n, base_file='base.xml'):
    # Parse from path
    mjcf_model = mjcf.from_path(base_file)
    even_link_quat = "0.707107 0 0 0.707107"
    odd_link_quat = "1 0 0 0"
    body = ""
    ypos = (n * (54.3 - 2 / 2) / 1000 + 4.45 / 1000 + 64.3 / 2000)
    m = 91 / 23
    for i in range(n):
        ypos -= (54.3 - 3/2) / 1000
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
            body.add('inertial', mass='0', pos="-0 0 " + str(ypos))
        else:
            body.add('joint', type='free')
            # body.add('inertial', pos="-0 0 " + str(ypos), mass=m)
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

import mujoco_py
from dm_control import mjcf, mujoco
import argparse
import json
import numpy as np
import time

parser = argparse.ArgumentParser()
parser.add_argument("-n", "--num_links", help="Specifies number of links in chain", default="10")
parser.add_argument("-s", "--scale", help="Specifies scale of geoms", default=".001")
parser.add_argument("-b", "--buffer", help="Specifies buffer", default="20")

args = parser.parse_args()

def create_n_links(n, base_file='base.xml'):
    # Parse from path
    mjcf_model = mjcf.from_path(base_file)
    mjcf_model.option.timestep=str(1e-2)
    even_link_quat = "0.707107 0 0 0.707107"
    odd_link_quat = "1 0 0 0"
    body = ""
    ypos = (n * (54.3 - 2 / 2) / 1000 + 4.45 / 1000 + 64.3 / 2000)
    m = 91 / 23
    assets = mjcf_model.asset
    for j in range(18):
        assets.add('mesh', file=f"meshes/convex_{j}_lab.stl", name=f"torus{j}", scale=args.scale + " " + args.scale + " " + args.scale)
    for i in range(n):
        ypos -= (54.3 - float(args.buffer)) / 1000
        if i % 2 == 0:
            body = mjcf_model.worldbody.add('body', name="body" + str(i), pos="-0 0 " + str(ypos), quat=even_link_quat)
        else:
            body = mjcf_model.worldbody.add('body', name="body" + str(i), pos="-0 0 " + str(ypos), quat=odd_link_quat)
        mass = 0.001
        for j in range(18):
            body.add('geom', mesh=f'torus{j}', type='mesh', mass=mass, condim=3)
        if i == 0:
            body.add('joint', type="slide", name="chain_x", axis="1 0 0")
            body.add('joint', type="slide", name="chain_y", axis="0 1 0")
            # body.add('joint', type="slide", name="chain_z", axis="0 0 1")
        else:
            body.add('freejoint')
    mjcf_model.actuator.add('velocity', joint="chain_x")
    mjcf_model.actuator.add('velocity', joint="chain_y")
    # mjcf_model.actuator.add('velocity', joint="chain_z")
    mjcf.export_with_assets(mjcf_model, 'xml', out_file_name='chain_' + str(n) + '.xml')

create_n_links(int(args.num_links))
xml_path = "xml/chain_" + str(args.num_links) + ".xml"
physics = mujoco.Physics.from_xml_path(xml_path)

with open("json/data.txt", "w+") as file_:
    d = []
    for index, position in enumerate(physics.named.data.xpos):
        d.append({
            'mesh': 'link_lab_vhcad.obj',
            'position': position.tolist(),
            'scale': 0.001,
            'body_id': index,
            'boundary_id': index})
    json.dump(d, file_, indent=2)

model = mujoco_py.load_model_from_path(xml_path)
sim = mujoco_py.MjSim(model)
viewer = mujoco_py.MjViewer(sim)

scale = 2
timestart = time.time()
for i in range(1000):
    # sim.data.ctrl[:] = (scale*np.random.rand(2)-scale/2).tolist()
    sim.data.ctrl[:] = [0, 0]
    sim.forward()
    sim.step()
    viewer.render()
print(time.time() - timestart)

"""Run with blender -b -P tools/preview.py from the workspace root."""
from pathlib import Path
import json,math,xml.etree.ElementTree as E
import bpy
from mathutils import Euler,Vector,Matrix
root=Path.cwd();p=root/'src/c3_cell';world=E.parse(p/'models/workcell.sdf').find('world')
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
def trans(x):return Matrix.Translation(Vector(x[:3]))@Euler(x[3:]).to_matrix().to_4x4()
def coords(n):return list(map(float,(n.text if n is not None else '0 0 0 0 0 0').split()))
q=[-.42,.78,-1.32,0,.54,-.42]
def draw(v,t):
    g=list(v.find('geometry'))[0];T=t@trans(coords(v.find('pose')))
    if g.tag=='box':
        bpy.ops.mesh.primitive_cube_add(size=1);obj=bpy.context.object;obj.dimensions=list(map(float,g.findtext('size').split()))
        bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    elif g.tag=='cylinder':
        bpy.ops.mesh.primitive_cylinder_add(vertices=40,radius=float(g.findtext('radius')),depth=float(g.findtext('length')));obj=bpy.context.object
    else:
        bpy.ops.mesh.primitive_uv_sphere_add(radius=float(g.findtext('radius')));obj=bpy.context.object
    obj.matrix_world=T
    bevel=obj.modifiers.new('machined edges','BEVEL');bevel.width=.0013;bevel.segments=3
    normal=obj.modifiers.new('normals','WEIGHTED_NORMAL')
    rgba=tuple(map(float,v.findtext('material/diffuse').split()))
    key=str(rgba);mat=bpy.data.materials.get(key) or bpy.data.materials.new(key);mat.diffuse_color=rgba;mat.use_nodes=True
    bs=mat.node_tree.nodes.get('Principled BSDF');bs.inputs['Base Color'].default_value=rgba;bs.inputs['Metallic'].default_value=.30;bs.inputs['Roughness'].default_value=.32
    obj.data.materials.append(mat)
for model in world.findall('model'):
    poses={'__model__':trans(coords(model.find('pose')))}
    for l in model.findall('link'):
        name=l.get('name');pos=l.find('pose');parent=pos.get('relative_to','__model__') if pos is not None else '__model__'
        t=poses[parent]@trans(coords(pos))
        if model.get('name')=='arm' and name in [f'l{i}' for i in range(1,7)]:t=t@Matrix.Rotation(q[int(name[1:])-1],4,'Z')
        if name in ['left','right']:t=t@Matrix.Translation((0,.009 if name=='left' else -.009,0))
        poses[name]=t
        for v in l.findall('visual'):draw(v,t)
sc=bpy.context.scene;sc.render.engine='CYCLES';sc.cycles.samples=24;sc.cycles.use_denoising=False
sc.world.color=(.38,.41,.44)
def aim(obj,loc):obj.rotation_euler=(Vector(loc)-obj.location).to_track_quat('-Z','Y').to_euler()
for xyz,power,size in [((0,-1,1.6),75,1),((1,.7,1.4),95,1),((-.8,.2,.9),65,.7)]:
    bpy.ops.object.light_add(type='AREA',location=xyz);a=bpy.context.object;a.data.energy=power;a.data.shape='DISK';a.data.size=size;aim(a,(.1,0,0))
bpy.ops.object.camera_add(location=(.95,-1.,.78));cam=bpy.context.object;aim(cam,(.14,0,.105));sc.camera=cam;cam.data.type='ORTHO';cam.data.ortho_scale=1.06
sc.render.resolution_x=1050;sc.render.resolution_y=630;sc.render.resolution_percentage=100
sc.render.filepath=str(root/'docs/workcell.png');bpy.ops.render.render(write_still=True)

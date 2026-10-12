from pathlib import Path
import sys, json
import numpy as np
import trimesh
from PIL import Image, ImageDraw, ImageFont
import matplotlib
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'outputs/rounded_holder'))
import make_holder as holder
sys.path.insert(0,str(ROOT/'outputs/flat_drum'))
import make_flat_drum as drumlib
out=holder.OUT
parts={'u_frame':holder.frame()}
panels={side:holder.motor_panel(side) for side in (-1,1)}
for p in panels.values():p.visual.face_colors=[35,156,151,145]
parts['u_frame'].visual.face_colors=[68,123,166,65]
refs=ROOT/'outputs/flat_drum'
core=trimesh.load_mesh(refs/'drum_core.stl');core.visual.face_colors=[32,39,47,255]
source=[core]
for i in range(10):
 for name,color in [('background',[29,35,42,255]),('digit',[245,239,212,255])]:
  m=trimesh.load_mesh(refs/f'tile_{i}_{name}.stl');m.visual.face_colors=color
  source.append(drumlib.on_face(m,i,30*np.cos(np.pi/10),20,2.4))
source=trimesh.util.concatenate(source)
modules={side:[panels[side]] for side in (-1,1)}
motors=[];drums=[]
for row,height in enumerate(reversed(holder.AXIS_HEIGHTS)):
 for side in (-1,1):
  displaydigit=1+row*2+(side==1)
  m=source.copy();m.apply_transform(trimesh.transformations.rotation_matrix(-displaydigit*2*np.pi/10,[0,0,1]))
  trans=np.array([[0,0,-1,side*holder.DRUM_CENTER+holder.DRUM_WIDTH/2],[-1,0,0,holder.AXIS_Y],[0,1,0,height],[0,0,0,1]])
  m.apply_transform(trans);drums.append(m);modules[side].append(m)
  motor=holder.box([17.3,42,42],[side*(holder.MOTOR_FACE+11.85),holder.AXIS_Y,height]);motor.visual.face_colors=[85,88,93,255]
  cap=holder.box([3.2,42,42],[side*(holder.MOTOR_FACE+1.6),holder.AXIS_Y,height]);cap.visual.face_colors=[180,184,191,255]
  boss=holder.cylinder(11,2,[side*(holder.MOTOR_FACE-1),holder.AXIS_Y,height],[1,0,0]);boss.visual.face_colors=[172,177,185,255]
  shaft=holder.cylinder(2.5,24,[side*(holder.MOTOR_FACE-holder.SHAFT_LENGTH/2),holder.AXIS_Y,height],[1,0,0]);shaft.visual.face_colors=[190,194,202,255]
  motors.extend([motor,cap,boss,shaft]);modules[side].extend([motor,cap,boss,shaft])

for side in (-1,1):
 for height in holder.AXIS_HEIGHTS:
  for y in (-15.5,15.5):
   for dz in (-15.5,15.5):
    head=holder.motor_screw_head(side,holder.AXIS_Y+y,height+dz);head.visual.face_colors=[195,158,72,255];modules[side].append(head)
retainers=[]
for side in (-1,1):
 for y,z in holder.RETAINING_POINTS:
  screw=holder.cylinder(2.75,3,[side*(holder.MOTOR_FACE+1.5),y,z],[1,0,0]);screw.visual.face_colors=[220,164,53,255];retainers.append(screw)
  insert=holder.subtract(holder.cylinder(holder.INSERT_OD/2,holder.INSERT_LENGTH,[side*(holder.PLATE_INNER-holder.INSERT_LENGTH/2),y,z],[1,0,0]),[holder.cylinder(1.25,5,[side*(holder.PLATE_INNER-holder.INSERT_LENGTH/2),y,z],[1,0,0])]);insert.visual.face_colors=[210,161,62,255];retainers.append(insert)
full=[parts['u_frame']]+modules[-1]+modules[1]+retainers
export_meshes={f'part_{i}':m.copy() for i,m in enumerate(full)}
for mesh in export_meshes.values():
 color=mesh.visual.face_colors[0]
 if color[3]<255:mesh.visual=trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(baseColorFactor=color,alphaMode='BLEND',doubleSided=True,roughnessFactor=.8))
trimesh.Scene(export_meshes).export(out/'assembly_reference.glb')
def render(meshes, filename, camera, title):
 m=trimesh.util.concatenate(meshes);p=m.vertices.copy()
 p -= (p.min(axis=0)+p.max(axis=0))/2
 camera=np.array(camera,dtype=float);camera/=np.linalg.norm(camera)
 reference_up=np.array([0,1,0]) if abs(camera[2])>.999 else np.array([0,0,1])
 right=np.cross(reference_up,camera);right/=np.linalg.norm(right);up=np.cross(camera,right)
 q=p@np.column_stack((right,up,camera));scale=min(950/np.ptp(q[:,0]),860/np.ptp(q[:,1]))
 q[:,0]=q[:,0]*scale+600;q[:,1]=555-q[:,1]*scale
 image=np.full((1050,1200,3),[239,242,246],dtype=np.uint8);zbuffer=np.full(image.shape[:2],-np.inf)
 light=camera+up*.8-right*.6;light/=np.linalg.norm(light)
 # Opaque surfaces establish depth; transparent shell triangles blend back to front.
 alphas=m.visual.face_colors[:,3]/255.
 opaque=np.flatnonzero(alphas>=1.)
 translucent=np.flatnonzero(alphas<1.)
 translucent=translucent[np.argsort(q[m.faces[translucent],2].mean(axis=1))]
 for fi in np.concatenate((opaque,translucent)):
  face=m.faces[fi];alpha=alphas[fi]
  tri=q[face];normal=np.cross(p[face[1]]-p[face[0]],p[face[2]]-p[face[0]])
  norm=np.linalg.norm(normal)
  if norm<1e-10:continue
  normal/=norm
  if normal@camera<=0:
   if alpha>=1.:continue
   normal=-normal
  x0,y0=np.maximum(np.floor(tri[:,:2].min(axis=0)).astype(int),[0,0]);x1,y1=np.minimum(np.ceil(tri[:,:2].max(axis=0)).astype(int),[1199,1049])
  if x1<x0 or y1<y0:continue
  xx,yy=np.meshgrid(np.arange(x0,x1+1)+.5,np.arange(y0,y1+1)+.5);a,b,c=tri
  den=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
  if abs(den)<1e-10:continue
  wa=((b[1]-c[1])*(xx-c[0])+(c[0]-b[0])*(yy-c[1]))/den
  wb=((c[1]-a[1])*(xx-c[0])+(a[0]-c[0])*(yy-c[1]))/den;wc=1-wa-wb
  dep=wa*a[2]+wb*b[2]+wc*c[2];buf=zbuffer[y0:y1+1,x0:x1+1]
  visible=(wa>=-1e-8)&(wb>=-1e-8)&(wc>=-1e-8)&(dep>buf)
  if alpha>=1.:buf[visible]=dep[visible]
  color=m.visual.face_colors[fi,:3]*(.4+.6*max(0,normal@light))
  pixels=image[y0:y1+1,x0:x1+1]
  pixels[visible]=(alpha*color+(1-alpha)*pixels[visible]).astype(np.uint8)
 img=Image.fromarray(image);draw=ImageDraw.Draw(img)
 font=ImageFont.truetype(str(Path(matplotlib.get_data_path())/'fonts/ttf/DejaVuSans-Bold.ttf'),30)
 draw.text((36,26),title,fill='#263347',font=font)
 img.save(out/filename)


render(full,'assembly_preview.png',[.7,-1,.5],'Triangular side fasteners — panels tie the front wall to the base')
render(full,'rear_preview.png',[.9,1,.6],'Two bottom screws and one upper-front screw on each side')
exploded=[parts['u_frame']]+modules[-1]
for mesh in modules[1]:
 copy=mesh.copy();copy.apply_translation([0,80,0]);exploded.append(copy)
render(exploded,'assembly_sequence.png',[1,1,.6],'Slide each assembled side in from the rear, then bolt in place')
render([parts['u_frame'],*panels.values()],'housing_preview.png',[1,1,.65],'Front and base flanges — tall rear posts removed')
# Bench view: motor fasteners accessible before either drum is installed.
bench=[panels[1]]+[m for m in modules[1][1:] if not any(m is d for d in drums)]
render(bench,'bench_mounting.png',[-1,.3,.25],'Bench assembly — install motor screws before fitting drums')

flat=panels[1].copy();flat.visual.face_colors=[45,145,145,255]
flat=holder.bed(flat,trimesh.geometry.align_vectors([1,0,0],[0,0,1]))
render([flat],'flat_plate.png',[.25,-.35,1],'Each motor plate prints flat — 3 mm thick throughout')

render([parts['u_frame']]+[m for i,m in enumerate(retainers) if i%2==1],'insert_locations.png',[1,1,.65],'Three inserts per side — bottom-front, bottom-rear, upper-front')

upright=parts['u_frame'].copy();upright.visual.face_colors=[100,132,157,255]
render([upright],'upright_print.png',[.85,1,.65],'Print upright — open top; supports disabled; 138 mm tall')

render([parts['u_frame']]+[m for i,m in enumerate(retainers) if i%2==1],'fastener_side_view.png',[-1,0,0],'Side view — two bottom inserts, one upper-front insert')

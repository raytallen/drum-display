"""Upright-print housing with triangular side fasteners and flat removable motor plates for the existing 60 x 20 mm numeral drums.
Units: mm. Dependencies: cadquery numpy trimesh manifold3d.
Run: python make_holder.py
"""
from pathlib import Path
import json
import zipfile
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
import cadquery as cq

OUT = Path(__file__).resolve().parent
AXIS_HEIGHTS = (36., 102.)
FRAME_HEIGHT = 138.
WALL_THICKNESS = 3.
CAP_THICKNESS = WALL_THICKNESS
DRUM_WIDTH = 20.
DRUM_RADIUS = 30.
DRUM_GAP = 2.
DRUM_SIDE_CLEARANCE = 4.5
DRUM_CENTER = (DRUM_WIDTH + DRUM_GAP) / 2
MOTOR_FACE = DRUM_WIDTH + DRUM_GAP / 2 + DRUM_SIDE_CLEARANCE + WALL_THICKNESS
SHAFT_LENGTH = 24.
MOTOR_LENGTH = 20.5
MOTOR_BOLT_PITCH = 31.
PANEL_FRONT = -39.5
FRONT_THICKNESS = WALL_THICKNESS
ROTATION_CLEARANCE = 1.5
AXIS_Y = PANEL_FRONT + FRONT_THICKNESS + DRUM_RADIUS + ROTATION_CLEARANCE
ARM_BACK = AXIS_Y + 29.
FRAME_DEPTH = ARM_BACK - PANEL_FRONT
WINDOW_WIDTH, WINDOW_HEIGHT = 17., 15.
WINDOW_CHAMFER = 2.
BEND_INNER_RADIUS = 6.
BEND_OUTER_RADIUS = BEND_INNER_RADIUS + WALL_THICKNESS
SMALL_FILLET = 1.


def box(size, center):
    m = trimesh.creation.box(size); m.apply_translation(center); return m


def cylinder(radius, length, center, axis=(0,0,1), sections=96):
    m = trimesh.creation.cylinder(radius, length, sections=sections)
    m.apply_transform(trimesh.geometry.align_vectors([0,0,1], axis))
    m.apply_translation(center); return m


def subtract(body, cutters):
    return trimesh.boolean.difference([body, *cutters], engine='manifold')


def rounded_shell():
    def profile(half,front,back,radius):
        wire=cq.Workplane('XY').moveTo(-half,back).lineTo(-half,front+radius)
        wire=wire.radiusArc((-half+radius,front),-radius).lineTo(half-radius,front)
        return wire.radiusArc((half,front+radius),-radius).lineTo(half,back).close()

    outer=profile(MOTOR_FACE,PANEL_FRONT,ARM_BACK,BEND_OUTER_RADIUS).extrude(FRAME_HEIGHT).val()
    inside_half=MOTOR_FACE-WALL_THICKNESS
    inner=profile(inside_half,PANEL_FRONT+WALL_THICKNESS,ARM_BACK+5,BEND_INNER_RADIUS)
    inner=inner.extrude(FRAME_HEIGHT+5).translate((0,0,WALL_THICKNESS)).val()
    return outer.cut(inner)


def mesh_from_cad(solid):
    if not solid.isValid():
        raise ValueError('Invalid CAD solid')
    vertices,faces=solid.tessellate(.02,.08)
    mesh=trimesh.Trimesh(vertices=[v.toTuple() for v in vertices],faces=faces,process=True)
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.remove_unreferenced_vertices()
    if not mesh.is_volume:
        raise ValueError('CAD mesh must be a watertight volume')
    return mesh


def cad_box(size, center):
    return cq.Workplane('XY').box(*size).translate(center).val()


PANEL_FRONT_EDGE = -27.4
FIT_CLEARANCE = .3
# Per side: bottom-front, bottom-rear, top-front (Y, Z).
RETAINING_POINTS = ((-22.,9.),(19.5,9.),(-22.,129.))
FRONT_FLANGE_BACK = -17.
BOTTOM_FLANGE_HEIGHT = 14.
# CNC Kitchen TC-M3x3.0: OD 4.6, length 3; manufacturer pilot diameter 4.0.
INSERT_OD = 4.6
INSERT_LENGTH = 3.
INSERT_BORE = 4.
INSERT_BORE_DEPTH = INSERT_LENGTH+1.
PLATE_INNER = MOTOR_FACE-WALL_THICKNESS
LEDGE_INNER = PLATE_INNER-WALL_THICKNESS
INSERT_PAD_INNER = PLATE_INNER-INSERT_BORE_DEPTH
RETAINING_SCREW_LENGTH = 6.


def merge(parts):
    return trimesh.boolean.union(parts,engine='manifold')


def motor_screw_head(side,y,z,diameter=6.,depth=1.5):
    # A 90-degree countersunk head, flush with the panel's inward face.
    head=trimesh.creation.cylinder(1,depth,sections=64)
    head.vertices[:,:2] *= (diameter/2-(head.vertices[:,2]+depth/2))[:,None]
    head.apply_transform(trimesh.geometry.align_vectors([0,0,1],[side,0,0]))
    head.apply_translation([side*(PLATE_INNER+depth/2),y,z])
    return head


def motor_cutters(side):
    cutters=[]
    for z in AXIS_HEIGHTS:
        cutters.append(cylinder(11.3,8,[side*(MOTOR_FACE-WALL_THICKNESS/2),AXIS_Y,z],[1,0,0]))
        for y in (-15.5,15.5):
            for dz in (-15.5,15.5):
                cutters.append(cylinder(1.7,8,[side*(MOTOR_FACE-WALL_THICKNESS/2),AXIS_Y+y,z+dz],[1,0,0]))
                cutters.append(motor_screw_head(side,AXIS_Y+y,z+dz,6.4,1.5))
    return cutters


def insert_pad(side,y,z):
    # One millimeter of added depth, with a 45-degree taper for printing.
    pad=trimesh.creation.cylinder(1,1,sections=96)
    pad.vertices[:,:2] *= (pad.vertices[:,2]+4.1)[:,None]
    pad.apply_transform(trimesh.geometry.align_vectors([0,0,1],[side,0,0]))
    pad.apply_translation([side*(INSERT_PAD_INNER+.5),y,z])
    return subtract(pad,[box([60,4,FRAME_HEIGHT+4],[0,ARM_BACK+2,FRAME_HEIGHT/2])])


def frame_cad():
    body=rounded_shell()
    for side in (-1,1):
        opening=cad_box([8,ARM_BACK-PANEL_FRONT_EDGE+2,FRAME_HEIGHT+2],
                        [side*(PLATE_INNER+4),(ARM_BACK+2+PANEL_FRONT_EDGE)/2,FRAME_HEIGHT/2])
        body=body.cut(opening)
        # Reach the inner front face to fill the slit beside the original curved bend.
        flange_front=PANEL_FRONT+WALL_THICKNESS
        front=cad_box([WALL_THICKNESS,FRONT_FLANGE_BACK-flange_front,FRAME_HEIGHT],
                      [side*(PLATE_INNER-WALL_THICKNESS/2),(FRONT_FLANGE_BACK+flange_front)/2,FRAME_HEIGHT/2])
        bottom=cad_box([WALL_THICKNESS,ARM_BACK-flange_front,BOTTOM_FLANGE_HEIGHT],
                       [side*(PLATE_INNER-WALL_THICKNESS/2),(ARM_BACK+flange_front)/2,BOTTOM_FLANGE_HEIGHT/2])
        body=body.fuse(front,bottom).clean()
    def at(value,target):
        return abs(value-target)<1e-4
    junction=[e for e in body.Edges() if at(e.Center().y,FRONT_FLANGE_BACK)
              and at(e.Center().z,BOTTOM_FLANGE_HEIGHT) and e.BoundingBox().xlen>2.9]
    body=body.fillet(4,junction)
    tips=[e for e in body.Edges() if e.BoundingBox().xlen>2.9 and
          ((at(e.Center().y,FRONT_FLANGE_BACK) and at(e.Center().z,FRAME_HEIGHT)) or
           (at(e.Center().y,ARM_BACK) and at(e.Center().z,BOTTOM_FLANGE_HEIGHT)))]
    body=body.fillet(2,tips)
    edges=[]
    for edge in body.Edges():
        bounds=edge.BoundingBox();center=edge.Center()
        if bounds.zlen<1e-5 and any(at(center.z,z) for z in (0,FRAME_HEIGHT,WALL_THICKNESS)):
            edges.append(edge)
        elif bounds.xlen<1e-5 and any(at(abs(center.x),x) for x in (LEDGE_INNER,PLATE_INNER)):
            # Keep this concave seating shoulder square so it cannot obstruct the panel.
            if at(abs(center.x),PLATE_INNER) and at(center.y,PANEL_FRONT_EDGE):
                continue
            edges.append(edge)
        elif bounds.zlen>FRAME_HEIGHT-8 and at(center.y,PANEL_FRONT_EDGE) and at(abs(center.x),MOTOR_FACE):
            edges.append(edge)
    return body.fillet(.8,edges)


def frame():
    body=mesh_from_cad(frame_cad())
    cutters=[];pads=[]
    for side in (-1,1):
        for y,z in RETAINING_POINTS:
            pads.append(insert_pad(side,y,z))
            cutters.append(cylinder(INSERT_BORE/2,INSERT_BORE_DEPTH,[side*(PLATE_INNER-INSERT_BORE_DEPTH/2),y,z],[1,0,0]))
    body=merge([body,*pads])
    for x in (-DRUM_CENTER,DRUM_CENTER):
        for z in AXIS_HEIGHTS:
            cutters.append(box([WINDOW_WIDTH,FRONT_THICKNESS+2,WINDOW_HEIGHT],
                               [x,PANEL_FRONT+FRONT_THICKNESS/2,z]))
            points=[]
            for y in (PANEL_FRONT-1,PANEL_FRONT+WINDOW_CHAMFER):
                offset=PANEL_FRONT+WINDOW_CHAMFER-y
                w,h=WINDOW_WIDTH+2*offset,WINDOW_HEIGHT+2*offset
                for dx in (-w/2,w/2):
                    for dz in (-h/2,h/2): points.append([x+dx,y,z+dz])
            cutters.append(trimesh.convex.convex_hull(np.array(points)))
    return subtract(body,cutters)


def motor_panel(side):
    front=PANEL_FRONT_EDGE+FIT_CLEARANCE
    bottom,top=FIT_CLEARANCE,FRAME_HEIGHT-FIT_CLEARANCE
    # Broad upper-rear radius, smaller bottom-rear radius, and soft perimeter edges.
    outline=(cq.Workplane('YZ').moveTo(front,bottom).lineTo(ARM_BACK-4,bottom)
             .radiusArc((ARM_BACK,bottom+4),-4).lineTo(ARM_BACK,top-10)
             .radiusArc((ARM_BACK-10,top),-10).lineTo(front,top).close())
    solid=outline.extrude(WALL_THICKNESS).translate((PLATE_INNER,0,0)).val()
    perimeter=[e for e in solid.Edges() if e.BoundingBox().xlen<1e-5]
    solid=solid.fillet(.7,perimeter)
    body=mesh_from_cad(solid)
    if side<0:
        body.apply_transform(np.diag([-1,1,1,1]))
    cutters=motor_cutters(side)
    for y,z in RETAINING_POINTS:
        cutters.append(cylinder(1.7,8,[side*(MOTOR_FACE-WALL_THICKNESS/2),y,z],[1,0,0]))
    return subtract(body,cutters)


def bed(mesh, rotation=None):
    m=mesh.copy()
    if rotation is not None: m.apply_transform(rotation)
    bounds=m.bounds
    m.apply_translation([-np.mean(bounds[:,0]),-np.mean(bounds[:,1]),-bounds[0,2]])
    return m


def write_3mf(groups, path):
    ns='http://schemas.microsoft.com/3dmanufacturing/core/2015/02'
    ET.register_namespace('',ns)
    def sub(parent, tag, **attrs):
        return ET.SubElement(parent,'{'+ns+'}'+tag,{k:str(v) for k,v in attrs.items()})
    root=ET.Element('{'+ns+'}model',{'unit':'millimeter'})
    sub(root,'metadata',name='Title').text=path.stem
    resources=sub(root,'resources'); build=sub(root,'build'); config=ET.Element('config')
    for i,(name,mesh,xy) in enumerate(groups,1):
        obj=sub(resources,'object',id=i,type='model',name=name)
        body=sub(obj,'mesh'); vertices=sub(body,'vertices'); faces=sub(body,'triangles')
        for x,y,z in mesh.vertices: sub(vertices,'vertex',x=f'{x:.7f}',y=f'{y:.7f}',z=f'{z:.7f}')
        for a,b,c in mesh.faces: sub(faces,'triangle',v1=a,v2=b,v3=c)
        sub(build,'item',objectid=i,transform=f'1 0 0 0 1 0 0 0 1 {xy[0]} {xy[1]} 0')
        cfg=ET.SubElement(config,'object',{'id':str(i)})
        ET.SubElement(cfg,'metadata',{'key':'name','value':name})
        ET.SubElement(cfg,'metadata',{'key':'extruder','value':'1'})
    with zipfile.ZipFile(path,'w',zipfile.ZIP_DEFLATED) as z:
        z.writestr('[Content_Types].xml','<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/><Default Extension="config" ContentType="application/xml"/></Types>')
        z.writestr('_rels/.rels','<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" Target="/3D/3dmodel.model"/></Relationships>')
        z.writestr('3D/3dmodel.model',ET.tostring(root,encoding='utf-8',xml_declaration=True))
        z.writestr('Metadata/model_settings.config',ET.tostring(config,encoding='utf-8',xml_declaration=True))


def build():
    OUT.mkdir(parents=True,exist_ok=True)
    fixed=frame();panels={side:motor_panel(side) for side in (-1,1)}
    parts={'housing':fixed,'left_motor_panel':panels[-1],'right_motor_panel':panels[1]}
    report={'parts':{},'clearances_mm':{'drum_to_panel':DRUM_SIDE_CLEARANCE,'between_drums':2,'panel_edge_clearance':FIT_CLEARANCE,'shaft_engagement_nominal':SHAFT_LENGTH-WALL_THICKNESS-DRUM_SIDE_CLEARANCE,'flush_motor_head_to_drum':DRUM_SIDE_CLEARANCE},'nominal_wall_mm':3,'assembly_width_with_motors_mm':2*(MOTOR_FACE+MOTOR_LENGTH),'housing_width_mm':2*MOTOR_FACE,'assembly_path_samples':[]}
    plate=[]
    for name,part in parts.items():
        if not part.is_volume or len(part.split())!=1:raise ValueError(name+' must be one solid')
        rotation=None if name=='housing' else trimesh.geometry.align_vectors([-1,0,0] if name=='right_motor_panel' else [1,0,0],[0,0,1])
        printable=bed(part,rotation)
        printable.export(OUT/(name+'.stl'))
        write_3mf([(name,printable,[110,110])],OUT/(name+'.3mf'))
        arranged=printable.copy()
        if name!='housing':arranged.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[0,0,1]))
        plate.append((name,arranged,{'housing':[45,128],'left_motor_panel':[114,128],'right_motor_panel':[181,128]}[name]))
        report['parts'][name]={'watertight':bool(part.is_watertight),'components':len(part.split()),'print_extents_mm':printable.extents.tolist()}
    write_3mf(plate,OUT/'complete_holder.3mf')
    def collision(a,b,label):
        overlap=trimesh.boolean.intersection([a,b],engine='manifold')
        if not overlap.is_empty and abs(overlap.volume)>.005:raise ValueError(label+' collision '+str(overlap.volume))
    for side,panel in panels.items():
        collision(panel,fixed,'Panel seated')
        # Conservative rotating cylinders include every possible drum angle.
        moving=[panel]
        for height in AXIS_HEIGHTS:
            drum=cylinder(30,20,[side*11,AXIS_Y,height],[1,0,0],180)
            motor=box([20.5,42,42],[side*(MOTOR_FACE+10.25),AXIS_Y,height])
            collision(drum,fixed,'Drum seated')
            collision(drum,panel,'Drum/panel')
            moving.extend([drum,motor,cylinder(11,2,[side*(MOTOR_FACE-1),AXIS_Y,height],[1,0,0])])
            # Flush countersunk heads must clear the front flange during insertion.
            for y in (-15.5,15.5):
                for dz in (-15.5,15.5):
                    head=motor_screw_head(side,AXIS_Y+y,height+dz)
                    collision(drum,head,'Drum/bolt head');moving.append(head)
        assembly=trimesh.util.concatenate(moving)
        for offset in (0,.2,1,3,6,12,20,30,45,60,80):
            shifted=assembly.copy();shifted.apply_translation([0,offset,0])
            collision(shifted,fixed,'Rear insertion')
            collision(shifted,panels[-side],'Opposite panel')
            for height in AXIS_HEIGHTS:
                opposite=cylinder(30,20,[-side*11,AXIS_Y,height],[1,0,0],180)
                collision(shifted,opposite,'Opposite drum')
        report['assembly_path_samples'].append({'side':side,'rear_translation_mm':[0,.2,1,3,6,12,20,30,45,60,80],'intersections':0})
    obstacles=[fixed,*panels.values()]
    for side in (-1,1):
        for z in AXIS_HEIGHTS:
            obstacles.extend([box([20.5,42,42],[side*(MOTOR_FACE+10.25),AXIS_Y,z]),
                              cylinder(30,20,[side*11,AXIS_Y,z],[1,0,0],180)])
    def clear(mesh,label):
        for obstacle in obstacles:
            collision(mesh,obstacle,label)
    for side in (-1,1):
        for y,z in RETAINING_POINTS:
            clear(cylinder(5,50,[side*56,y,z],[1,0,0]),'Straight external screwdriver')
            clear(cylinder(1.5,RETAINING_SCREW_LENGTH,[side*(MOTOR_FACE-RETAINING_SCREW_LENGTH/2),y,z],[1,0,0]),'Retaining screw shank')
            # Install inserts while panels and motors are off, approaching from outside.
            collision(cylinder(5,60,[side*(PLATE_INNER+30.1),y,z],[1,0,0]),fixed,'Heat-set tool approach')
    report['retaining_fastener_access']={'external_driver_radius_mm':5,'heat_set_tool_radius_mm':5,'installation_from':'outside, before fitting panels'}
    report['heat_set_inserts']={'model':'CNC Kitchen TC-M3x3.0','quantity':6,'thread':'M3','outside_diameter_mm':INSERT_OD,'length_mm':INSERT_LENGTH,'pilot_diameter_mm':INSERT_BORE,'pilot_depth_mm':INSERT_BORE_DEPTH,'local_total_pad_depth_mm':INSERT_BORE_DEPTH,'retaining_screw_length_mm':RETAINING_SCREW_LENGTH}
    report['print_orientation']='Housing upright on its bottom wall; motor plates flat'
    report['support_requirement']='None: open top, front flange and base flange, 45-degree insert pads; windows bridge 17 mm'
    report['roof']='removed'
    report['side_retaining_points_yz_mm']=RETAINING_POINTS
    report['rear_posts']='removed'
    report['flanges']={'thickness_mm':3,'bottom_height_mm':BOTTOM_FLANGE_HEIGHT,'front_flange_back_y_mm':FRONT_FLANGE_BACK}
    report['motor_fasteners']='16 x M3 x 5 mm 90-degree countersunk screws, seated flush'
    report['motor_countersink']={'diameter_mm':6.4,'depth_mm':1.5}
    report['largest_intended_bridge_mm']=WINDOW_WIDTH
    report['rounding_mm']={'panel_upper_rear':10,'panel_lower_rear':4,'panel_perimeter':.7,'housing_exposed_edges':.8,'flange_tips':2,'flange_junction':4}
    report['front_flange_gap']='filled before edge filleting'
    report['fallbacks']=[]
    report['migration_logic']=[]
    (OUT/'geometry_checks.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    return parts

if __name__=='__main__':build()

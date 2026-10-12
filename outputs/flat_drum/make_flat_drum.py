"""Flat numeral tiles and matching drum; millimeters.
Dependencies: numpy matplotlib shapely trimesh manifold3d mapbox-earcut pillow
Run: python make_flat_drum.py --font /path/to/JetBrainsMono-Bold.ttf
"""
import argparse
from pathlib import Path
import zipfile
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import textpath, font_manager
from shapely import geometry
import trimesh

CORE_COLOR = [49, 104, 139, 255]
TILE_COLOR = [49, 104, 139, 255]
DIGIT_COLOR = [246, 242, 221, 255]

def box(size, center):
    mesh = trimesh.creation.box(size)
    mesh.apply_translation(center)
    return mesh

def on_face(mesh, index, apothem, width, thickness):
    angle = index * 2 * np.pi / 10
    # Local tile X reads right along the shaft, Y reads up tangentially.
    matrix = np.array([[0, -np.sin(angle), np.cos(angle), (apothem-thickness)*np.cos(angle)],
                       [0, np.cos(angle), np.sin(angle), (apothem-thickness)*np.sin(angle)],
                       [-1, 0, 0, width/2], [0, 0, 0, 1]])
    result = mesh.copy(); result.apply_transform(matrix)
    return result

def check(mesh, label):
    if not mesh.is_watertight or not mesh.is_volume:
        raise ValueError(f'{label} is not a closed volume')

def write_3mf(groups, destination):
    ns = 'http://schemas.microsoft.com/3dmanufacturing/core/2015/02'
    ET.register_namespace('', ns)
    def add(parent, tag, **attrs):
        return ET.SubElement(parent, '{'+ns+'}'+tag, {k:str(v) for k,v in attrs.items()})
    root = ET.Element('{'+ns+'}model', {'unit':'millimeter', '{http://www.w3.org/XML/1998/namespace}lang':'en-US'})
    add(root, 'metadata', name='Title').text = 'Flat tile digit drum — JetBrains Mono Bold'
    resources = add(root, 'resources')
    materials = add(resources, 'basematerials', id=1)
    for name, color in [('Drum', CORE_COLOR), ('Tile background', TILE_COLOR), ('Digits', DIGIT_COLOR)]:
        add(materials, 'base', name=name, displaycolor='#'+''.join(f'{x:02X}' for x in color))
    build = add(root, 'build')
    config = ET.Element('config')
    next_id = 2
    for name, parts, translation in groups:
        ids = []
        for part_name, mesh, color_index, extruder in parts:
            oid = next_id; next_id += 1
            ids.append((oid, part_name, extruder))
            obj = add(resources, 'object', id=oid, type='model', name=part_name, pid=1, pindex=color_index)
            shape = add(obj, 'mesh'); vertices = add(shape, 'vertices'); triangles = add(shape, 'triangles')
            for x,y,z in mesh.vertices:
                add(vertices, 'vertex', x=f'{x:.8f}', y=f'{y:.8f}', z=f'{z:.8f}')
            for a,b,c in mesh.faces:
                add(triangles, 'triangle', v1=a, v2=b, v3=c)
        assembly_id = next_id; next_id += 1
        obj = add(resources, 'object', id=assembly_id, type='model', name=name)
        components = add(obj, 'components')
        for oid, _, _ in ids:
            add(components, 'component', objectid=oid)
        transform = '1 0 0 0 1 0 0 0 1 '+' '.join(str(x) for x in translation)
        add(build, 'item', objectid=assembly_id, transform=transform)
        settings = ET.SubElement(config, 'object', {'id':str(assembly_id)})
        ET.SubElement(settings, 'metadata', {'key':'name', 'value':name})
        ET.SubElement(settings, 'metadata', {'key':'extruder', 'value':'1'})
        for oid, part_name, extruder in ids:
            part = ET.SubElement(settings, 'part', {'id':str(oid), 'subtype':'normal_part'})
            ET.SubElement(part, 'metadata', {'key':'name', 'value':part_name})
            ET.SubElement(part, 'metadata', {'key':'extruder', 'value':str(extruder)})
    with zipfile.ZipFile(destination, 'w', zipfile.ZIP_DEFLATED) as archive:
        archive.writestr('[Content_Types].xml', '<?xml version="1.0"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/><Default Extension="config" ContentType="application/xml"/></Types>')
        archive.writestr('_rels/.rels', '<?xml version="1.0"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
        archive.writestr('3D/3dmodel.model', ET.tostring(root, encoding='utf-8', xml_declaration=True))
        archive.writestr('Metadata/model_settings.config', ET.tostring(config, encoding='utf-8', xml_declaration=True))


def generate(args):
    out = args.out; out.mkdir(parents=True, exist_ok=True)
    radius, width, bore = args.diameter/2, args.width, args.bore
    apothem = radius*np.cos(np.pi/10)
    tile_width, tile_height, thickness, depth = width-3, 16, 2.4, .8
    gap = .15
    if tile_width < 13 or 2*radius*np.sin(np.pi/10) < tile_height+2*gap+1:
        raise ValueError('Drum too small for the 13 mm digits and face tiles.')
    if bore <= 0 or bore/2 >= apothem-thickness-2:
        raise ValueError('Bore must leave at least 2 mm behind the pockets.')
    angles = np.arange(10)*2*np.pi/10 + np.pi/10
    polygon = geometry.Polygon(np.column_stack((radius*np.cos(angles), radius*np.sin(angles))))
    core = trimesh.creation.extrude_polygon(polygon, width, engine='earcut')
    hole_profile = [[0,-1],[bore/2+.3,-1],[bore/2+.3,0],[bore/2,.3],
                    [bore/2,width-.3],[bore/2+.3,width],[bore/2+.3,width+1],[0,width+1],[0,-1]]
    bore_cut = trimesh.creation.revolve(hole_profile, sections=120); bore_cut.fix_normals()
    # Extra .3 mm on the outside makes each pocket cutter pass completely through the face.
    pocket = box([tile_width+2*gap,tile_height+2*gap,thickness+.3], [0,0,(thickness+.3)/2])
    pockets = [on_face(pocket, i, apothem, width, thickness) for i in range(10)]
    core = trimesh.boolean.difference([core, bore_cut, *pockets], engine='manifold')
    check(core, 'Drum'); core.visual.face_colors = CORE_COLOR
    core.export(out/'drum_core.stl')
    font = font_manager.FontProperties(fname=str(args.font))
    if font.get_name() != 'JetBrains Mono':
        raise ValueError('Use a JetBrains Mono font file.')
    groups = [('Drum core', [('Drum core', core, 0, 1)], [40,40,0])]
    assembled = [core]
    for digit in range(10):
        path = textpath.TextPath((0,0), str(digit), size=100, prop=font)
        bounds = path.get_extents(); scale = 13/bounds.height
        glyph = geometry.Polygon()
        for ring in path.to_polygons():
            points = (ring-[bounds.x0+bounds.width/2,bounds.y0+bounds.height/2])*scale
            glyph = glyph.symmetric_difference(geometry.Polygon(points))
        if glyph.bounds[2]-glyph.bounds[0] > tile_width-2:
            raise ValueError('Tile is too narrow for the digit.')
        polygons = list(glyph.geoms) if glyph.geom_type == 'MultiPolygon' else [glyph]
        ink = trimesh.util.concatenate([trimesh.creation.extrude_polygon(p, depth, engine='earcut') for p in polygons])
        ink.apply_translation([0,0,thickness-depth]); ink.visual.face_colors = DIGIT_COLOR
        blank = box([tile_width,tile_height,thickness], [0,0,thickness/2])
        background = trimesh.boolean.difference([blank, ink], engine='manifold')
        background.visual.face_colors = TILE_COLOR
        check(ink, f'Digit {digit}'); check(background, f'Tile {digit}')
        combined = trimesh.boolean.union([background,ink], engine='manifold')
        if not np.isclose(combined.volume, blank.volume, atol=.002):
            raise ValueError('Tile and digit do not fill the tile volume')
        background.export(out/f'tile_{digit}_background.stl'); ink.export(out/f'tile_{digit}_digit.stl')
        parts = [(f'Tile {digit} background',background,1,1),(f'Digit {digit}',ink,2,2)]
        groups.append((f'Tile {digit}', parts, [88+(digit%4)*23,17+(digit//4)*24,0]))
        assembled.extend([on_face(background,digit,apothem,width,thickness), on_face(ink,digit,apothem,width,thickness)])
    write_3mf(groups, out/'flat_digit_drum.3mf')
    scene = trimesh.util.concatenate(assembled)
    preview(scene, out/'assembled_preview.png')
    # Show the print layout from overhead with exact glyph contours.
    from matplotlib import pyplot
    fig, ax = pyplot.subplots(figsize=(12,6))
    for _, parts, offset in groups:
        for _, mesh, color_index, _ in parts:
            if color_index == 2:
                top = mesh.triangles[np.where(mesh.face_normals[:,2]>.99)[0]]
                from matplotlib.collections import PolyCollection
                ax.add_collection(PolyCollection(top[:,:,:2]+offset[:2], facecolors='#f6f2dd', edgecolors='none'))
            elif color_index == 1:
                ax.add_patch(pyplot.Rectangle((offset[0]-tile_width/2,offset[1]-tile_height/2),tile_width,tile_height,color='#31688b'))
            else:
                ax.add_patch(pyplot.Polygon(np.array(polygon.exterior.coords)+offset[:2],color='#31688b'))
                ax.add_patch(pyplot.Circle(offset[:2],bore/2,color='#f0f2f5'))
    ax.set_aspect('equal'); ax.set_xlim(3,170); ax.set_ylim(2,83); ax.axis('off')
    fig.patch.set_facecolor('#f0f2f5'); fig.tight_layout(); fig.savefig(out/'print_layout.png',dpi=160); pyplot.close(fig)
    print(f'Validated 21 closed meshes; 10 two-part tiles. Tile size: {tile_width} x {tile_height} x {thickness} mm.')
    print(f'Pocket clearance per edge: {gap} mm. Drum corner diameter: {2*radius} mm. Across flats: {2*apothem:.3f} mm.')

def preview(drum, destination):
    from PIL import Image
    p = drum.vertices
    shown = np.column_stack((-p[:, 2], -p[:, 0], p[:, 1]))
    shown -= (shown.min(axis=0) + shown.max(axis=0)) / 2
    azimuth, elevation = np.deg2rad([-58, 18])
    camera = np.array([np.cos(elevation)*np.cos(azimuth), np.cos(elevation)*np.sin(azimuth), np.sin(elevation)])
    right = np.cross([0, 0, 1], camera); right /= np.linalg.norm(right)
    up = np.cross(camera, right)
    projected = shown @ np.column_stack((right, up, camera))
    scale = 720 / np.ptp(projected[:, 1])
    projected[:, 0] = projected[:, 0] * scale + 600
    projected[:, 1] = 450 - projected[:, 1] * scale
    pixels = np.full((900, 1200, 3), [240, 242, 245], dtype=np.uint8)
    zbuffer = np.full((900, 1200), -np.inf)
    light = camera + up * .7 - right * .4; light /= np.linalg.norm(light)
    for face_index, face in enumerate(drum.faces):
        triangle = projected[face]
        normal = np.cross(shown[face[1]] - shown[face[0]], shown[face[2]] - shown[face[0]])
        normal /= np.linalg.norm(normal)
        if normal @ camera <= 0:
            continue
        x0, y0 = np.maximum(np.floor(triangle[:, :2].min(axis=0)).astype(int), [0, 0])
        x1, y1 = np.minimum(np.ceil(triangle[:, :2].max(axis=0)).astype(int), [1199, 899])
        if x1 < x0 or y1 < y0:
            continue
        xx, yy = np.meshgrid(np.arange(x0, x1+1)+.5, np.arange(y0, y1+1)+.5)
        a, b, c = triangle
        denominator = (b[1]-c[1])*(a[0]-c[0]) + (c[0]-b[0])*(a[1]-c[1])
        if abs(denominator) < 1e-10:
            continue
        wa = ((b[1]-c[1])*(xx-c[0])+(c[0]-b[0])*(yy-c[1]))/denominator
        wb = ((c[1]-a[1])*(xx-c[0])+(a[0]-c[0])*(yy-c[1]))/denominator
        wc = 1-wa-wb
        depth = wa*a[2]+wb*b[2]+wc*c[2]
        target = zbuffer[y0:y1+1, x0:x1+1]
        visible = (wa >= -1e-8)&(wb >= -1e-8)&(wc >= -1e-8)&(depth > target)
        target[visible] = depth[visible]
        color = drum.visual.face_colors[face_index, :3]*(.35+.65*max(0, normal @ light))
        pixels[y0:y1+1, x0:x1+1][visible] = color.astype(np.uint8)
    Image.fromarray(pixels).save(destination)



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--diameter',type=float,default=60)
    parser.add_argument('--width',type=float,default=20)
    parser.add_argument('--bore',type=float,default=5.2)
    parser.add_argument('--font',type=Path,required=True)
    parser.add_argument('--out',type=Path,default=Path(__file__).resolve().parent)
    generate(parser.parse_args())

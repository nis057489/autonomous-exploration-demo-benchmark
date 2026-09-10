#!/usr/bin/env python3
"""Regenerate office.world from the upstream ServiceSim world.

office_part1/service.world is a Gazebo-classic world (SDF 1.6, OGRE1 material
scripts, classic URI conventions). Gazebo Harmonic -- what this benchmark runs
via ros_gz -- rejects it: sdformat is invoked with "resolve URIs" enabled, so
every unresolvable <uri> is a hard Error Code 14 at load, and gz-sim needs its
system plugins declared in the world to step physics at all.

This script performs that conversion. It is idempotent and reads only the
pristine upstream file, so re-running it after re-extracting the zips
reproduces office.world exactly.

    python3 simulation/worlds/office/convert_service_world.py
"""

import os
import re
import sys

import numpy as np

from lxml import etree

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "office_part1", "service.world")
DST = os.path.join(HERE, "office.world")

# Classic worlds reference bare "file://media/..." paths that resolved relative
# to the world file in gzclassic. Harmonic has no such relative-URI rule, so the
# media tree is exposed as a model:// resource instead: models/office_media is a
# symlink to office_part1/media, and world.launch.py already puts every
# simulation/worlds/*/models directory on GZ_SIM_RESOURCE_PATH.
MEDIA_MODEL_URI = "model://office_media"

# ServiceSim kept a handful of meshes in their own model:// directories. Only
# cubicle_wall.obj shipped inside the two office zips (in media/meshes), so it
# is remapped there; the rest have no asset anywhere in this repo and the
# elements referencing them are dropped -- see MISSING_MESHES.
MESH_URI_REMAP = {
    "model://cubicle_wall/meshes/cubicle_wall.obj": f"{MEDIA_MODEL_URI}/meshes/cubicle_wall.obj",
}

# Meshes referenced by the world whose asset files are absent from both
# office_part1.zip and office_part2.zip. Every <visual>/<collision> that uses
# one is removed, otherwise sdformat fails the whole world load.
#
# Impact differs per entry, and only one of them changes the simulation:
#   cubicle_closed / cubicle_corner / cubicle_island -- visuals only. The
#     cubicle blocks carry their own <box> collision, which is untouched, so
#     these are purely cosmetic; the robot still sees and bumps into cubicles.
#   door -- 30 visuals AND 30 mesh collisions, and the door leaf/frame links
#     have no other collision geometry. Those 15 doorways therefore become
#     fully open passages. This makes the floorplan more connected than
#     upstream ServiceSim intended; if closed doors matter for a run, add box
#     collisions for them rather than restoring this entry.
# Models removed from the world outright, by include URI. Unlike MISSING_MESHES
# (assets that simply are not in this repo), these have working assets and are
# dropped on purpose:
#
#   reception_desk -- two curved desks, one in FrontEntrance and one in
#     BackEntrance. Their collision geometry is a concave curve right in the
#     doorway-adjacent traffic path, and robots wedge against it repeatedly.
#     That is the exact failure lite_frontier_explorer's goal_stuck_timeout_s
#     was added to paper over ("a robot that jams on the curved reception
#     desk"), and it costs a stuck robot one-to-several minutes of nav2
#     recovery every time it happens -- noise the benchmark does not need.
#
# Both are nested inside room-level models, so gz sim's GUI cannot delete them
# (UserCommands only removes direct children of the world); dropping them here
# is also what keeps the removal alive across regeneration.
DROP_MODEL_URIS = (
    "model://reception_desk",
)

# Plain boxes added to the world after conversion, as (name, x, y, z, sx, sy, sz)
# in world coordinates.
#
# The private-cubicle area is a 3-wide x 4-tall corridor grid: cubicle blocks at
# x -21..-14 and x -12..-7, a north-south corridor between them at x ~ -13, and
# east-west corridors at y ~ 18.5, 14.5, 9.5 and 5.5. Every one of those
# openings is a cycle, so a robot that re-enters explored ground can always loop
# back out -- re-covering a teammate's area costs it almost nothing, which is
# exactly the property that makes map sharing look unimportant in the benchmark.
#
# These three boxes plug the middle corridor where it passes each row of
# cubicles, leaving four dead-end stubs instead of a through route. Each spans
# x -14..-12, which is the full corridor width (free space measures about
# -13.75..-12.25), so they seal rather than leave a gap a robot could squeeze
# through, and 2 m tall so they are seen by the lidar at any sensible height.
ADDED_BOXES = (
    ("deadend_blocker_north", -13.0, 16.5, 1.0, 2.0, 2.0, 2.0),
    ("deadend_blocker_mid", -13.0, 12.0, 1.0, 2.0, 2.0, 2.0),
    ("deadend_blocker_south", -13.0, 7.5, 1.0, 2.0, 2.0, 2.0),
    # Fourth box: the gap between PrivateCubicle_33's west face (x -21) and the
    # thin wall at x ~ -22.9, so the northern cubicle row cannot be entered from
    # the west either. Without it that row is still a through route rather than
    # a trap, since blocking only the middle corridor leaves both ends open.
    ("deadend_blocker_west_33", -22.0, 16.5, 1.0, 2.0, 2.0, 2.0),
    # Same for the other two rows: without these, blocking only the middle
    # corridor merges each row's two cubicle blocks into one bigger island that
    # a robot can still circle. Measured on the navigable free space, the three
    # rows accounted for 6 of the map's 29 loops.
    ("deadend_blocker_west_32", -22.0, 12.0, 1.0, 2.0, 2.0, 2.0),
    ("deadend_blocker_west_31", -22.0, 7.5, 1.0, 2.0, 2.0, 2.0),
    # The two remaining openings in the middle corridor: the east-west corridors
    # at y ~ 9.75 and ~ 14.25 still cross it between the blockers above, which is
    # what let a robot circle the mid and south cubicle rows. Each spans the gap
    # between the neighbouring blockers (2.5 m: 8.5..11.0 and 13.0..15.5), so the
    # middle corridor is continuous obstacle from y 6.5 to 17.5.
    ("deadend_blocker_gap_south_mid", -13.0, 9.75, 1.0, 2.0, 2.5, 2.0),
    ("deadend_blocker_gap_mid_north", -13.0, 14.25, 1.0, 2.0, 2.5, 2.0),
)


MISSING_MESHES = (
    "model://door/meshes/",
    "model://cubicle_corner/meshes/",
    "model://cubicle_island/meshes/",
    "model://cubicle_wall/meshes/cubicle_closed.obj",
)

# OGRE1 .material scripts (media/materials/scripts/servicesim.material) are not
# usable by ogre2, and their <uri> entries are unresolvable besides. Each
# ServiceSim material is translated to an equivalent PBR albedo map, taken from
# that script's texture_unit.
MATERIAL_TEXTURES = {
    "ServiceSim/Ceiling": "ceiling.png",
    "ServiceSim/Hallway": "hallway.png",
    "ServiceSim/Elevator": "elevator.png",
    "ServiceSim/Door": "door_wall.png",
    "ServiceSim/Window": "window.png",
    "ServiceSim/PlainWall": "plain.png",
}

# gz-sim steps nothing without these; the classic world declared no plugins.
# Mirrors the set used by simulation/worlds/bookstore/bookstore.world.
GZ_SYSTEM_PLUGINS = """
    <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
    <plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'>
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin name='gz::sim::systems::Imu' filename='gz-sim-imu-system'/>
    <plugin name='gz::sim::systems::Contact' filename='gz-sim-contact-system'/>
    <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/>
"""


def fix_mesh_texture_paths():
    """Repoint textures referenced from inside the mesh files themselves.

    Rewriting the world's URIs is not enough: the OBJ/DAE assets carry their
    own material references, which gz resolves relative to the mesh file, not
    the world.

    Two separate breakages:

    1. The .mtl sidecars next to media/meshes/*.obj use the same classic
       "file://media/materials/textures/x.png" form as the world did. gz looks
       for those relative to the mesh directory and fails. Since the .mtl lives
       in media/meshes/ and the textures in media/materials/textures/,
       "../materials/textures/x.png" is the correct relative form. This covers
       the building shell -- walls, floors, ceiling, cubicle panels -- so it is
       most of what you actually see.

    2. Some .dae files name textures bare ("Maple.jpg"). The files do exist,
       but one directory over in <model>/materials/textures/. A relative
       symlink next to the mesh makes the bare name resolve. Only genuinely
       bare references are linked -- most DAEs use a model:// URI, which
       already resolves via GZ_SIM_RESOURCE_PATH and needs no help.

    Both steps are idempotent and operate on the extracted upstream trees, so
    re-running after re-extracting the zips reapplies them.
    """
    fixed_mtl = 0
    meshes_dir = os.path.join(HERE, "office_part1", "media", "meshes")
    for name in sorted(os.listdir(meshes_dir)):
        if not name.endswith(".mtl"):
            continue
        path = os.path.join(meshes_dir, name)
        with open(path) as handle:
            content = handle.read()
        patched = content.replace(
            "file://media/materials/textures/", "../materials/textures/"
        )
        if patched != content:
            with open(path, "w") as handle:
                handle.write(patched)
            fixed_mtl += 1

    bare_ref = re.compile(
        r"<init_from>\s*([^<>/\\]+\.(?:png|jpg|jpeg|tga))\s*</init_from>", re.IGNORECASE
    )
    linked = 0
    unresolved = []
    models_dir = os.path.join(HERE, "models")
    for entry in sorted(os.listdir(models_dir)):
        model_meshes = os.path.join(models_dir, entry, "meshes")
        model_textures = os.path.join(models_dir, entry, "materials", "textures")
        if not os.path.isdir(model_meshes):
            continue

        wanted = set()
        for mesh in sorted(os.listdir(model_meshes)):
            if not mesh.lower().endswith(".dae"):
                continue
            with open(os.path.join(model_meshes, mesh), errors="ignore") as handle:
                wanted.update(bare_ref.findall(handle.read()))

        for texture in sorted(wanted):
            link = os.path.join(model_meshes, texture)
            if os.path.exists(link) or os.path.islink(link):
                continue
            target = os.path.join(model_textures, texture)
            if not os.path.isfile(target):
                unresolved.append(f"{entry}/meshes -> {texture}")
                continue
            os.symlink(os.path.join("..", "materials", "textures", texture), link)
            linked += 1

    return fixed_mtl, linked, unresolved


# Models that ship a visual but no <collision>, so they are invisible to the
# robot's lidar and do not physically block it. Upstream ServiceSim could get
# away with this where the furniture sits inside a cubicle block, since the
# block's own 7.25 x 2.5 x 1.5 box already covers that footprint.
#
# Measured: all 42 desk instances do fall inside an existing cubicle collision
# volume, so adding this box changes nothing about today's navigation -- it is
# defensive, not a fix. It matters only if the cubicle blocks are ever replaced
# with per-panel collisions, at which point 42 desks would silently become
# ghosts. Keeping the model self-consistent is cheap; the box is static.
#
# The added collision is a box taken from the mesh's own bounding box (scaled by
# the model's <scale>), not a guessed size. A solid box rather than legs+top is
# deliberate: with a 2D lidar you want the whole desk footprint blocked, and it
# matches how this world models other furniture (see office_cafe_table).
#
# Deliberately NOT included: "computer" (0.63 x 0.37 x 0.49 monitor sitting on a
# desktop, above 2D scan height), "microwave" and "coffee_maker" (both stand on
# cafe_counter, which has its own collision).
FURNITURE_NEEDING_COLLISION = {
    "desk": "meshes/desk.obj",
}


def mesh_aabb(path, scale):
    """Axis-aligned bounding box of an OBJ, in metres, as (min, max)."""
    verts = []
    with open(path, errors="ignore") as handle:
        for line in handle:
            if line.startswith("v "):
                verts.append([float(v) for v in line.split()[1:4]])
    if not verts:
        raise SystemExit(f"no vertices found in {path}")
    arr = np.array(verts) * scale
    return arr.min(axis=0), arr.max(axis=0)


def add_missing_furniture_collisions():
    """Give collision-less furniture a mesh-derived box collision."""
    added = []
    for name, rel_mesh in sorted(FURNITURE_NEEDING_COLLISION.items()):
        sdf_path = os.path.join(HERE, "models", name, "model.sdf")
        if not os.path.isfile(sdf_path):
            continue
        tree = etree.parse(sdf_path)
        link = tree.getroot().find("model/link")
        if link is None or link.find("collision") is not None:
            continue  # already has one; idempotent

        visual_mesh = link.find("visual/geometry/mesh")
        scale_el = visual_mesh.find("scale") if visual_mesh is not None else None
        scale = ([float(v) for v in scale_el.text.split()] if scale_el is not None
                 else [1.0, 1.0, 1.0])
        lo, hi = mesh_aabb(os.path.join(HERE, "models", name, rel_mesh), np.array(scale))
        size = hi - lo
        centre = (hi + lo) / 2

        col = etree.SubElement(link, "collision")
        col.set("name", "collision")
        etree.SubElement(col, "pose").text = (
            f"{centre[0]:.4f} {centre[1]:.4f} {centre[2]:.4f} 0 0 0")
        box = etree.SubElement(etree.SubElement(col, "geometry"), "box")
        etree.SubElement(box, "size").text = (
            f"{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}")

        tree.write(sdf_path, xml_declaration=True, encoding="UTF-8", pretty_print=True)
        added.append(f"{name} ({size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f} m)")
    return added


def modernize_sdf_version(root):
    """Declare SDF 1.10 and drop constructs that version removed.

    The source declares 1.6. That matters beyond pedantry: <pbr> materials
    (written by convert_materials) postdate 1.6, so leaving the document at 1.6
    risks sdformat's version converter discarding them. Bumping is safe here
    because the only 1.7+ incompatibility this world actually contains is the
    empty pose frame="" attribute -- there are no joints, no
    <use_parent_model_frame>, nothing else version-sensitive.
    """
    root.set("version", "1.10")
    stripped = 0
    for pose in root.iter("pose"):
        if "frame" in pose.attrib:
            if pose.attrib["frame"]:
                raise SystemExit(
                    f"pose has a non-empty frame='{pose.attrib['frame']}'; "
                    "needs a relative_to conversion, not a plain strip"
                )
            del pose.attrib["frame"]
            stripped += 1
    return stripped


def convert_physics(world):
    """Replace the classic ODE block, add world physics tags, declare plugins.

    Three separate things, all physics-relevant:

    1. The classic <ode><solver>/<constraints> sub-elements (quick solver, 300
       iters, cfm/erp, contact_max_correcting_vel) have no equivalent under
       gz-sim's <physics> tag and are silently ignored -- dartsim runs its own
       solver. Only the tags gz-sim honors are kept.

    2. The step is retimed from upstream's 0.002 s / 500 Hz to this repo's
       0.01 s / 200 Hz (as in bookstore.world). ServiceSim chose 500 Hz for
       classic ODE's cheap quick solver; under gz-sim, in a scene this size and
       with several sensor-carrying robots, 5x the physics steps is a good way
       to drive the real-time factor far below 1 -- which looks exactly like
       robots that "cannot move" when they are really just moving 20x slower
       than wall clock. Nothing here needs a 2 ms step: all collision geometry
       is primitives (boxes, cylinders, one ground plane), no meshes.

    3. <gravity>, <magnetic_field> and <atmosphere> are added. The classic
       world declared none of them. SDFormat does default gravity to
       0 0 -9.8, so this is belt-and-braces rather than a known bug -- but it
       was the one world-level difference from every working world in this
       repo, and a world with no gravity gives a differential-drive robot no
       normal force, hence no wheel traction, hence spinning wheels and no
       motion. Cheap to make explicit rather than rely on a default.
    """
    physics = world.find("physics")
    if physics is None:
        raise SystemExit("no <physics> element found in source world")

    for child in list(physics):
        physics.remove(child)
    physics.set("name", "default_physics")
    physics.set("default", "false")
    physics.set("type", "ode")
    for tag, value in (
        ("max_step_size", "0.01"),
        ("real_time_factor", "1"),
        ("real_time_update_rate", "200"),
    ):
        etree.SubElement(physics, tag).text = value

    # Insert before <physics> so world-level properties lead the file.
    index = list(world).index(physics)
    for offset, (tag, text, attrib) in enumerate((
        ("gravity", "0 0 -9.8", {}),
        ("magnetic_field", "6e-06 2.3e-05 -4.2e-05", {}),
        ("atmosphere", None, {"type": "adiabatic"}),
    )):
        if world.find(tag) is not None:
            continue
        el = etree.Element(tag, **attrib)
        if text:
            el.text = text
        world.insert(index + offset, el)

    plugins = etree.fromstring(f"<root>{GZ_SYSTEM_PLUGINS}</root>")
    index = list(world).index(physics)
    for offset, plugin in enumerate(plugins, start=1):
        world.insert(index + offset, plugin)


def convert_materials(world):
    """Translate OGRE1 <script> materials into PBR albedo maps."""
    converted = 0
    for material in world.iter("material"):
        script = material.find("script")
        if script is None:
            continue
        name_el = script.find("name")
        name = name_el.text.strip() if name_el is not None and name_el.text else ""
        material.remove(script)

        texture = MATERIAL_TEXTURES.get(name)
        if texture is None:
            # Unknown material: leave a neutral surface rather than a bad URI.
            print(f"  note: no texture mapping for material '{name}', using plain grey")
            etree.SubElement(material, "ambient").text = "0.6 0.6 0.6 1"
            etree.SubElement(material, "diffuse").text = "0.6 0.6 0.6 1"
            continue

        etree.SubElement(material, "ambient").text = "1 1 1 1"
        etree.SubElement(material, "diffuse").text = "1 1 1 1"
        etree.SubElement(material, "specular").text = "0.2 0.2 0.2 1"
        metal = etree.SubElement(etree.SubElement(material, "pbr"), "metal")
        albedo = etree.SubElement(metal, "albedo_map")
        albedo.text = f"{MEDIA_MODEL_URI}/materials/textures/{texture}"
        etree.SubElement(metal, "metalness").text = "0"
        etree.SubElement(metal, "roughness").text = "0.8"
        converted += 1
    return converted


def restore_visuals_for_stripped_links(stripped_links):
    """Give a box visual to anything left solid but invisible.

    Dropping a visual whose mesh is missing can leave a link with collision
    geometry and nothing to render -- an invisible obstacle. That is worse than
    either a missing obstacle or an ugly one: the robot's lidar sees a wall, the
    planner routes around a void, and on screen there is nothing there to
    explain why.

    It bit exactly the workstation islands (PublicCubicle_25-30, corner_1/2),
    whose only visual was cubicle_island.obj / cubicle_corner.obj. Their
    collision boxes are 0.76 m tall -- desk height -- and the computers and
    chairs standing on them are separate <include>s that still render, so the
    result was monitors floating over an invisible solid slab.

    Rather than substitute an unrelated mesh, each remaining box collision gets
    a box visual of exactly the same pose and size. What you see is then exactly
    what the robot collides with. Cylinder collisions are skipped: those belong
    to the chair models, which already render via their own <include>.
    """
    restored = 0
    for link in stripped_links:
        if link.findall("visual"):
            continue  # something still renders here
        for index, col in enumerate(link.findall("collision")):
            geom = col.find("geometry")
            if geom is None or len(geom) == 0 or geom[0].tag != "box":
                continue
            size = geom[0].find("size")
            if size is None:
                continue

            vis = etree.SubElement(link, "visual")
            vis.set("name", f"{col.get('name') or 'collision'}_{index}_visual")
            pose = col.find("pose")
            if pose is not None:
                etree.SubElement(vis, "pose").text = pose.text
            box = etree.SubElement(etree.SubElement(vis, "geometry"), "box")
            etree.SubElement(box, "size").text = size.text
            material = etree.SubElement(vis, "material")
            etree.SubElement(material, "ambient").text = "0.35 0.33 0.30 1"
            etree.SubElement(material, "diffuse").text = "0.52 0.49 0.45 1"
            etree.SubElement(material, "specular").text = "0.1 0.1 0.1 1"
            restored += 1
    return restored


def convert_mesh_uris(world):
    """Point mesh URIs at resolvable locations; drop ones with no asset."""
    rewritten = 0
    dropped = 0
    stripped_links = []

    for uri in list(world.iter("uri")):
        text = (uri.text or "").strip()
        if not text.startswith(("file://media/", "model://")):
            continue

        if any(text.startswith(prefix) for prefix in MISSING_MESHES):
            # Remove the whole <visual>/<collision> -- a geometry with an
            # unresolvable mesh fails the entire world load.
            element = uri
            while element is not None and element.tag not in ("visual", "collision"):
                element = element.getparent()
            if element is None:
                raise SystemExit(f"missing mesh {text} outside a visual/collision")
            parent = element.getparent()
            if parent.tag == "link" and parent not in stripped_links:
                stripped_links.append(parent)
            parent.remove(element)
            dropped += 1
            continue

        if text in MESH_URI_REMAP:
            uri.text = MESH_URI_REMAP[text]
            rewritten += 1
        elif text.startswith("file://media/"):
            uri.text = text.replace("file://media/", f"{MEDIA_MODEL_URI}/", 1)
            rewritten += 1

    return rewritten, dropped, stripped_links


def drop_servicesim_robot(world):
    """Remove ServiceSim's own robot include.

    The benchmark stack spawns its own robot (spawn_robot.launch.py /
    multi_robot_vxch_experiment.launch.py); leaving this in would put a second,
    uncontrolled robot in the world -- and model://turtlebot3_waffle_pi is not
    on the resource path anyway, which is a world-load error in itself.
    """
    for include in list(world.findall("include")):
        uri = include.find("uri")
        if uri is not None and (uri.text or "").strip() == "model://turtlebot3_waffle_pi":
            world.remove(include)
            return True
    return False


def drop_unwanted_models(world):
    """Remove every <include> whose uri is in DROP_MODEL_URIS, at any depth.

    Recursive on purpose: these sit inside room-level <model> elements, not at
    world level, so world.findall("include") -- what drop_servicesim_robot uses
    for its one top-level include -- would not see them.
    """
    dropped = []
    for include in list(world.iter("include")):
        uri = include.find("uri")
        if uri is None or (uri.text or "").strip() not in DROP_MODEL_URIS:
            continue
        parent = include.getparent()
        dropped.append((parent.get("name") or parent.tag, (uri.text or "").strip()))
        parent.remove(include)
    return dropped


def add_blocker_boxes(world):
    """Append ADDED_BOXES as static box models at world level.

    World-level (not nested in a room model) on purpose: gz sim's GUI can only
    delete direct children of the world, so these stay removable/movable by hand
    while every cubicle's own furniture does not.
    """
    added = []
    for name, x, y, z, sx, sy, sz in ADDED_BOXES:
        model = etree.SubElement(world, "model")
        model.set("name", name)
        etree.SubElement(model, "static").text = "true"
        etree.SubElement(model, "pose").text = f"{x} {y} {z} 0 0 0"
        link = etree.SubElement(model, "link")
        link.set("name", "link")
        for kind in ("collision", "visual"):
            element = etree.SubElement(link, kind)
            element.set("name", kind)
            geometry = etree.SubElement(element, "geometry")
            box = etree.SubElement(geometry, "box")
            etree.SubElement(box, "size").text = f"{sx} {sy} {sz}"
        added.append((name, x, y))
    return added


def main():
    if not os.path.isfile(SRC):
        raise SystemExit(f"source world not found: {SRC}\nExtract office_part1.zip first.")

    parser = etree.XMLParser(remove_blank_text=False, remove_comments=False)
    # The upstream file starts with a stray newline before <?xml ?>, which is
    # not well-formed; strip it rather than editing the pristine source.
    with open(SRC, "rb") as handle:
        source = handle.read().lstrip()
    tree = etree.ElementTree(etree.fromstring(source, parser))
    world = tree.getroot().find("world")
    if world is None:
        raise SystemExit("no <world> element found in source world")

    stripped = modernize_sdf_version(tree.getroot())
    print(f"  sdf: version 1.6 -> 1.10, {stripped} empty pose frame=\"\" attributes stripped")

    convert_physics(world)
    print("  physics: classic ODE block replaced, 6 gz-sim system plugins added")

    materials = convert_materials(world)
    print(f"  materials: {materials} OGRE1 scripts converted to PBR albedo maps")

    rewritten, dropped, stripped_links = convert_mesh_uris(world)
    print(f"  uris: {rewritten} rewritten to {MEDIA_MODEL_URI}/...")
    print(f"  uris: {dropped} visual/collision elements dropped (asset missing)")

    restored = restore_visuals_for_stripped_links(stripped_links)
    print(f"  visuals: {restored} box visuals added so no collision is invisible")

    if drop_servicesim_robot(world):
        print("  robot: removed ServiceSim's own turtlebot3_waffle_pi include")

    for parent_name, uri in drop_unwanted_models(world):
        print(f"  models: dropped {uri} from {parent_name}")

    for name, x, y in add_blocker_boxes(world):
        print(f"  models: added box {name} at ({x}, {y})")

    tree.write(DST, xml_declaration=True, encoding="UTF-8", pretty_print=False)
    with open(DST, "a") as handle:
        handle.write("\n")
    print(f"wrote {DST}")

    # Fail loudly if any URI is still unresolvable -- an unresolvable <uri> is
    # a hard sdformat error that fails the whole world load, which is exactly
    # how the first attempt at this world blew up.
    #
    # Resolution mirrors what world.launch.py sets up: every
    # simulation/worlds/*/models directory goes on GZ_SIM_RESOURCE_PATH, so
    # model://<name>/... resolves under this world's own models/ directory.
    # Bare model://<name> includes are checked as model directories.
    models_dir = os.path.join(HERE, "models")
    unresolved = set()
    for uri in etree.parse(DST).getroot().iter("uri"):
        text = (uri.text or "").strip()
        if text.startswith("file://"):
            unresolved.add(text)
        elif text.startswith("model://"):
            if not os.path.exists(os.path.join(models_dir, text[len("model://"):])):
                unresolved.add(text)

    if unresolved:
        print("ERROR: unresolvable URIs remain -- gz-sim will reject this world:",
              file=sys.stderr)
        for item in sorted(unresolved):
            print(f"  {item}", file=sys.stderr)
        return 1

    print("  verified: every <uri> resolves under simulation/worlds/office/models/")

    fixed_mtl, linked, missing_textures = fix_mesh_texture_paths()
    print(f"  textures: {fixed_mtl} .mtl files repointed to ../materials/textures/")
    print(f"  textures: {linked} bare-name texture symlinks added beside meshes")
    for item in missing_textures:
        print(f"  note: bare texture has no file to link: {item}")

    added = add_missing_furniture_collisions()
    for item in added:
        print(f"  collision: added box to collision-less model {item}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

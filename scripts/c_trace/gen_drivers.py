#!/usr/bin/env python3
"""Generate the headless C++ trace drivers for the Character mover samples.

The drivers copy the scene construction and physics methods of `DynamicMoverSample`
and `GeometricMoverSample` verbatim out of the pinned C sample source
(`samples/sample_character.cpp`, unchanged from upstream 3362633 to 38f2ae1) and replace only the app
plumbing (camera, key table, ImGui, rendering) with a scripted camera + keys, so the
C reference and the Rust port (`demo/wasm/src/character_demo/{dynamic,mover}/tests.rs`)
can run the same script and be diffed bit-for-bit.

Usage: gen_drivers.py <sample_character.cpp> <out_dir>
Writes <out_dir>/dyn_sample.h and <out_dir>/gm_sample.h. See run.sh.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def find_line(lines, prefix, start=0):
    for i in range(start, len(lines)):
        if lines[i].startswith(prefix):
            return i
    raise SystemExit(f"marker not found: {prefix!r}")


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise SystemExit(f"expected exactly one occurrence of: {old[:60]!r}")
    return text.replace(old, new)


def dynamic_driver(lines):
    # Constructor body: after `explicit DynamicMoverSample(...)` + `: Sample` + `{`, up to
    # (not including) the closing brace before the destructor.
    ctor = find_line(lines, "\texplicit DynamicMoverSample( SampleContext* context )")
    dtor = find_line(lines, "\t~DynamicMoverSample() override", ctor)
    body = "\n".join(lines[ctor + 3 : dtor - 2])
    body = replace_once(
        body,
        "\t\tif ( m_context->restart == false )\n\t\t{\n"
        "\t\t\tm_camera->SetView( 180.0f, 20.0f, 6.0f, startPosition );\n\t\t}",
        "\t\tm_camera->SetView( 180.0f, 20.0f, 6.0f, startPosition );",
    )
    body = replace_once(body, "\t\tsapp_lock_mouse( true );\n", "")
    body = replace_once(body, "\t\tm_launcherMesh = FindOrAddHull( m_launcherHull );\n", "")

    # Physics methods: AddRamp .. ShootBall (everything before DrawWeapon).
    first = find_line(lines, "\tvoid AddRamp(", dtor)
    last = find_line(lines, "\tvoid DrawWeapon(", first)
    methods = "\n".join(lines[first : last - 1])

    prefix = open(os.path.join(HERE, "dynamic_prefix.h")).read()
    suffix = open(os.path.join(HERE, "dynamic_suffix.h")).read()
    return prefix + body + "\n\t}\n\n" + methods + suffix


GM_GET_INPUT = """\tb3WorldId m_worldId;
\tCamera m_cam;
\tCamera* m_camera = &m_cam;
\tKeys m_keys = {};
\tbool m_didStep = false;

\tMoverInput GetMoverInput() const
\t{
\t\tMoverInput input = {};
\t\tinput.forward = -m_camera->GetForward();
\t\tinput.right = m_camera->GetRight();
\t\tinput.forward.y = 0.0f;
\t\tinput.forward = b3Normalize( input.forward );
\t\tif ( m_camera->m_thirdPerson == false )
\t\t\treturn input;
\t\tif ( m_keys.w )
\t\t\tinput.throttle.x += 1.0f;
\t\tif ( m_keys.s )
\t\t\tinput.throttle.x -= 1.0f;
\t\tif ( m_keys.a )
\t\t\tinput.throttle.y -= 1.0f;
\t\tif ( m_keys.d )
\t\t\tinput.throttle.y += 1.0f;
\t\tinput.jump = m_keys.space;
\t\tinput.sprint = m_keys.shift;
\t\tinput.walk = m_keys.ctrl;
\t\treturn input;
\t}

\tGeometricMoverSample()
\t{
\t\tb3WorldDef worldDef = b3DefaultWorldDef();
\t\tworldDef.workerCount = 1;
\t\tworldDef.enableSleep = true;
\t\tm_worldId = b3CreateWorld( &worldDef );
\t\tb3World_SetContactRecycleDistance( m_worldId, 0.05f );
"""


def geometric_driver(lines):
    start = find_line(lines, "struct MoverImpact")
    end = find_line(lines, "static int sampleMover =", start)
    body = "\n".join(lines[start:end])
    body = replace_once(body, "class GeometricMoverSample : public Sample", "class GeometricMoverSample")
    body = replace_once(
        body,
        "\texplicit GeometricMoverSample( SampleContext* context )\n\t\t: Sample( context )\n\t{",
        GM_GET_INPUT,
    )
    body = replace_once(
        body,
        "\t\tif ( m_context->restart == false )\n\t\t{\n"
        "\t\t\tm_camera->SetView( 120.0f, 30.0f, 5.0f, moverPosition );\n\t\t}",
        "\t\tm_camera->SetView( 120.0f, 30.0f, 5.0f, moverPosition );",
    )
    body = replace_once(
        body,
        "\t~GeometricMoverSample() override\n\t{\n\t\tm_camera->m_thirdPerson = false;\n"
        "\t\tsapp_lock_mouse( false );",
        "\t~GeometricMoverSample()\n\t{",
    )
    # Drop Render / DrawControls / Keyboard and DrawImpact (drawing + ImGui only).
    a = body.index("\tvoid Render() override")
    b = body.index("\tvoid CreateFallingTree()")
    body = body[:a] + body[b:]
    a = body.index("\tvoid DrawImpact( const MoverImpact* impact, b3HexColor hexColor )")
    b = body.index("\t// True once the body has held still for a second.")
    body = body[:a] + body[b:]
    body = replace_once(
        body,
        "\t\t*timer += m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;",
        "\t\t*timer += 1.0f / 60.0f;",
    )
    body = replace_once(
        body,
        "\t\tfloat timeStep = m_context->hertz > 0.0f ? 1.0f / m_context->hertz : 0.0f;",
        "\t\tfloat timeStep = 1.0f / 60.0f;",
    )
    body = replace_once(
        body,
        "\t\tSetDrawOrigin( m_camera->DrawOrigin() );\n\t\tm_mover.Draw();\n\n"
        '\t\tDrawTextLine( "third person (T) = %d", m_camera->m_thirdPerson );\n'
        '\t\tDrawTextLine( "deltaX = %g, deltaY = %g", m_mouseDelta.x, m_mouseDelta.y );\n\n'
        "\t\tSample::Step();\n",
        "\t\tm_mover.Draw();\n\n\t\tb3World_Step( m_worldId, timeStep, 4 );\n\t\tm_didStep = true;\n",
    )
    body = replace_once(
        body,
        "\t\tif ( m_boxImpact.valid )\n\t\t{\n"
        "\t\t\tDrawCube( m_impactTransform, 2.0f * m_fallingExtent,\n"
        "\t\t\t\t\t  MakeColorAlpha( b3_colorYellow, 0.4f ) );\n\t\t}\n\n"
        "\t\tDrawImpact( &m_boxImpact, b3_colorYellow );\n"
        "\t\tDrawImpact( &m_treeImpact, b3_colorCyan );\n",
        "",
    )
    body = replace_once(
        body,
        "\tstatic Sample* Create( SampleContext* context )\n\t{\n"
        "\t\treturn new GeometricMoverSample( context );\n\t}\n",
        "",
    )
    body = replace_once(body, "\tvoid Step() override", "\tvoid Step()")
    head = (
        "// Generated by scripts/c_trace/gen_drivers.py from samples/sample_character.cpp:\n"
        "// GeometricMoverSample with the app plumbing replaced by a scripted camera + keys.\n"
        '#include "dyn_sample.h"\n#include "geometric_mover.h"\n#include "mesh_loader.h"\n\n'
    )
    return head + body + "\n"


def main():
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    lines = open(sys.argv[1]).read().split("\n")
    out = sys.argv[2]
    with open(os.path.join(out, "dyn_sample.h"), "w") as f:
        f.write(dynamic_driver(lines))
    with open(os.path.join(out, "gm_sample.h"), "w") as f:
        f.write(geometric_driver(lines))


if __name__ == "__main__":
    main()

import os
import re
from typing import Dict, List, Optional, Tuple


TOP_BLOCK_RE = re.compile(
    r"^(PART|MARKER|JOINT|GGEOM|GGEOMCONTACT|MOTION|EXPRESSION|UNITS|ACCGRAV|OUTPUT|INTPAR|ANALYSIS|FUNCTION|FORCE|TORQUE)\s*/",
    re.IGNORECASE,
)
FLOAT_RE = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eEdD][-+]?\d+)?")
CONTACT_MODE = "mask"  # "auto" | "mask" | "pair"
VELOCITY_ACTUATOR_KV = 1.0e-6
SIM_TIMESTEP = 1.0e-5
SDF_ITERATIONS = 80
SDF_INITPOINTS = 40
CONTACT_SOLREF = "0.0001 2.0"
CONTACT_SOLIMP = "0.99 0.999 0.0002 0.5 2"


def _read_lines(file_path: str) -> List[str]:
    for enc in ("mbcs", "utf-8", "latin1"):
        try:
            with open(file_path, "r", encoding=enc) as f:
                return f.readlines()
        except UnicodeDecodeError:
            continue
    with open(file_path, "r", encoding="latin1", errors="ignore") as f:
        return f.readlines()


def _is_block_header(line: str) -> bool:
    return bool(TOP_BLOCK_RE.match(line.strip()))


def _parse_kv(line: str) -> Tuple[Optional[str], Optional[str]]:
    if "=" not in line:
        return None, None
    key_raw, val_raw = line.split("=", 1)
    key = key_raw.strip().lstrip(",").strip()
    val = val_raw.strip().strip("'").strip()
    return key, val


def _floats(text: str) -> List[float]:
    values = []
    for tok in FLOAT_RE.findall(text.replace("D", "E").replace("d", "e")):
        try:
            values.append(float(tok))
        except ValueError:
            pass
    return values


def _to_int(value: Optional[str], default: int = 0) -> int:
    if value is None:
        return default
    vals = _floats(value)
    if not vals:
        return default
    return int(round(vals[0]))


def _to_float(value: Optional[str], default: float = 0.0) -> float:
    if value is None:
        return default
    vals = _floats(value)
    if not vals:
        return default
    return vals[0]


def _safe_name(raw: Optional[str], fallback: str) -> str:
    if not raw:
        return fallback
    return raw.replace("'", "").strip()


def _clean_function_token(raw: Optional[str]) -> str:
    if not raw:
        return ""
    token = raw.strip()
    while token.endswith("\\"):
        token = token[:-1].strip()
    return token.strip("'").strip()


def _unit_scale_length(unit_name: str) -> float:
    key = re.sub(r"[^a-zA-Z]", "", unit_name).lower()
    table = {
        "meter": 1.0,
        "metre": 1.0,
        "m": 1.0,
        "millimeter": 1e-3,
        "millimetre": 1e-3,
        "mm": 1e-3,
        "centimeter": 1e-2,
        "centimetre": 1e-2,
        "cm": 1e-2,
    }
    return table.get(key, 1e-3)


def _unit_scale_mass(unit_name: str) -> float:
    key = re.sub(r"[^a-zA-Z]", "", unit_name).lower()
    table = {
        "kilogram": 1.0,
        "kg": 1.0,
        "gram": 1e-3,
        "g": 1e-3,
    }
    return table.get(key, 1.0)


def _unit_scale_time(unit_name: str) -> float:
    key = re.sub(r"[^a-zA-Z]", "", unit_name).lower()
    table = {
        "second": 1.0,
        "sec": 1.0,
        "s": 1.0,
        "millisecond": 1e-3,
        "ms": 1e-3,
    }
    return table.get(key, 1.0)


def _parse_ip_diag(part_block: Dict) -> Optional[Tuple[float, float, float]]:
    vals = _floats(part_block["props"].get("IP"))
    if len(vals) >= 3:
        return vals[0], vals[1], vals[2]

    lines = part_block.get("lines", [])
    for idx, line in enumerate(lines):
        if "IP" in line and "=" in line:
            vals = _floats(line)
            nxt = idx + 1
            while len(vals) < 3 and nxt < len(lines):
                cont = lines[nxt]
                if "=" in cont and "IP" not in cont:
                    break
                vals.extend(_floats(cont))
                nxt += 1
            if len(vals) >= 3:
                return vals[0], vals[1], vals[2]
            break

    return None


def _read_kv_block(lines: List[str], start: int, include_start_line: bool = False) -> Tuple[Dict, List[str], int]:
    props = {}
    raw_lines = []
    i = start
    if not include_start_line:
        i += 1

    while i < len(lines):
        s = lines[i].strip()
        if s and not s.startswith("!") and _is_block_header(s):
            break
        if not s or s.startswith("!"):
            i += 1
            continue
        raw_lines.append(s)
        k, v = _parse_kv(s)
        if k:
            props[k] = v
        i += 1

    return props, raw_lines, i


def _read_accgrav_block(lines: List[str], start: int) -> Tuple[Dict, List[str], int]:
    props = {}
    raw_lines = []
    i = start
    while i < len(lines):
        s = lines[i].strip()
        if i > start and s and not s.startswith("!") and _is_block_header(s):
            break
        if not s or s.startswith("!"):
            i += 1
            continue
        raw_lines.append(s)
        if "/" in s and "=" in s:
            _, rhs = s.split("/", 1)
            k, v = _parse_kv(rhs)
            if k:
                props[k] = v
        k, v = _parse_kv(s)
        if k:
            props[k] = v
        i += 1
    return props, raw_lines, i


def _read_ggeom_block(lines: List[str], start: int) -> Tuple[Dict, List[str], List[List[float]], List[List[int]], int]:
    props = {}
    raw_lines = []
    nodes: List[List[float]] = []
    faces: List[List[int]] = []
    i = start + 1

    while i < len(lines):
        s = lines[i].strip()
        if s and not s.startswith("!") and _is_block_header(s):
            break
        if not s or s.startswith("!"):
            i += 1
            continue

        raw_lines.append(s)
        upper = s.lstrip(", ").upper()

        if upper.startswith("PATCHES"):
            i += 1
            while i < len(lines):
                pl = lines[i].strip()
                if pl and not pl.startswith("!") and _is_block_header(pl):
                    break
                if not pl or pl.startswith("!"):
                    i += 1
                    continue
                if "=" in pl and not pl.lstrip().startswith("3,"):
                    break

                items = [x.strip() for x in pl.split(",")]
                if len(items) >= 4 and items[0] == "3":
                    try:
                        faces.append([int(items[1]), int(items[2]), int(items[3])])
                        i += 1
                        continue
                    except ValueError:
                        pass

                vals = _floats(pl)
                if len(vals) >= 3:
                    nodes.append(vals[-3:])
                    i += 1
                    continue
                break
            continue

        if upper.startswith("NODE"):
            i += 1
            while i < len(lines):
                nl = lines[i].strip()
                if nl and not nl.startswith("!") and _is_block_header(nl):
                    break
                if not nl or nl.startswith("!"):
                    i += 1
                    continue
                if "=" in nl:
                    break
                vals = _floats(nl)
                if len(vals) >= 3:
                    nodes.append(vals[-3:])
                    i += 1
                    continue
                break
            continue

        k, v = _parse_kv(s)
        if k:
            props[k] = v
        i += 1

    return props, raw_lines, nodes, faces, i


def _resolve_motion_target(
    motions: Dict[int, Dict], expressions: Dict[int, Dict], joints: Dict[int, Dict]
) -> Tuple[Optional[str], Optional[str], float]:
    for mid in sorted(motions):
        mprops = motions[mid]["props"]
        motion_name = _safe_name(mprops.get("NAME"), f"Motion{mid}")
        joint_id = _to_int(mprops.get("JOINT"), -1)
        if joint_id not in joints:
            continue
        joint_name = _safe_name(joints[joint_id]["props"].get("NAME"), f"Joint{joint_id}")

        token = _clean_function_token(mprops.get("FUNCTION"))
        if not token:
            return motion_name, joint_name, 1.0

        vals = _floats(token)
        if vals and token.replace(".", "", 1).replace("-", "", 1).isdigit():
            return motion_name, joint_name, vals[0]

        expr_id = _to_int(token, -1)
        if expr_id in expressions:
            expr_token = _clean_function_token(expressions[expr_id]["props"].get("FUNCTION"))
            expr_vals = _floats(expr_token)
            if expr_vals:
                return motion_name, joint_name, expr_vals[0]

        num = _to_float(token, None)
        if num is not None:
            return motion_name, joint_name, num

        return motion_name, joint_name, 1.0

    return None, None, 1.0


def _write_two_gear_baseline_xml(src_xml_path: str, dst_xml_path: str) -> None:
    with open(src_xml_path, "r", encoding="utf-8") as f:
        xml_text = f.read()

    # Disable SwingArm contact while keeping all other model settings identical.
    pattern_with_masks = re.compile(
        r'(<geom\s+name="geom_SWINGARM000"[^>]*?)contype="[^"]*"\s+conaffinity="[^"]*"',
        re.IGNORECASE,
    )
    xml_text, count = pattern_with_masks.subn(r'\1contype="0" conaffinity="0"', xml_text, count=1)

    if count == 0:
        pattern_no_masks = re.compile(
            r'(<geom\s+name="geom_SWINGARM000"\s+)([^>]*?)>',
            re.IGNORECASE,
        )
        xml_text, count = pattern_no_masks.subn(
            r'\1contype="0" conaffinity="0" \2>',
            xml_text,
            count=1,
        )

    if count == 0:
        raise RuntimeError("Failed to locate SwingArm geom when generating two-gear baseline XML.")

    with open(dst_xml_path, "w", encoding="utf-8") as f:
        f.write(xml_text)


def parse_rmd(file_path: str, output_dir: str) -> None:
    lines = _read_lines(file_path)

    parts: Dict[int, Dict] = {}
    markers: Dict[int, Dict] = {}
    joints: Dict[int, Dict] = {}
    motions: Dict[int, Dict] = {}
    expressions: Dict[int, Dict] = {}
    ggeoms: Dict[int, Dict] = {}
    contacts: Dict[int, Dict] = {}
    units_data = {}
    gravity_data = {}

    re_part = re.compile(r"^PART\s*/\s*(\d+)", re.IGNORECASE)
    re_marker = re.compile(r"^MARKER\s*/\s*(\d+)", re.IGNORECASE)
    re_joint = re.compile(r"^JOINT\s*/\s*(\d+)", re.IGNORECASE)
    re_motion = re.compile(r"^MOTION\s*/\s*(\d+)", re.IGNORECASE)
    re_expr = re.compile(r"^EXPRESSION\s*/\s*(\d+)", re.IGNORECASE)
    re_ggeom = re.compile(r"^GGEOM\s*/\s*(\d+)", re.IGNORECASE)
    re_contact = re.compile(r"^GGEOMCONTACT\s*/\s*(\d+)", re.IGNORECASE)
    re_units = re.compile(r"^UNITS\s*/", re.IGNORECASE)
    re_accgrav = re.compile(r"^ACCGRAV\s*/", re.IGNORECASE)

    i = 0
    while i < len(lines):
        s = lines[i].strip()
        if not s or s.startswith("!"):
            i += 1
            continue

        m_part = re_part.match(s)
        m_marker = re_marker.match(s)
        m_joint = re_joint.match(s)
        m_motion = re_motion.match(s)
        m_expr = re_expr.match(s)
        m_ggeom = re_ggeom.match(s)
        m_contact = re_contact.match(s)

        if m_part:
            bid = int(m_part.group(1))
            props, raw_lines, nxt = _read_kv_block(lines, i)
            parts[bid] = {"props": props, "lines": raw_lines}
            i = nxt
            continue

        if m_marker:
            bid = int(m_marker.group(1))
            props, raw_lines, nxt = _read_kv_block(lines, i)
            markers[bid] = {"props": props, "lines": raw_lines}
            i = nxt
            continue

        if m_joint:
            bid = int(m_joint.group(1))
            props, raw_lines, nxt = _read_kv_block(lines, i)
            jtype = "revolute" if any("REVOLUTE" in ln.upper() for ln in raw_lines) else ""
            if jtype:
                props["JOINT_TYPE"] = jtype
            joints[bid] = {"props": props, "lines": raw_lines}
            i = nxt
            continue

        if m_motion:
            bid = int(m_motion.group(1))
            props, raw_lines, nxt = _read_kv_block(lines, i)
            motions[bid] = {"props": props, "lines": raw_lines}
            i = nxt
            continue

        if m_expr:
            bid = int(m_expr.group(1))
            props, raw_lines, nxt = _read_kv_block(lines, i)
            expressions[bid] = {"props": props, "lines": raw_lines}
            i = nxt
            continue

        if m_contact:
            bid = int(m_contact.group(1))
            props, raw_lines, nxt = _read_kv_block(lines, i)
            contacts[bid] = {"props": props, "lines": raw_lines}
            i = nxt
            continue

        if m_ggeom:
            bid = int(m_ggeom.group(1))
            props, raw_lines, nodes, faces, nxt = _read_ggeom_block(lines, i)
            ggeoms[bid] = {"props": props, "lines": raw_lines, "nodes": nodes, "faces": faces}
            i = nxt
            continue

        if re_units.match(s):
            props, raw_lines, nxt = _read_kv_block(lines, i)
            units_data.update(props)
            i = nxt
            continue

        if re_accgrav.match(s):
            props, raw_lines, nxt = _read_accgrav_block(lines, i)
            gravity_data.update(props)
            i = nxt
            continue

        i += 1

    length_unit = units_data.get("LENGTH", "Millimeter")
    mass_unit = units_data.get("MASS", "Kilogram")
    time_unit = units_data.get("TIME", "Second")
    force_unit = units_data.get("FORCE", "Newton")

    scale_len = _unit_scale_length(length_unit)
    scale_mass = _unit_scale_mass(mass_unit)
    scale_time = _unit_scale_time(time_unit)
    scale_inertia = scale_mass * scale_len * scale_len

    g_scale = scale_len / (scale_time * scale_time)
    grav = (
        _to_float(gravity_data.get("IGRAV"), 0.0) * g_scale,
        _to_float(gravity_data.get("JGRAV"), 0.0) * g_scale,
        _to_float(gravity_data.get("KGRAV"), 0.0) * g_scale,
    )

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    part_geom: Dict[int, Dict] = {}
    ggeom_to_part: Dict[int, int] = {}

    for gid, data in ggeoms.items():
        rm_id = _to_int(data["props"].get("RM"), 0)
        rm_pos = [0.0, 0.0, 0.0]
        part_id = 0
        if rm_id in markers:
            mprops = markers[rm_id]["props"]
            rm_vals = _floats(mprops.get("QP", "0,0,0"))
            if len(rm_vals) >= 3:
                rm_pos = rm_vals[:3]
            part_id = _to_int(mprops.get("PART"), 0)

        if part_id and part_id in parts:
            part_name = _safe_name(parts[part_id]["props"].get("NAME"), f"part_{part_id}")
        else:
            part_name = _safe_name(data["props"].get("NAME"), f"geom_{gid}")

        obj_path = os.path.join(output_dir, f"{part_name}.obj")
        with open(obj_path, "w", encoding="utf-8") as f:
            f.write(f"# Exported from RMD GGEOM {gid}\n")
            for v in data["nodes"]:
                f.write(f"v {v[0]} {v[1]} {v[2]}\n")
            for tri in data["faces"]:
                f.write(f"f {tri[0]} {tri[1]} {tri[2]}\n")

        if part_id:
            part_geom[part_id] = {"obj": part_name, "rm_pos": rm_pos, "ggeom": gid}
            ggeom_to_part[gid] = part_id

    contact_records = []
    for cid in sorted(contacts):
        cprops = contacts[cid]["props"]
        igid = _to_int(cprops.get("IGGEOMID"), 0)
        jgid = _to_int(cprops.get("JGGEOMID"), 0)
        ipid = ggeom_to_part.get(igid, 0)
        jpid = ggeom_to_part.get(jgid, 0)
        if not ipid or not jpid or ipid == jpid:
            continue
        mu = _to_float(cprops.get("D_F_C"), 0.16)
        margin_mm = _to_float(cprops.get("MAXPEN"), 0.0)
        margin = max(0.0, margin_mm * scale_len)
        cname = _safe_name(cprops.get("NAME"), f"contact_{cid}")
        contact_records.append(
            {"name": cname, "pid1": ipid, "pid2": jpid, "mu": mu, "margin": margin}
        )

    contact_masks = {}
    if CONTACT_MODE == "mask" and contact_records:
        involved = sorted({r["pid1"] for r in contact_records} | {r["pid2"] for r in contact_records})
        pid_bit = {}
        for idx, pid in enumerate(involved):
            if idx >= 30:
                break
            pid_bit[pid] = 1 << idx

        for pid in involved:
            contact_masks[pid] = {"contype": pid_bit[pid], "conaffinity": 0}

        for r in contact_records:
            p1 = r["pid1"]
            p2 = r["pid2"]
            if p1 in pid_bit and p2 in pid_bit:
                contact_masks[p1]["conaffinity"] |= pid_bit[p2]
                contact_masks[p2]["conaffinity"] |= pid_bit[p1]

    body_pos_map: Dict[int, List[float]] = {}
    body_name_map: Dict[int, str] = {}
    body_geom_name: Dict[int, str] = {}

    joint_defs_by_body: Dict[int, List[Dict]] = {}
    ground_part_id = 1

    for jid, jdata in joints.items():
        props = jdata["props"]
        if props.get("JOINT_TYPE", "").lower() != "revolute":
            continue

        marker_i = _to_int(props.get("I"), 0)
        marker_j = _to_int(props.get("J"), 0)
        part_i = _to_int(markers.get(marker_i, {}).get("props", {}).get("PART"), 0)
        part_j = _to_int(markers.get(marker_j, {}).get("props", {}).get("PART"), 0)

        child_part = 0
        child_marker = 0
        if part_i and part_i != ground_part_id and (not part_j or part_j == ground_part_id):
            child_part = part_i
            child_marker = marker_i
        elif part_j and part_j != ground_part_id and (not part_i or part_i == ground_part_id):
            child_part = part_j
            child_marker = marker_j
        elif part_j and part_j != ground_part_id:
            child_part = part_j
            child_marker = marker_j
        elif part_i and part_i != ground_part_id:
            child_part = part_i
            child_marker = marker_i

        if not child_part:
            continue

        jname = _safe_name(props.get("NAME"), f"joint_{jid}")
        jqp = _floats(markers.get(child_marker, {}).get("props", {}).get("QP", "0,0,0"))
        if len(jqp) < 3:
            jqp = [0.0, 0.0, 0.0]
        joint_defs_by_body.setdefault(child_part, []).append(
            {"name": jname, "global_pos": jqp[:3], "axis": [1.0, 0.0, 0.0]}
        )

    motion_name, motion_joint_name, motion_target_vel = _resolve_motion_target(
        motions, expressions, joints
    )

    xml_path = os.path.join(os.path.dirname(output_dir), "fuzajiaolian.xml")
    print(f"Generating XML: {xml_path}")

    with open(xml_path, "w", encoding="utf-8") as f:
        f.write('<simcore model="fuzajiaolian">\n')
        f.write('  <compiler angle="radian" autolimits="true" boundmass="1e-8" boundinertia="1e-12"/>\n')
        f.write('  <size nconmax="10000" memory="100M"/>\n')
        f.write(
            f'  <!-- Units from RMD: length={length_unit}, mass={mass_unit}, '
            f'time={time_unit}, force={force_unit} -->\n'
        )
        f.write(
            f'  <option timestep="{SIM_TIMESTEP:.15g}" gravity="{grav[0]:.9g} {grav[1]:.9g} {grav[2]:.9g}" '
            f'sdf_iterations="{SDF_ITERATIONS}" sdf_initpoints="{SDF_INITPOINTS}"/>\n\n'
        )

        f.write("  <asset>\n")
        for pid in sorted(part_geom):
            obj_name = part_geom[pid]["obj"]
            f.write(
                f'    <mesh name="{obj_name}_mesh" file="obj/{obj_name}.obj" '
                f'scale="{scale_len} {scale_len} {scale_len}"/>\n'
            )
        f.write("  </asset>\n\n")

        f.write("  <default>\n")
        f.write(
            f'    <geom type="sdf" friction="0.16" solref="{CONTACT_SOLREF}" solimp="{CONTACT_SOLIMP}"/>\n'
        )
        f.write("  </default>\n\n")

        f.write("  <worldbody>\n")
        for pid in sorted(parts):
            if pid == ground_part_id:
                continue

            pprops = parts[pid]["props"]
            name = _safe_name(pprops.get("NAME"), f"part_{pid}")
            body_name_map[pid] = name

            cm_marker = _to_int(pprops.get("CM"), 0)
            cm_qp = _floats(markers.get(cm_marker, {}).get("props", {}).get("QP", "0,0,0"))
            if len(cm_qp) < 3:
                cm_qp = [0.0, 0.0, 0.0]
            body_pos = [cm_qp[0] * scale_len, cm_qp[1] * scale_len, cm_qp[2] * scale_len]
            body_pos_map[pid] = body_pos

            mass = _to_float(pprops.get("MASS"), 0.0) * scale_mass
            ip_diag = _parse_ip_diag(parts[pid])
            if ip_diag is None:
                ixx, iyy, izz = (1e-12, 1e-12, 1e-12)
            else:
                ixx = ip_diag[0] * scale_inertia
                iyy = ip_diag[1] * scale_inertia
                izz = ip_diag[2] * scale_inertia

            f.write(
                f'    <body name="{name}" pos="{body_pos[0]:.15g} {body_pos[1]:.15g} {body_pos[2]:.15g}">\n'
            )
            f.write(
                f'      <inertial mass="{mass:.15g}" diaginertia="{ixx:.15g} {iyy:.15g} {izz:.15g}" pos="0 0 0"/>\n'
            )

            for jdef in joint_defs_by_body.get(pid, []):
                jp = [
                    jdef["global_pos"][0] * scale_len - body_pos[0],
                    jdef["global_pos"][1] * scale_len - body_pos[1],
                    jdef["global_pos"][2] * scale_len - body_pos[2],
                ]
                f.write(
                    f'      <joint name="{jdef["name"]}" type="hinge" axis="{jdef["axis"][0]:.0f} {jdef["axis"][1]:.0f} {jdef["axis"][2]:.0f}" '
                    f'pos="{jp[0]:.15g} {jp[1]:.15g} {jp[2]:.15g}"/>\n'
                )

            if pid in part_geom:
                obj_name = part_geom[pid]["obj"]
                rm_pos = part_geom[pid]["rm_pos"]
                gp = [
                    rm_pos[0] * scale_len - body_pos[0],
                    rm_pos[1] * scale_len - body_pos[1],
                    rm_pos[2] * scale_len - body_pos[2],
                ]
                geom_name = f"geom_{name}"
                body_geom_name[pid] = geom_name
                if CONTACT_MODE == "pair":
                    f.write(
                        f'      <geom name="{geom_name}" contype="0" conaffinity="0" mesh="{obj_name}_mesh" '
                        f'pos="{gp[0]:.15g} {gp[1]:.15g} {gp[2]:.15g}" rgba="0.8 0.2 0.2 1"/>\n'
                    )
                elif CONTACT_MODE == "mask" and pid in contact_masks:
                    contype = contact_masks[pid]["contype"]
                    conaffinity = contact_masks[pid]["conaffinity"]
                    f.write(
                        f'      <geom name="{geom_name}" contype="{contype}" conaffinity="{conaffinity}" mesh="{obj_name}_mesh" '
                        f'pos="{gp[0]:.15g} {gp[1]:.15g} {gp[2]:.15g}" rgba="0.8 0.2 0.2 1"/>\n'
                    )
                else:
                    f.write(
                        f'      <geom name="{geom_name}" mesh="{obj_name}_mesh" '
                        f'pos="{gp[0]:.15g} {gp[1]:.15g} {gp[2]:.15g}" rgba="0.8 0.2 0.2 1"/>\n'
                    )

            f.write("    </body>\n")
        f.write("  </worldbody>\n\n")

        pairs = []
        for r in contact_records:
            geom1 = body_geom_name.get(r["pid1"])
            geom2 = body_geom_name.get(r["pid2"])
            if not geom1 or not geom2:
                continue
            pairs.append((r["name"], geom1, geom2, r["mu"], r["margin"]))

        if pairs and CONTACT_MODE == "pair":
            f.write("  <contact>\n")
            for cname, g1, g2, mu, margin in pairs:
                f.write(
                    f'    <pair name="{cname}" geom1="{g1}" geom2="{g2}" '
                    f'friction="{mu:.15g} {mu:.15g} 0.005 0.0001 0.0001" margin="{margin:.15g}"/>\n'
                )
            f.write("  </contact>\n\n")
        elif pairs:
            f.write(f"  <!-- Contact pairs parsed from RMD (mode={CONTACT_MODE}):\n")
            for cname, g1, g2, _, _ in pairs:
                f.write(f"       {cname}: {g1} <-> {g2}\n")
            f.write("  -->\n\n")

        if motion_joint_name:
            actuator_name = motion_name if motion_name else "drive_velocity"
            f.write("  <actuator>\n")
            f.write(
                f'    <velocity name="{actuator_name}" joint="{motion_joint_name}" kv="{VELOCITY_ACTUATOR_KV:.15g}"/>\n'
            )
            f.write("  </actuator>\n")
            f.write(f"  <!-- Motion target velocity from RMD function: {motion_target_vel:.15g} rad/s -->\n")
        else:
            f.write("  <actuator/>\n")

        f.write("</simcore>\n")

    baseline_xml_path = os.path.join(os.path.dirname(output_dir), "fuzajiaolian_two_gear_baseline.xml")
    _write_two_gear_baseline_xml(xml_path, baseline_xml_path)
    print(f"Wrote XML and {len(part_geom)} OBJ meshes.")
    print(f"Wrote two-gear baseline XML: {baseline_xml_path}")


if __name__ == "__main__":
    parse_rmd(
        "e:\\workspace\\NexDyn-Diff\\temps\\models\\complex_gear_system\\fuzajiaolian.rmd",
        "e:\\workspace\\NexDyn-Diff\\models\\obj",
    )

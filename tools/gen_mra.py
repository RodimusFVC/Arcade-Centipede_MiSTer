#!/usr/bin/env python3
"""Generate MRAs for the Centipede core straight from MAME centiped.cpp.

Region placement matches rtl/centiped_board.sv; images are copied as dumped (descrambling happens in the core).
DIP switches and the input map come from the set's MAME INPUT_PORTS.

DIP space: every DIP field of every port is packed into 3 bytes (Main_MiSTer only keeps 3 default bytes reliably,
see vault "MRA switches default keeps only 3 bytes"); the input map points a port bit at a DIP bit (0x80 | n) or at
a control id (1-63, XORed onto the port's idle level).

Release layout: a parent set goes to <releases>/<Title>.mra with the trailing parentheses dropped; a clone goes to
<releases>/_alternatives/_<Parent Title>/<Full Title>.mra.

Usage:
    gen_mra.py <driver.cpp> <releases_dir> [set ...]      (no sets: every supported set)
"""
import copy
import re
import sys
from pathlib import Path

MAME_VERSION = "0289"
RBF = "Centipede"
SYNC_PROM = ("136001-213.p4", "6fa3093a", "centiped")   # Atari sync PROM; same dump on every Atari board here

# ioctl index 0
CPU_BASE, GFX_LO, GFX_HI, SYNC_BASE, COLOR_BASE = 0x00000, 0x10000, 0x12000, 0x14000, 0x14100

# (machine config, init) -> board variant byte
SUPPORTED = {
    ("centiped", "empty_init"): 0,
    ("centipedj", "empty_init"): 1,
    ("centipdb", "empty_init"): 2,
    ("caterplr", "empty_init"): 3,
    ("magworm", "empty_init"): 4,
    ("milliped", "empty_init"): 5,
    ("multiped", "init_multiped"): 6,
    ("warlords", "empty_init"): 7,
    ("mazeinv", "empty_init"): 8,
    ("bullsdrt", "init_bullsdrt"): 9,
}

IGNORED_REGIONS = {"plds"}
UNUSED_PROMS = {"ba40e1a9", "a5101624",            # millpac / magworma PROMs, "not used" in MAME
                "d841b7e0"}                         # bullsdrt 82s147, "unknown" in MAME

F_TB, F_PADDLE, F_STICK, F_NOJOY, F_VERT, F_ROT90 = 0x01, 0x02, 0x04, 0x08, 0x10, 0x80

# port order on the board: index 1 idle bytes 4-9, input map bytes 16-63
PORTS = [("IN0",), ("IN1",), ("IN2",), ("IN3",), ("DSW1",), ("DSW2",)]

# control ids (Arcade-Centipede.sv)
CTL = {("JOYSTICK_UP", 1): 1, ("JOYSTICK_DOWN", 1): 2, ("JOYSTICK_LEFT", 1): 3, ("JOYSTICK_RIGHT", 1): 4,
       ("BUTTON1", 1): 5, ("BUTTON2", 1): 6, ("BUTTON3", 1): 7, ("BUTTON4", 1): 8,
       ("JOYSTICK_UP", 2): 9, ("JOYSTICK_DOWN", 2): 10, ("JOYSTICK_LEFT", 2): 11, ("JOYSTICK_RIGHT", 2): 12,
       ("BUTTON1", 2): 13, ("BUTTON2", 2): 14, ("BUTTON3", 2): 15, ("BUTTON4", 2): 16,
       ("COIN1", 0): 17, ("COIN2", 0): 18, ("START1", 0): 19, ("START2", 0): 20,
       ("TILT", 0): 21, ("SERVICE1", 0): 22, ("SERVICE", 0): 22, ("COIN3", 0): 23,
       ("BUTTON1", 3): 24, ("BUTTON1", 4): 25, ("START3", 0): 26, ("START4", 0): 27}
TRACKBALL_TYPES = {"TRACKBALL_X", "TRACKBALL_Y"}
IGNORED_TYPES = {"UNUSED", "UNKNOWN"}

# ---------------------------------------------------------------- parsing

GAME_RE = re.compile(r'^GAME\(\s*([\w?]+),\s*(\w+),\s*(\w+),\s*(\w+),\s*(\w+),\s*\w+,\s*(\w+),\s*(ROT\d+),\s*"([^"]*)",\s*"([^"]*)",\s*([^)]*)\)', re.M)
LOAD_RE = re.compile(r'ROM_LOAD\(\s*"([^"]+)",\s*(0x[0-9a-fA-F]+),\s*(0x[0-9a-fA-F]+),\s*(?:BAD_DUMP\s+)?CRC\(([0-9a-fA-F]+)\)')
CONT_RE = re.compile(r'ROM_CONTINUE\(\s*(0x[0-9a-fA-F]+),\s*(0x[0-9a-fA-F]+)\s*\)')
IGNORE_RE = re.compile(r'ROM_IGNORE\(\s*(0x[0-9a-fA-F]+)\s*\)')
REGION_RE = re.compile(r'ROM_REGION\(\s*(0x[0-9a-fA-F]+),\s*"([^"]+)"')
MACRO_RE = re.compile(r'\b(PORT_\w+|INPUT_PORTS_\w+)\b(\s*\(((?:[^()"]|"[^"]*"|\([^()]*\))*)\))?')


def parse_games(src):
    games = {}
    for m in GAME_RE.finditer(src):
        year, name, parent, machine, inputs, init, rot, manuf, desc, flags = m.groups()
        games[name] = dict(year=year, name=name, parent=None if parent == "0" else parent,
                           machine=machine, inputs=inputs, init=init, rot=rot, manuf=manuf, desc=desc,
                           working="MACHINE_NOT_WORKING" not in flags)
    return games


def parse_roms(src, setname):
    m = re.search(r'ROM_START\(\s*%s\s*\)(.*?)ROM_END' % re.escape(setname), src, re.S)
    if not m:
        raise SystemExit(f"ROM_START({setname}) not found")
    segs, region, rsize, last = [], None, 0, None
    for line in m.group(1).splitlines():
        line = line.split("//")[0]
        if "NO_DUMP" in line:
            continue
        if (r := REGION_RE.search(line)):
            region, rsize, last = r.group(2), int(r.group(1), 16), None
        elif (l := LOAD_RE.search(line)):
            name, off, length, crc = l.group(1), int(l.group(2), 16), int(l.group(3), 16), l.group(4).lower()
            last = dict(name=name, crc=crc, src=0, dst=off, len=length, flen=length, region=region, rsize=rsize)
            segs.append(last)
        elif (c := CONT_RE.search(line)):
            off, length = int(c.group(1), 16), int(c.group(2), 16)
            nxt = dict(last, src=last["src"] + last["len"], dst=off, len=length)
            segs.append(nxt)
            last = nxt
        elif IGNORE_RE.search(line):
            for s in segs:                              # file is longer than what MAME loads: always slice it
                if s["name"] == last["name"]:
                    s["partial"] = True
        elif "ROM_" in line and any(k in line for k in ("ROM_LOAD", "ROM_COPY", "ROM_FILL", "ROM_RELOAD")):
            raise SystemExit(f"{setname}: unhandled ROM statement: {line.strip()}")
    return segs


def def_str(s):
    s = s.strip()
    m = re.fullmatch(r'DEF_STR\(\s*(\w+)\s*\)', s)
    if not m:
        return s.strip('"')
    w = m.group(1)
    c = re.fullmatch(r'(\d+)C_(\d+)C', w)
    if c:
        return f"{c.group(1)}C/{c.group(2)}C"
    return w.replace("_", " ")


def idle_level(arg, mask):
    if arg == "IP_ACTIVE_LOW":
        return mask
    if arg == "IP_ACTIVE_HIGH":
        return 0
    return int(arg, 0) & mask


def split_args(a):
    out, depth, cur, q = [], 0, "", False
    for ch in a:
        if ch == '"':
            q = not q
        if not q and ch == "(":
            depth += 1
        elif not q and ch == ")":
            depth -= 1
        if ch == "," and depth == 0 and not q:
            out.append(cur.strip())
            cur = ""
        else:
            cur += ch
    if cur.strip():
        out.append(cur.strip())
    return out


def parse_inputs(src):
    """INPUT_PORTS name -> {tag: [field]}"""
    blocks = {m.group(1): m.group(2) for m in re.finditer(
        r'INPUT_PORTS_START\(\s*(\w+)\s*\)(.*?)INPUT_PORTS_END', src, re.S)}
    cache = {}

    def build(name):
        if name in cache:
            return copy.deepcopy(cache[name])
        ports, cur, fld = {}, None, None
        body = "\n".join(l.split("//")[0] for l in blocks[name].splitlines())
        body = re.sub(r'/\*.*?\*/', '', body, flags=re.S)
        for m in MACRO_RE.finditer(body):
            mac, args = m.group(1), split_args(m.group(3) or "")
            if mac == "PORT_INCLUDE":
                ports.update(build(args[0]))
            elif mac in ("PORT_START", "PORT_MODIFY"):
                cur = args[0].strip('"')
                if mac == "PORT_START":
                    ports[cur] = []
                fld = None
            elif mac in ("PORT_BIT", "PORT_DIPNAME", "PORT_CONFNAME", "PORT_SERVICE", "PORT_SERVICE_NO_TOGGLE",
                         "PORT_DIPUNUSED", "PORT_DIPUNUSED_DIPLOC", "PORT_DIPUNKNOWN", "PORT_DIPUNKNOWN_DIPLOC"):
                mask = int(args[0], 0)
                ports[cur] = [f for f in ports[cur] if not (f["mask"] & mask)]
                if mac == "PORT_BIT":
                    low = args[1] == "IP_ACTIVE_LOW"
                    t = args[2].replace("IPT_", "")
                    if t in TRACKBALL_TYPES:
                        low = False
                    fld = dict(mask=mask, kind="unused" if t in IGNORED_TYPES else "input",
                               default=mask if low else 0, type=t, player=1, name=None, settings=[])
                elif mac in ("PORT_DIPNAME", "PORT_CONFNAME"):
                    fld = dict(mask=mask, kind="dip", default=int(args[1], 0), name=def_str(args[2]), settings=[])
                elif mac == "PORT_SERVICE":
                    off = idle_level(args[1], mask)
                    fld = dict(mask=mask, kind="dip", default=off, name="Service Mode",
                               settings=[(off, "Off"), (off ^ mask, "On")])
                elif mac == "PORT_SERVICE_NO_TOGGLE":
                    fld = dict(mask=mask, kind="input", default=idle_level(args[1], mask), type="SERVICE", player=0,
                               name=None, settings=[])
                else:
                    d = {"IP_ACTIVE_LOW": mask, "IP_ACTIVE_HIGH": 0}.get(args[1])
                    fld = dict(mask=mask, kind="unused", default=int(args[1], 0) if d is None else d)
                ports[cur].append(fld)
            elif mac in ("PORT_DIPSETTING", "PORT_CONFSETTING"):
                fld["settings"].append((int(args[0], 0), def_str(args[1])))
            elif mac == "PORT_PLAYER":
                fld["player"] = int(args[0])
            elif mac == "PORT_COCKTAIL":
                fld["player"] = 2
            elif mac == "PORT_REVERSE":
                fld["reverse"] = True
            elif mac == "PORT_NAME":
                fld["name"] = args[0].strip('"')
            elif mac in ("PORT_DIPLOCATION", "PORT_CODE", "PORT_8WAY", "PORT_4WAY", "PORT_CONDITION", "PORT_MINMAX",
                         "PORT_SENSITIVITY", "PORT_KEYDELTA", "PORT_CUSTOM_MEMBER", "PORT_READ_LINE_DEVICE_MEMBER",
                         "PORT_CENTERDELTA", "PORT_IMPULSE", "PORT_TOGGLE"):
                pass
            else:
                raise SystemExit(f"INPUT_PORTS({name}): unhandled {mac}")
        cache[name] = ports
        return copy.deepcopy(ports)

    return build


def place(segs, g):
    """-> list of segments with ioctl index 0 addresses; adds the sync PROM when the set has none"""
    out, have_sync = [], False
    gfx = [s for s in segs if s["region"] == "gfx1"]
    half = (gfx[0]["rsize"] // 2) if gfx else 0
    for s in segs:
        r = s["region"]
        if r in IGNORED_REGIONS:
            continue
        if r in ("maincpu", "user1"):
            if r == "maincpu" and g["machine"] == "multiped":
                continue
            out.append(dict(s, addr=CPU_BASE + s["dst"]))
        elif r == "gfx1":
            if s["dst"] < half < s["dst"] + s["len"]:        # one file holds both planes (Warlords): split it
                n = half - s["dst"]
                out.append(dict(s, len=n, addr=GFX_LO + s["dst"], partial=True))
                out.append(dict(s, src=s["src"] + n, dst=half, len=s["len"] - n, addr=GFX_HI, partial=True))
            elif s["dst"] < half:
                out.append(dict(s, addr=GFX_LO + s["dst"]))
            else:
                out.append(dict(s, addr=GFX_HI + s["dst"] - half))
        elif r == "proms":
            if s["crc"] == SYNC_PROM[1]:
                out.append(dict(s, addr=SYNC_BASE))
                have_sync = True
            elif s["crc"] in UNUSED_PROMS:
                continue
            else:
                out.append(dict(s, addr=COLOR_BASE + s["dst"]))
        else:
            raise SystemExit(f'{g["name"]}: unmapped region {r}')
    if not have_sync:
        out.append(dict(name=SYNC_PROM[0], crc=SYNC_PROM[1], src=0, len=0x100, flen=0x100, addr=SYNC_BASE,
                        extra_zip=SYNC_PROM[2]))
    out.sort(key=lambda s: s["addr"])
    for a, b in zip(out, out[1:]):
        assert a["addr"] + a["len"] <= b["addr"], f'{g["name"]}: overlap {a["name"]} / {b["name"]}'
    return out


def whole_file(seg, segs):
    return (sum(1 for s in segs if s["name"] == seg["name"] and s["crc"] == seg["crc"]) == 1 and seg["src"] == 0
            and seg["len"] == seg["flen"] and not seg.get("partial"))

# ---------------------------------------------------------------- inputs -> MRA

def input_config(g, ports):
    """-> (idle bytes, dip list, dip defaults (3 bytes), input map, P1 button names)"""
    idle, dips, imap, buttons = [], [], [0] * 48, {}
    dbit, ddef = 0, 0
    for p, names in enumerate(PORTS):
        tag = next((n for n in names if n in ports), names[0])
        fields = ports.get(tag)
        if fields is None:
            idle.append(0xFF)
            continue
        level = 0xFF
        for f in fields:
            level = (level & ~f["mask"]) | (f["default"] & f["mask"])
            if f["kind"] == "dip":
                lo = (f["mask"] & -f["mask"]).bit_length() - 1
                hi = f["mask"].bit_length() - 1
                if f["mask"] != ((1 << (hi + 1)) - (1 << lo)):
                    raise SystemExit(f'{g["name"]}: non-contiguous DIP {f["name"]} mask {f["mask"]:#x}')
                w = hi - lo + 1
                vals = [v >> lo for v, _ in f["settings"]]
                ids = ",".join(n.replace(",", ";") for _, n in f["settings"])
                bits = f"{dbit}" if w == 1 else f"{dbit},{dbit + w - 1}"
                seq = vals == list(range(len(vals))) and len(vals) == 1 << w
                dips.append((f["name"], bits, ids, None if seq else ",".join(str(v) for v in vals)))
                for i in range(w):
                    imap[p * 8 + lo + i] = 0x80 | (dbit + i)
                ddef |= ((f["default"] >> lo) & ((1 << w) - 1)) << dbit
                dbit += w
            elif f["kind"] == "input":
                t, pl = f["type"], f["player"]
                if t in ("CUSTOM", "SPECIAL"):
                    continue                           # board-supplied (VBLANK, trackball) or fixed
                key = (t, pl if t.startswith(("JOYSTICK", "BUTTON")) else 0)
                if key not in CTL:
                    raise SystemExit(f'{g["name"]}: unmapped input {t} player {pl} in {tag}')
                if f["mask"] & (f["mask"] - 1):
                    raise SystemExit(f'{g["name"]}: multi-bit input {t} in {tag}')
                imap[p * 8 + f["mask"].bit_length() - 1] = CTL[key]
                if t.startswith("BUTTON") and pl == 1:
                    buttons[int(t[6:])] = f["name"] or ("Fire" if t == "BUTTON1" else f"Button {t[6:]}")
        idle.append(level & 0xFF)
    if dbit > 24:
        raise SystemExit(f'{g["name"]}: {dbit} DIP bits; Main_MiSTer only keeps 3 default bytes reliably')
    return idle, dips, [(ddef >> (8 * i)) & 0xFF for i in range(3)], imap, buttons


def uses_trackball(ports):
    return any(f["kind"] == "input" and f["type"] in TRACKBALL_TYPES
               for t in ("TRACK0_X", "TRACK0_Y") for f in ports.get(t, []))


def config_bytes(g, ports):
    idle, dips, ddef, imap, buttons = input_config(g, ports)
    flags = (F_TB if uses_trackball(ports) else 0) | (F_PADDLE if "PADDLE0" in ports else 0) | \
            (F_STICK if "STICK0" in ports else 0) | (F_VERT if g["rot"] in ("ROT90", "ROT270") else 0) | \
            (0 if any(i in (1, 2, 3, 4, 9, 10, 11, 12) for i in imap) else F_NOJOY) | \
            (F_ROT90 if g["rot"] == "ROT90" else 0)
    cfg = [SUPPORTED[(g["machine"], g["init"])], flags, 0, 0] + idle + [0] * 6 + imap
    return cfg, idle, dips, ddef, imap, buttons, flags


def mra(g, games, segs, build_inputs):
    ports = build_inputs(g["inputs"])
    cfg, idle, dips, ddef, imap, buttons, flags = config_bytes(g, ports)
    nbtn = max(buttons) if buttons else 0
    names = ",".join([buttons.get(i, "Not Used").replace(",", " ") for i in range(1, 5)] + ["Coin", "Start 1P", "Start 2P", "Pause"])
    parent = g["parent"] or g["name"]
    zips = [g["name"]] + ([g["parent"]] if g["parent"] else [])
    for s in segs:
        if s.get("extra_zip") and s["extra_zip"] not in zips:
            zips.append(s["extra_zip"])
    zipname = "|".join(f"{z}.zip" for z in zips)
    rotation = {"ROT0": "horizontal", "ROT270": "vertical (ccw)", "ROT90": "vertical (cw)"}[g["rot"]]
    bootleg = "yes" if "bootleg" in g["manuf"].lower() or "hack" in g["desc"].lower() else "no"
    top = games.get(parent, g)
    series = title_case(clean_title(top["desc"]))

    def part_lines(isegs):
        lines, pos = [], 0
        for s in isegs:
            if s["addr"] > pos:
                lines.append(f'        <part repeat="0x{s["addr"] - pos:X}">00</part>')
            if whole_file(s, isegs):
                lines.append(f'        <part crc="{s["crc"]}" name="{s["name"]}"/>')
            else:
                lines.append(f'        <part crc="{s["crc"]}" name="{s["name"]}" offset="0x{s["src"]:X}" length="0x{s["len"]:X}"/>')
            pos = s["addr"] + s["len"]
        return "\n".join(lines)

    dip_lines = "\n".join(f'        <dip name="{n}" bits="{b}" ids="{i}"' + (f' values="{v}"' if v else "") + "/>"
                          for n, b, i, v in dips)
    cfg_rows = "\n".join("            " + " ".join(f"{b:02X}" for b in cfg[i:i + 16]) for i in range(0, len(cfg), 16))
    players = "4 (simultaneous)" if any(i in (24, 25) for i in imap) else "2 (alternating)"
    return f"""<misterromdescription>
    <name>{display_name(g)}</name>
    <region>World</region>
    <homebrew>no</homebrew>
    <bootleg>{bootleg}</bootleg>
    <version></version>
    <alternative></alternative>
    <platform></platform>
    <series>{series}</series>
    <year>{g["year"]}</year>
    <manufacturer>{g["manuf"]}</manufacturer>
    <category>Shooter</category>

    <setname>{g["name"]}</setname>
    <parent>{parent}</parent>
    <mameversion>{MAME_VERSION}</mameversion>
    <rbf>{RBF}</rbf>
    <about></about>

    <resolution>15kHz</resolution>
    <rotation>{rotation}</rotation>
    <flip>yes</flip>

    <players>{players}</players>
    <joystick>8-way</joystick>
    <special_controls>{"trackball" if flags & F_TB else "paddle" if flags & F_PADDLE else "analog stick" if flags & F_STICK else ""}</special_controls>
    <num_buttons>{nbtn}</num_buttons>
    <buttons names="{names}" default="A,B,X,Y,Select,Start,R,L"/>

    <switches default="{",".join(f"{b:02X}" for b in ddef)}">
{dip_lines}
    </switches>

    <!-- Index 0: CPU 0x00000, gfx low plane 0x10000, gfx high plane 0x12000, sync PROM 0x14000, color PROM 0x14100 -->
    <rom index="0" md5="none" zip="{zipname}">
{part_lines(segs)}
    </rom>

    <!-- Index 1: board variant, flags, port idle levels, input map (see Arcade-Centipede.sv) -->
    <rom index="1">
        <part>
{cfg_rows}
        </part>
    </rom>

    <remark>{remark(g)}</remark>
    <mratimestamp>20261012000000</mratimestamp>
</misterromdescription>
"""


def title_case(text):
    out = []
    for word in re.split(r"(\s+|[()/,-])", text):
        if word and word[0].isalpha():
            word = word[0].upper() + word[1:]
        out.append(word)
    return "".join(out)


def clean_title(desc):
    return re.sub(r"(\s*\([^()]*\))+$", "", desc).strip()


def display_name(g):
    return title_case(clean_title(g["desc"]) if g["parent"] is None else g["desc"])


def remark(g):
    """'Title (Maker)', without repeating a maker the title already names."""
    m = re.fullmatch(r'(?:bootleg|hack) \((.+)\)', g["manuf"])
    maker = m.group(1) if m else g["manuf"]
    desc = title_case(g["desc"])
    return desc if maker.lower() in g["desc"].lower() else f"{desc} ({title_case(maker)})"


def safe(name):
    return re.sub(r'\s*/\s*', " - ", re.sub(r'[\\:*?"<>|]', "-", name))


def out_path(root, g, games):
    if g["parent"] is None:
        return root / f"{safe(display_name(g))}.mra"
    parent_title = display_name(games[g["parent"]]) if g["parent"] in games else title_case(clean_title(g["desc"]))
    return root / "_alternatives" / f"_{safe(parent_title)}" / f"{safe(display_name(g))}.mra"


def main():
    src = "\n".join(Path(f).read_text() for f in sys.argv[1].split(","))
    out_dir = Path(sys.argv[2])
    games = parse_games(src)
    build_inputs = parse_inputs(src)
    names = sys.argv[3:] or [n for n, g in games.items() if (g["machine"], g["init"]) in SUPPORTED and g["working"]]
    for name in names:
        g = games[name]
        if (g["machine"], g["init"]) not in SUPPORTED:
            raise SystemExit(f'{name}: board {g["machine"]} / {g["init"]} not implemented')
        segs = place(parse_roms(src, name), g)
        path = out_path(out_dir, g, games)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(mra(g, games, segs, build_inputs))
        print(f'{name:12s} {path.relative_to(out_dir)}')


if __name__ == "__main__":
    main()

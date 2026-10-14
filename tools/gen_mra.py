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
import xml.etree.ElementTree as ET
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
    ("ccastles", "empty_init"): 10,
}
VARIANT_BY_INPUTS = {"ccastlesj": 11}               # Crystal Castles (joystick version): stick on LETA 0

IGNORED_REGIONS = {"plds"}
UNUSED_PROMS = {"ba40e1a9", "a5101624",            # millpac / magworma PROMs, "not used" in MAME
                "d841b7e0"}                         # bullsdrt 82s147, "unknown" in MAME

# Saves (index 1 byte 2): the board's own save chip where the game uses it (EAROM accessed at boot in the sim:
# every set but Magic Worm, 2026-10-13), else hiscore.dat RAM ranges through hiscore.v.
CHIP_NVRAM = {0: 64, 1: 64, 2: 64, 3: 64, 5: 64, 8: 64, 9: 64,    # ER2055 EAROM
              6: 128,                                             # Multipede 93C46
              10: 256, 11: 256}                                   # Crystal Castles X2212 pair
HISCORE_DAT = Path("/CybertronMD/Mame/plugins/hiscore/hiscore.dat")
HISCORE_HEADER_TAIL = "00 FF 00 02 00 02 00 01 11 11 00 00"
# START_WAIT = last boot-time write to the hiscore.dat ranges (MAME write taps, 30 s, 2026-10-13) + 2 frames
HISCORE_INIT_FRAME = {"warlords": 2, "magworm": 0}
CLK_PER_FRAME = 384 * 8 * 263                                     # Centipede board, 48.384 MHz

# MAME artwork (rtl/art/artwork.sv): set -> games/mame-artwork/<zip>, used as-is. The MRA sends a 1024-byte layout
# table (from the zip's default.lay) then each PNG behind an "AWI"+id marker on ioctl index 5. The OSD Artwork page
# picks the view (View n = the n-th view of default.lay, listed in the MRA) and MAME's layer toggles.
ARTWORK = {"warlords": "warlords", "ccastles": "ccastles", "centiped": "centiped", "milliped": "milliped"}
ART_DIR = Path(__file__).resolve().parent.parent / "games" / "mame-artwork"
ART_SCREEN_W = 512          # canvas pixels across the game screen (HDMI framebuffer); 2x native fits the DDR budget
ART_SCREEN_H = 512          # vertical games: canvas pixels down the (portrait) screen, same per-frame load as 512 across
ART_ROT = {"ROT0": 0, "ROT270": 1, "ROT90": 2}   # layout table byte 10
ART_NATIVE = (256, 240)     # game pixels (MAME visarea) = the native composite plane
ART_NATIVE_BY = {"ccastles": (256, 232)}     # boards whose visarea differs (ccastles: sync PROM vblank, lines 24-255)

F_TB, F_PADDLE, F_STICK, F_NOJOY, F_VERT, F_ROT90 = 0x01, 0x02, 0x04, 0x08, 0x10, 0x80

# port order on the board: index 1 idle bytes 4-9, input map bytes 16-63
PORTS = [("IN0",), ("IN1",), ("IN2",), ("IN3",), ("DSW1",), ("DSW2",)]
PORTS_BY_MACHINE = {"ccastles": [("IN0",), ("IN1",), ("LETA0",), ("LETA1",), ("LETA2",), ("LETA3",)]}
# Crystal Castles PROM regions (ccastles_board.sv); "addressprom" is not used by MAME
PROM_REGIONS = {"syncprom": SYNC_BASE, "wpprom": COLOR_BASE, "priprom": COLOR_BASE + 0x100}
BANK_HIGH = {"ccastles": 0x2000}                    # maincpu 10000-13FFF (banked A000-DFFF) -> CPU ROM 2000

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
                removed = [f for f in ports[cur] if f["mask"] & mask]
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
            elif mac == "PORT_CONDITION":
                # a conditional bit (Crystal Castles: jump / start by cabinet): both stay, input_config picks
                fld["cond"] = (args[0].strip('"'), int(args[1], 0), args[2], int(args[3], 0))
                ports[cur] += [f for f in removed if "cond" in f]
            elif mac in ("PORT_DIPLOCATION", "PORT_CODE", "PORT_8WAY", "PORT_4WAY", "PORT_MINMAX",
                         "PORT_SENSITIVITY", "PORT_KEYDELTA", "PORT_CUSTOM_MEMBER", "PORT_READ_LINE_DEVICE_MEMBER",
                         "PORT_CENTERDELTA", "PORT_IMPULSE", "PORT_TOGGLE", "PORT_READ_LINE_MEMBER"):
                pass
            else:
                raise SystemExit(f"INPUT_PORTS({name}): unhandled {mac}")
        cache[name] = ports
        return copy.deepcopy(ports)

    return build


def place(segs, g):
    """-> list of segments with ioctl index 0 addresses; adds the sync PROM when the set has none"""
    out, have_sync = [], False
    gfx = [s for s in segs if s["region"] in ("gfx1", "gfx")]
    half = (gfx[0]["rsize"] // 2) if gfx else 0
    for s in segs:
        r = s["region"]
        if r in IGNORED_REGIONS:
            continue
        if r in ("maincpu", "user1"):
            if r == "maincpu" and g["machine"] == "multiped":
                continue
            if s["dst"] >= 0x10000:
                out.append(dict(s, addr=CPU_BASE + s["dst"] - 0x10000 + BANK_HIGH[g["machine"]]))
            else:
                out.append(dict(s, addr=CPU_BASE + s["dst"]))
        elif r in PROM_REGIONS:
            out.append(dict(s, addr=PROM_REGIONS[r] + s["dst"]))
            have_sync |= r == "syncprom"
        elif r == "addressprom":
            continue
        elif r in ("gfx1", "gfx"):
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
    defaults = {tag: sum(f["default"] & f["mask"] for f in fl if "cond" not in f) for tag, fl in ports.items()}

    def cond_ok(f):
        if "cond" not in f:
            return True
        tag, mask, op, val = f["cond"]
        v = defaults.get(tag, 0) & mask
        return v == val if op == "EQUALS" else v != val

    for p, names in enumerate(PORTS_BY_MACHINE.get(g["machine"], PORTS)):
        tag = next((n for n in names if n in ports), names[0])
        fields = ports.get(tag)
        if fields is None:
            idle.append(0xFF)
            continue
        level = 0xFF
        for f in fields:
            if not cond_ok(f):
                continue
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
                if t in ("CUSTOM", "SPECIAL") or t in TRACKBALL_TYPES:
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
    return any(f["kind"] == "input" and f["type"] in TRACKBALL_TYPES for fl in ports.values() for f in fl)


def config_bytes(g, ports):
    idle, dips, ddef, imap, buttons = input_config(g, ports)
    flags = (F_TB if uses_trackball(ports) else 0) | (F_PADDLE if "PADDLE0" in ports else 0) | \
            (F_STICK if "STICK0" in ports else 0) | (F_VERT if g["rot"] in ("ROT90", "ROT270") else 0) | \
            (0 if any(i in (1, 2, 3, 4, 9, 10, 11, 12) for i in imap) else F_NOJOY) | \
            (F_ROT90 if g["rot"] == "ROT90" else 0)
    variant = VARIANT_BY_INPUTS.get(g["inputs"], SUPPORTED[(g["machine"], g["init"])])
    save = 2 if variant in CHIP_NVRAM else 1 if HISCORES.get(g["name"]) else 0
    cfg = [variant, flags, save, 0] + idle + [0] * 6 + imap
    return cfg, idle, dips, ddef, imap, buttons, flags


def parse_hiscores(path):
    """hiscore.dat -> {set: [entry lines]}; sets listed together share the entry lines below them."""
    out, names, body = {}, [], False
    for line in path.read_text(encoding="latin-1").splitlines():
        line = line.strip()
        if not line or line.startswith(";"):
            names, body = ([], False) if not line else (names, body)
            continue
        if line.endswith(":"):
            if body:
                names, body = [], False
            names.append(line[:-1])
        elif line.startswith("@"):
            for n in names:
                out.setdefault(n, []).append(line)
            body = True
    return out


HISCORES = parse_hiscores(HISCORE_DAT) if HISCORE_DAT.exists() else {}


def art_table(zipname, rot="ROT0"):
    """MAME default.lay -> (1024-byte table, [png names], [view names]); see rtl/art/artwork.sv for the format.
    rot = the set's orientation: the layout is in screen orientation (portrait for ROT90 / ROT270), the core's
    picture is the unrotated raster (ART_NATIVE). Views keep the screen and canvas on even pixels both
    ways, so the core can also draw them turned 90 (OSD Orientation Horz).
    Layer record: op, image (rect: R), flags, colour alpha, X, Y, W, H, X2 (rect: G, B), W2.
    Flags: [0] flipx, [1] alpha from mask, [3:2] kind (OSD toggle), [4] bezel-off rect, [5] side art, [6] rect."""
    import struct, zipfile
    z = zipfile.ZipFile(ART_DIR / f"{zipname}.zip")
    native = ART_NATIVE_BY.get(zipname, ART_NATIVE)
    lay = ET.fromstring(z.read("default.lay"))

    def png_size(name):
        d = z.read(name)
        return struct.unpack(">II", d[16:24])

    def rgba(c):
        return [float(c.get(k, "1")) if c is not None else 1.0 for k in ("red", "green", "blue", "alpha")]

    def _bounds(e):
        b = e.find("bounds")
        if b is None:
            return None
        if b.get("left") is not None:
            return tuple(float(b.get(k)) for k in ("left", "top", "right", "bottom"))
        x, y = float(b.get("x", "0")), float(b.get("y", "0"))
        return x, y, x + float(b.get("width")), y + float(b.get("height"))

    def bounds(n):
        b = _bounds(n)
        return None if b is None else (b[0], b[1], b[2] - b[0], b[3] - b[1])

    # element -> ("img", file, alphafile) | ("rects", [(x0, y0, x1, y1 in element units 0-1, rgba)])
    elems = {}
    for e in lay.findall("element"):
        name, kids = e.get("name"), list(e)
        imgs = [k for k in kids if k.tag == "image"]
        rects = [k for k in kids if k.tag == "rect"]
        if imgs and len(imgs) == len(kids):
            defstate = e.get("defstate", "0")
            pick = [i for i in imgs if i.get("state", defstate) == defstate]
            if not pick:                  # a lamp: no image in its default state (outputs are not driven yet)
                print(f"artwork {zipname}: element {name} has no image for state {defstate}, skipped")
                continue
            elems[name] = ("img", pick[0].get("file"), pick[0].get("alphafile"))
        elif rects and len(rects) == len(kids):
            ib = [_bounds(r) or (0.0, 0.0, 1.0, 1.0) for r in rects]     # MAME default: the unit square
            ix0, iy0 = min(b[0] for b in ib), min(b[1] for b in ib)
            ix1, iy1 = max(b[2] for b in ib), max(b[3] for b in ib)
            elems[name] = ("rects", [((b[0] - ix0) / (ix1 - ix0), (b[1] - iy0) / (iy1 - iy0), (b[2] - ix0) / (ix1 - ix0),
                                      (b[3] - iy0) / (iy1 - iy0), rgba(r.find("color"))) for r, b in zip(rects, ib)])
        else:
            print(f"artwork {zipname}: element {name} is not plain images or rects, skipped")

    images, views, layers = [], [], []

    def img_id(name):
        if name not in images:
            images.append(name)
        return images.index(name)

    OP_MUL, OP_ADD, OP_ALPHA, OP_UNDER, OP_MASK = 0, 1, 2, 3, 4
    groups = {g.get("name"): g for g in lay.findall("group")}

    def flatten(node):
        # <group ref>: the group's items mapped from its bounds (or the union of its items) into the reference's bounds
        out = []
        for it in node:
            if it.tag == "group" and it.get("ref") in groups:
                inner = flatten(groups[it.get("ref")])
                gb = bounds(groups[it.get("ref")])
                if gb is None:
                    bs = [bounds(i) for i in inner]
                    x0, y0 = min(b[0] for b in bs), min(b[1] for b in bs)
                    gb = (x0, y0, max(b[0] + b[2] for b in bs) - x0, max(b[1] + b[3] for b in bs) - y0)
                tb = bounds(it) or gb
                kx, ky = tb[2] / gb[2], tb[3] / gb[3]
                for i in inner:
                    b = bounds(i)
                    c = copy.deepcopy(i)
                    for old_b in c.findall("bounds"):
                        c.remove(old_b)
                    ET.SubElement(c, "bounds", x=str(tb[0] + (b[0] - gb[0]) * kx), y=str(tb[1] + (b[1] - gb[1]) * ky),
                                  width=str(b[2] * kx), height=str(b[3] * ky))
                    out.append(c)
            elif it.tag != "group" and bounds(it) is not None:
                out.append(it)
        return out

    for v in lay.findall("view"):
        items = {k: [] for k in ("backdrop", "screen", "overlay", "bezel", "cpanel", "marquee")}
        for it in flatten(v):
            if it.tag in items:
                items[it.tag].append(it)
        if len(items["screen"]) != 1:
            print(f"artwork {zipname}: view {v.get('name')} needs exactly one screen, skipped")
            continue
        vb = bounds(v)
        if vb is None:      # union of every item
            bs = [bounds(i) for k in items for i in items[k]]
            x0, y0 = min(b[0] for b in bs), min(b[1] for b in bs)
            vb = (x0, y0, max(b[0] + b[2] for b in bs) - x0, max(b[1] + b[3] for b in bs) - y0)
        sb = bounds(items["screen"][0])
        vert = rot != "ROT0"
        sc = ART_SCREEN_H / sb[3] if vert else ART_SCREEN_W / sb[2]
        cw, ch = (round(vb[2] * sc) + 1) & ~1, round(vb[3] * sc)
        ox = round((sb[0] - vb[0]) * sc) & 1        # shift the view so the screen starts on an even pixel
        oy = round((sb[1] - vb[1]) * sc) & 1        # and an even row (the view turned 90)
        sx = round((sb[0] - vb[0]) * sc) - ox
        sy = round((sb[1] - vb[1]) * sc) - oy
        sw, sh = (round(sb[2] * sc / 2) * 2, ART_SCREEN_H) if vert else (ART_SCREEN_W, round(sb[3] * sc / 2) * 2)
        ch = (ch + 1) & ~1
        gw, gh = native[::-1] if vert else native        # game picture as the layout shows it
        assert cw <= 2048 and ch <= 2048 and sw >= gw and sh >= gh, v.get("name")
        if items["overlay"] or len(items["backdrop"]) <= 1:
            order = [(OP_MUL, "overlay"), (OP_ADD, "backdrop"), (OP_ALPHA, "bezel"), (OP_ALPHA, "cpanel"), (OP_ALPHA, "marquee")]
        else:
            order = [(OP_UNDER, "backdrop"), (OP_ALPHA, "bezel"), (OP_ALPHA, "cpanel"), (OP_ALPHA, "marquee")]
        KIND = {"backdrop": 0, "overlay": 1, "bezel": 2, "cpanel": 2, "marquee": 2}     # OSD toggle per layer
        first = len(layers)
        for op, kind in order:
            for it in items[kind]:
                if it.get("element") not in elems:
                    continue
                el = elems[it.get("element")]
                b = bounds(it)
                col = rgba(it.find("color"))
                o = it.find("orientation")
                flipx = o is not None and o.get("flipx") == "yes"
                if o is not None and (o.get("rotate", "0") != "0" or o.get("flipy") == "yes"):
                    print(f"artwork {zipname}: {it.get('element')} rotate / flipy not supported, drawn unrotated")

                def place(x0, y0, x1, y1):
                    x = round((x0 - vb[0]) * sc); y = round((y0 - vb[1]) * sc)
                    return x - ox, y - oy, round((x1 - vb[0]) * sc) - x, round((y1 - vb[1]) * sc) - y

                k = KIND[kind] << 2
                if el[0] == "rects":      # one solid-colour layer per rect, item colour x rect colour
                    for rx0, ry0, rx1, ry1, rc in el[1]:
                        x, y, w, h = place(b[0] + rx0 * b[2], b[1] + ry0 * b[3], b[0] + rx1 * b[2], b[1] + ry1 * b[3])
                        if w <= 0 or h <= 0:
                            continue
                        r, gr, bl, a = (round(255 * rc[i] * col[i]) for i in range(4))
                        side = kind != "bezel" and (x + w <= sx or x >= sx + sw)
                        layers.append((op, r, 64 | k | (32 if side else 0), a, x, y, w, h, gr | bl << 8, 0))
                    continue
                if col[:3] != [1.0] * 3:
                    print(f"artwork {zipname}: {it.get('element')} colour tint not supported, drawn untinted")
                _, f, af = el
                x, y, w, h = place(b[0], b[1], b[0] + b[2], b[1] + b[3])
                ca = round(255 * col[3])
                assert w <= 2048, f
                for name in [n for n in (af, f) if n]:
                    pw, ph = png_size(name)
                    assert pw <= 15 * w and ph <= 15 * h, f"{name}: downscale factor over 15"
                # side art beside the screen and taller than it: with bezels off it is scaled to the screen's height
                # (flag bit 4, alternate x / width; y / height = the screen's), so the game can fill the display
                alt, x2, w2 = 0, 0, 0
                side = kind != "bezel" and (x + w <= sx or x >= sx + sw)     # side art: OSD Side Art toggle (flag bit 5)
                if side:
                    k |= 32
                if side and h > sh:
                    w2 = max(1, round(w * sh / h))
                    x2 = x + w - w2 if x + w <= sx else x
                    alt = 16
                    for name in [n for n in (af, f) if n]:
                        pw, ph = png_size(name)
                        assert pw <= 15 * w2 and ph <= 15 * sh, f"{name}: downscale factor over 15 (bezel off)"
                if af:
                    layers.append((OP_MASK, img_id(af), k | alt, 255, x, y, w, h, x2, w2))
                layers.append((op, img_id(f), (1 if flipx else 0) | (2 if af else 0) | k | alt, ca, x, y, w, h, x2, w2))
        views.append((v.get("name"), (cw, ch, sx, sy, sw, sh, first, len(layers) - first)))
    assert len(views) <= 7 and len(layers) <= 48 and len(images) <= 32, (len(views), len(layers), len(images))
    t = bytearray(1024)
    struct.pack_into("<2sHHHHB", t, 0, b"AW", len(views), len(images), *native, ART_ROT[rot])
    for i, (_, rec) in enumerate(views):
        struct.pack_into("<8H", t, 16 + 16 * i, *rec)
    for i, (op, img, fl, ca, x, y, w, h, x2, w2) in enumerate(layers):
        struct.pack_into("<BBBBhhHHHH" if fl & 64 else "<BBBBhhHHhH", t, 256 + 16 * i, op, img, fl, ca, x, y, w, h, x2, w2)
    return bytes(t), images, [n for n, _ in views]


def art_xml(zipname, rot):
    table, images, views = art_table(zipname, rot)
    names = ", ".join(f"View {i + 1} = {n.replace('_', ' ')}" for i, n in enumerate(views))
    rows = "\n".join("            " + " ".join(f"{b:02X}" for b in table[i:i + 32]) for i in range(0, 1024, 32))
    parts = "\n".join(f'        <part>41 57 49 {i:02X}</part>\n        <part name="{n}"/>' for i, n in enumerate(images))
    return f"""
    <!-- Index 5: MAME artwork, layout table (from default.lay) + PNGs from the artwork zip, as-is -->
    <!-- OSD Artwork: {names} -->
    <rom index="5" md5="none" zip="/mame-artwork/{zipname}.zip">
        <part>
{rows}
        </part>
{parts}
    </rom>
"""


def save_xml(g, cfg):
    if cfg[2] == 2:
        return f"""
    <!-- Index 4: the board's save chip ({CHIP_NVRAM[cfg[0]]} bytes), restored by the game itself -->
    <rom index="4"></rom>
    <nvram index="4" size="{CHIP_NVRAM[cfg[0]]}"></nvram>
"""
    if cfg[2] != 1:
        return ""
    ents = []
    for line in HISCORES[g["name"]]:
        f = line.split(",")
        if f[0] != "@:maincpu" or f[1] != "program":
            raise SystemExit(f'{g["name"]}: unsupported hiscore.dat line {line}')
        ents.append((int(f[2], 16), int(f[3], 16), int(f[4], 16), int(f[5], 16)))
    rows = "\n".join(f"            {a >> 24 & 0xFF:02X} {a >> 16 & 0xFF:02X} {a >> 8 & 0xFF:02X} {a & 0xFF:02X} "
                      f"{n >> 8:02X} {n & 0xFF:02X} {s:02X} {e:02X}" for a, n, s, e in ents)
    if g["name"] not in HISCORE_INIT_FRAME:
        raise SystemExit(f'{g["name"]}: no measured table-init frame for its hiscore layout')
    wait = (HISCORE_INIT_FRAME[g["name"]] + 2) * CLK_PER_FRAME
    header = " ".join(f"{b:02X}" for b in wait.to_bytes(4, "big")) + " " + HISCORE_HEADER_TAIL
    return f"""
    <!-- Index 3: hiscore config (MAME hiscore.dat), index 4: saved scores -->
    <rom index="3" md5="none">
        <part>
            {header}
{rows}
        </part>
    </rom>
    <rom index="4"></rom>
    <nvram index="4" size="{sum(n for _, n, _, _ in ents)}"></nvram>
"""


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

    art = ARTWORK.get(g["name"]) or ARTWORK.get(g["parent"] or "")
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

    <!-- Index 0: CPU 0x00000, gfx low plane 0x10000, gfx high plane 0x12000, sync PROM 0x14000, PROMs 0x14100 -->
    <rom index="0" md5="none" zip="{zipname}">
{part_lines(segs)}
    </rom>

    <!-- Index 1: board variant, flags, port idle levels, input map (see Arcade-Centipede.sv) -->
    <rom index="1">
        <part>
{cfg_rows}
        </part>
    </rom>
{save_xml(g, cfg)}{art_xml(art, g["rot"]) if art else ""}
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

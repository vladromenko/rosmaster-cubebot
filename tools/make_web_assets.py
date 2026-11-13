"""Build the web assets of the experiments docs page from the vendor URDF/STL: decimated meshes + urdf.json.

Run on the robot:  python tools/make_web_assets.py   (needs numpy only)
Vertex clustering: vertices are snapped to a grid (`CELL` m), merged, degenerate triangles dropped.
"""
import json
import os
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

SRC = os.path.expanduser("~/Explorer/vendor/m3_reference")
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "cubebot", "web", "static")
CELL = {"base_link": 0.0030, "default": 0.0020}


def read_stl(path):
    raw = open(path, "rb").read()
    n = struct.unpack("<I", raw[80:84])[0]
    rec = np.frombuffer(raw[84:84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return rec["v"].reshape(-1, 3).astype(np.float64)


def decimate(v, cell):
    q = np.round(v / cell).astype(np.int64)
    keys, inv = np.unique(q, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    sums = np.zeros((len(keys), 3))
    np.add.at(sums, inv, v)
    cnt = np.bincount(inv, minlength=len(keys))[:, None]
    verts = sums / cnt
    tri = inv.reshape(-1, 3)
    ok = (tri[:, 0] != tri[:, 1]) & (tri[:, 1] != tri[:, 2]) & (tri[:, 0] != tri[:, 2])
    return verts, tri[ok]


def write_stl(path, verts, tri):
    a, b, c = verts[tri[:, 0]], verts[tri[:, 1]], verts[tri[:, 2]]
    nrm = np.cross(b - a, c - a)
    nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tri), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    rec["n"], rec["v"][:, 0], rec["v"][:, 1], rec["v"][:, 2] = nrm, a, b, c
    with open(path, "wb") as fh:
        fh.write(b"decimated".ljust(80, b"\0") + struct.pack("<I", len(tri)) + rec.tobytes())


def num(s):
    return [float(x) for x in s.split()]


def main():
    os.makedirs(os.path.join(OUT, "meshes"), exist_ok=True)
    root = ET.parse(os.path.join(SRC, "M3Pro.urdf")).getroot()
    links, joints = {}, []
    for l in root.findall("link"):
        v = l.find("visual")
        mesh = None
        if v is not None and v.find("geometry/mesh") is not None:
            mesh = os.path.basename(v.find("geometry/mesh").get("filename"))
        links[l.get("name")] = mesh
    for j in root.findall("joint"):
        o, a = j.find("origin"), j.find("axis")
        joints.append(dict(name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
                           child=j.find("child").get("link"), xyz=num(o.get("xyz")), rpy=num(o.get("rpy")),
                           axis=num(a.get("xyz")) if a is not None else [0, 0, 1]))
    total = 0
    for name, mesh in links.items():
        if not mesh:
            continue
        v = read_stl(os.path.join(SRC, "meshes", mesh))
        verts, tri = decimate(v, CELL.get(name, CELL["default"]))
        write_stl(os.path.join(OUT, "meshes", mesh), verts, tri)
        size = os.path.getsize(os.path.join(OUT, "meshes", mesh))
        total += size
        print("%-16s %7d -> %6d tris  %6.0f KB  bbox %s" % (mesh, len(v) // 3, len(tri), size / 1024, np.round(v.max(0) - v.min(0), 3)))
    json.dump(dict(links=links, joints=joints), open(os.path.join(OUT, "urdf.json"), "w"), indent=1)
    print("total %.1f MB" % (total / 1e6))


if __name__ == "__main__":
    sys.exit(main())

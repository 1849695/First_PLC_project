# Computes the robot poses with the RG2 gripper mounted: the grasp point (attachPoint)
# must reach the cube center, with the gripper pointing down.
# Simulation stopped. Writes pose_pinza.json.
import json, math
import numpy as np
from scipy.optimize import least_squares
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

sim = RemoteAPIClient().require('sim')
robot  = sim.getObject('/UR5')
joints = sim.getObjectsInTree(robot, sim.object_joint_type, 0)[:6]
nomi = {}
for h in sim.getObjectsInTree(robot, sim.handle_all, 0):
    nomi.setdefault(sim.getObjectAlias(h, 0), h)
connection, attach = nomi['connection'], nomi['attachPoint']
left, right = nomi['leftTouch'], nomi['rightTouch']
cube = sim.getObject('/Cuboid')
W = sim.handle_world

def pos(h):
    return np.array(sim.getObjectPosition(h, W))

def vai(q):
    for j, a in zip(joints, q):
        sim.setJointPosition(j, float(a))

def asse_pinza():
    v = pos(attach) - pos(connection)
    return v / np.linalg.norm(v)

# --- preliminary checks ---
vai([0] * 6)
print("gripper axis at robot position 0:", np.round(asse_pinza(), 3))
mid = (pos(left) + pos(right)) / 2
print("center between fingertips - attachPoint (m):", np.round(mid - pos(attach), 3))
cubo = pos(cube)
print("current cube position:", np.round(cubo, 3))
base = pos(joints[0])
print("base (joint 1):", np.round(base, 3))

CUBO = np.array([-0.172, 0.598, 0.046])      # known starting position of the cube
SCARTO = float(np.linalg.norm(mid - pos(attach)))   # fingertips beyond the attachPoint (m)
PRESA = CUBO + [0, 0, SCARTO]     # gripper pointing down: attachPoint above the cube
print(f"fingertips-attachPoint offset: {SCARTO*100:.1f} cm -> attachPoint {SCARTO*100:.1f} cm above the cube center")
if np.linalg.norm(cubo - CUBO) > 0.02:
    print("WARNING: the cube is not at its starting position; using anyway", CUBO)

def ruota_z(p, gradi):
    a = math.radians(gradi)
    d = p - base
    return base + np.array([d[0]*math.cos(a) - d[1]*math.sin(a),
                            d[0]*math.sin(a) + d[1]*math.cos(a), d[2]])

def residuo(x, target):
    q = list(x) + [0.0]
    vai(q)
    return np.concatenate([pos(attach) - target, 0.3 * (asse_pinza() - np.array([0, 0, -1.0]))])

def risolvi(target, partenze):
    migliore = None
    for s in partenze:
        r = least_squares(residuo, s, args=(target,),
                          bounds=([-math.pi]*5, [math.pi]*5), xtol=1e-6, ftol=1e-6)
        if migliore is None or r.cost < migliore.cost:
            migliore = r
    return migliore

partenze = [[q1, 0.56, 1.99, q4, q5]
            for q1 in (0.0, 1.0, -1.0) for q4 in (-1.5, 0.0, 1.5) for q5 in (1.57, -1.57)]

risultati = {}
for nome, target in [
    ("PICK",        PRESA),
    ("ABOVE_PICK",  PRESA + [0, 0, 0.15]),
    ("PLACE",       ruota_z(PRESA, 70)),
    ("ABOVE_PLACE", ruota_z(PRESA, 70) + [0, 0, 0.15]),
]:
    r = risolvi(np.array(target), partenze)
    q = list(r.x) + [0.0]
    vai(q)
    errore = np.linalg.norm(pos(attach) - target) * 100
    print(f"{nome:<12} gradi={[round(math.degrees(a), 1) for a in q]}  "
          f"grasp error={errore:.2f} cm  asse={np.round(asse_pinza(), 2)}")
    risultati[nome] = {"gradi": [round(math.degrees(a), 2) for a in q],
                       "errore_cm": round(float(errore), 3)}

vai([0] * 6)
json.dump(risultati, open("pose_pinza.json", "w"), indent=2)
print("Wrote pose_pinza.json")

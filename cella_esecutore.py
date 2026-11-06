# Cell executor: the PLC decides the step (cmd_step); this script runs it in
# CoppeliaSim and answers the PLC by writing done_step.
#   cmd_step  = %QW0 -> register 0   (written by the PLC)
#   done_step = %MW0 -> register 16  (written by us)
#   run       = %MW1 -> register 17  (1 = go, 0 = stop; changed with plc_cmd.py)
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from pymodbus.client import ModbusTcpClient
import json, math, os, time

CMD_STEP, STATE, PEZZI_FATTI, SLOT, ULTIMO, DONE_STEP, RUN, PEZZI_TARGET = 0, 1, 2, 3, 4, 16, 17, 19
N_SLOT = 10                      # maximum number of cubes / positions (the PLC uses pezzi_fatti MOD 10)
SLOT_STEP = 12                   # degrees between neighbouring slots (~11 cm at the pick radius)
# Pick slots fan out from the original cube position; drop slots fan out the other way around
# the base, so the two arcs never overlap. Rotations are relative to the poses in pose_pinza.json
# (PICK base angle 7 deg, PLACE base angle 77 deg): pick = 7 + 12k, place = -40 - 12k.
ROT_PICK  = [SLOT_STEP * k for k in range(N_SLOT)]
ROT_PLACE = [-117 - SLOT_STEP * k for k in range(N_SLOT)]
NOMI_STATO = {0: 'IDLE', 1: 'RUNNING', 2: 'STOPPED', 3: 'ALARM'}
ULTIMO_PASSO = 10

# ---------- Modbus ----------
plc = ModbusTcpClient("localhost", port=502)
if not plc.connect():
    raise SystemExit("PLC not reachable on localhost:502 (Docker running? program in Run?)")

def leggi(addr):
    r = plc.read_holding_registers(addr, count=1)
    if r.isError():
        raise SystemExit(f"Modbus read error on register {addr}: {r}")
    return r.registers[0]

def scrivi(addr, valore):
    r = plc.write_register(addr, valore)
    if r.isError():
        raise SystemExit(f"Modbus write error on register {addr}: {r}")

# ---------- CoppeliaSim ----------
client = RemoteAPIClient()
sim = client.require('sim')
robot  = sim.getObject('/UR5')
joints = sim.getObjectsInTree(robot, sim.object_joint_type, 0)
nomi = {}
for h in sim.getObjectsInTree(robot, sim.handle_all, 0):
    nomi.setdefault(sim.getObjectAlias(h, 0), h)
attach = nomi['attachPoint']          # point where the cube is 'glued' to the gripper
cube   = sim.getObject('/Cuboid')
sim.setObjectInt32Param(cube, sim.shapeintparam_static, 1)
sim.setObjectInt32Param(cube, sim.shapeintparam_respondable, 0)
# The original cube defines the "0 degrees" position; the other cubes are copies rotated around the base
CUBO_POS0 = sim.getObjectPosition(cube, sim.handle_world)
CUBO_ORI0 = sim.getObjectOrientation(cube, sim.handle_world)
BASE = sim.getObjectPosition(joints[0], sim.handle_world)
cubi = []                        # filled after the simulation starts (copies disappear on stop)

def ruota_z(p, gradi):
    a = math.radians(gradi)
    dx, dy = p[0] - BASE[0], p[1] - BASE[1]
    return [BASE[0] + dx * math.cos(a) - dy * math.sin(a),
            BASE[1] + dx * math.sin(a) + dy * math.cos(a), p[2]]

def slot():
    """Which piece is being handled (0, 1, ...): decided by the PLC."""
    return leggi(SLOT) % len(cubi)

def posiziona_cubo(k):
    c = cubi[k]
    ori = list(CUBO_ORI0)
    ori[2] += math.radians(ROT_PICK[k])          # the cube rotates together with the gripper
    sim.setObjectParent(c, -1, True)
    sim.setObjectPosition(c, ruota_z(CUBO_POS0, ROT_PICK[k]), sim.handle_world)
    sim.setObjectOrientation(c, ori, sim.handle_world)

def cubi_voluti():
    """How many cubes the PLC target asks for (at least 1, at most N_SLOT)."""
    return max(1, min(leggi(PEZZI_TARGET), N_SLOT))

def sincronizza_cubi():
    """Create or remove cubes so that there is one per requested piece, all at their pick slot."""
    n = cubi_voluti()
    while len(cubi) < n:
        cubi.extend(sim.copyPasteObjects([cube], 0))     # copies disappear when the simulation stops
    if len(cubi) > n:
        sim.removeObjects(cubi[n:])
        del cubi[n:]
    for k in range(n):
        posiziona_cubo(k)
    print(f"  cubes on the table: {n}")

def nuovo_pezzo():
    k = slot()
    print(f"  current piece: {k + 1}/{len(cubi)}")
    posiziona_cubo(k)
    return True

# RG2 motor joint position (micrometers) with the fingers ~5 cm apart: tune by eye
GRIP_POS_UM = -1600

def deg(*angles):
    return [math.radians(a) for a in angles]

def move_to(target, tol=0.01, timeout=30):
    t0 = sim.getSimulationTime()
    while sim.getSimulationTime() - t0 < timeout:
        for j, q in zip(joints, target):
            sim.setJointTargetPosition(j, q)
        err = max(abs(sim.getJointPosition(j) - q) for j, q in zip(joints, target))
        if err < tol:
            return True
        time.sleep(0.05)
    return False

def attesa_sim(secondi):
    t0 = sim.getSimulationTime()
    while sim.getSimulationTime() - t0 < secondi:
        time.sleep(0.05)

def pinza(aperta, secondi=2.5):
    """Open (True) or close (False) the RG2 and wait until done (stroke ~2 s).
    Closed = the fingers stop at GRIP_POS_UM (cube width) instead of crossing the cube."""
    sim.setIntProperty(sim.handle_scene, 'signal.RG2_hold', 0 if aperta else 100000 + GRIP_POS_UM)
    sim.setIntProperty(sim.handle_scene, 'signal.RG2_open', 1 if aperta else 0)
    attesa_sim(secondi)

def grip():
    pinza(False)                                  # close the fingers
    sim.setObjectParent(cubi[slot()], attach, True)   # the cube follows the gripper
    return True

def release():
    sim.setObjectParent(cubi[slot()], -1, True)       # first detach the cube
    pinza(True)                                   # then open
    return True

# Poses computed by calcola_pose_pinza.py
_pose = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "pose_pinza.json")))
HOME = deg(0, 0, 0, 0, 0, 0)

def pose(nome):
    """Pose from pose_pinza.json with the base rotated according to the current piece slot."""
    g = list(_pose[nome]["gradi"])
    g[0] += (ROT_PICK if "PICK" in nome else ROT_PLACE)[slot()]
    return deg(*g)

def passo_inizio():
    if leggi(PEZZI_FATTI) == 0:        # first piece of a run: all cubes back at their pick slots
        sincronizza_cubi()
    nuovo_pezzo()
    if leggi(PEZZI_FATTI) == 0:        # first piece: start from home
        return move_to(HOME)
    return True                        # following pieces: stay where I am

def passo_fine():
    if leggi(ULTIMO) == 1:             # last piece: go home
        return move_to(HOME)
    return True                        # otherwise go straight to the next piece

# step number -> (name, action)
PASSI = {
    1:  ("new piece (home only for the first)", passo_inizio),
    2:  ("above the cube",          lambda: move_to(pose("ABOVE_PICK"))),
    3:  ("down",                   lambda: move_to(pose("PICK"))),
    4:  ("grasp",                  grip),
    5:  ("up",                      lambda: move_to(pose("ABOVE_PICK"))),
    6:  ("above the drop point", lambda: move_to(pose("ABOVE_PLACE"))),
    7:  ("down",                   lambda: move_to(pose("PLACE"))),
    8:  ("release",                release),
    9:  ("up",                      lambda: move_to(pose("ABOVE_PLACE"))),
    10: ("end of piece (home only after the last)", passo_fine),
}

# ---------- Main loop ----------
sim.startSimulation()
try:
    time.sleep(0.5)
    cubi.append(cube)
    sincronizza_cubi()
    pinza(True)                     # the RG2 starts closed: open it
    scrivi(RUN, 0)                  # no leftover START: it only starts with plc_cmd.py run
    scrivi(DONE_STEP, 0)            # new sequence: no step completed
    print("Executor ready. To start: python3 plc_cmd.py run   (stop: python3 plc_cmd.py stop)")
    avvisato_attesa = False
    ultimo_stato = None
    ultimi_pezzi = None
    while True:
        stato = leggi(STATE)
        if stato != ultimo_stato:
            print(f"[PLC state: {NOMI_STATO.get(stato, stato)}]")
            ultimo_stato = stato
        pezzi = leggi(PEZZI_FATTI)
        if pezzi != ultimi_pezzi:
            print(f"[pieces completed: {pezzi}]")
            ultimi_pezzi = pezzi
        cmd = leggi(CMD_STEP)
        done = leggi(DONE_STEP)
        if stato == 0 and cmd == 0 and cubi_voluti() != len(cubi):
            sincronizza_cubi()          # idle and the target changed: show the new number of cubes
        if cmd == 0:
            if done >= ULTIMO_PASSO:
                print("Sequence completed.")
                break
            if not avvisato_attesa:
                print(f"Waiting for the PLC (run={leggi(RUN)}, mode={'man' if leggi(20) else 'auto'}, last step done: {done})")
                avvisato_attesa = True
        elif cmd in PASSI and cmd == done + 1:
            avvisato_attesa = False
            nome, azione = PASSI[cmd]
            print(f"PLC requests step {cmd}: {nome}")
            ok = azione()
            if not ok:
                print(f"  TIMEOUT at step {cmd}: stopping without giving feedback")
                break
            scrivi(DONE_STEP, cmd)      # feedback to the PLC
        time.sleep(0.1)
finally:
    sim.stopSimulation()
    plc.close()

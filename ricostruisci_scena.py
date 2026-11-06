#!/usr/bin/env python3
"""Rebuild the CoppeliaSim scene used by this project (UR5 + RG2 gripper + cube).

CoppeliaSim in evaluation mode cannot save scenes, so this script recreates the scene
from scratch through the ZMQ remote API.

Usage:
  1. Open CoppeliaSim, File > New scene, simulation stopped.
  2. python3 ricostruisci_scena.py [--models /path/to/coppeliaSim/models]

What it does (every step is idempotent, so it is safe to run it twice):
  - loads the UR5 model and puts the base of joint 1 at BASE_TARGET
  - removes the example script bundled with the UR5 model (it moves the robot by itself)
  - loads the RG2 gripper and aligns it with the 'connection' dummy of the UR5 wrist
  - patches the RG2 Lua script with the 'RG2_hold' signal (fingers stop at a given width)
  - creates the 5 cm cube '/Cuboid' at CUBE_POS, static and NOT respondable
The scene lives only in memory: do not close CoppeliaSim until you are done.
"""
import argparse, glob, os, sys
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

BASE_TARGET = [0.0, 0.1, 0.023]        # world position of joint 1 of the UR5
CUBE_POS = [-0.172, 0.598, 0.046]      # cube center (the PLC cell rotates copies of it around the base)
CUBE_SIZE = 0.05
UR5_MODEL = "robots/non-mobile/UR5.ttm"
RG2_MODEL = "components/grippers/RG2.ttm"

ap = argparse.ArgumentParser()
ap.add_argument("--models", help="path of the CoppeliaSim 'models' folder")
args = ap.parse_args()

sim = RemoteAPIClient().require('sim')

if sim.getSimulationState() != sim.simulation_stopped:
    sys.exit("Stop the simulation first.")


def find_models_dir():
    """Folder that contains robots/non-mobile/UR5.ttm."""
    cands = [args.models, os.environ.get("COPPELIASIM_MODELS_DIR")]
    try:
        app = sim.getStringParam(sim.stringparam_application_path)
        cands += [os.path.join(app, "..", "Resources", "models"), os.path.join(app, "models")]
    except Exception:
        pass
    for root in ("/Applications", "~/Applications", "~/Desktop", "~/Downloads", "~"):
        root = os.path.expanduser(root)
        cands += glob.glob(os.path.join(root, "*oppelia*", "Contents", "Resources", "models"))
        cands += glob.glob(os.path.join(root, "*oppelia*", "models"))
        cands += glob.glob(os.path.join(root, "*oppelia*", "*", "Contents", "Resources", "models"))
    for c in cands:
        if c and os.path.isfile(os.path.join(os.path.expanduser(c), UR5_MODEL)):
            return os.path.abspath(os.path.expanduser(c))
    return None


def find(alias, root=None):
    """First object with this alias in the scene (or in the tree of root)."""
    top = sim.handle_scene if root is None else root
    for h in sim.getObjectsInTree(top, sim.handle_all, 0):
        if sim.getObjectAlias(h, 0) == alias:
            return h
    return None


models = None
ur5 = find("UR5")
rg2 = find("RG2")
if ur5 is None or rg2 is None:
    models = find_models_dir()
    if models is None:
        sys.exit("CoppeliaSim 'models' folder not found. Pass it with --models "
                 "(on macOS: /Applications/coppeliaSim.app/Contents/Resources/models), "
                 "or drag UR5 and RG2 from the model browser and run again.")
    print("models folder:", models)

# ---- UR5 ----
if ur5 is None:
    ur5 = sim.loadModel(os.path.join(models, UR5_MODEL))
    # the model comes with a script that moves the robot by itself: remove it
    try:
        sim.removeObjects([sim.getScript(sim.scripttype_simulation, ur5)])
        print("UR5 example script removed")
    except Exception as e:
        print("could not remove the UR5 example script (remove it by hand):", e)
    print("UR5 loaded")
joints = sim.getObjectsInTree(ur5, sim.object_joint_type, 0)
cur = sim.getObjectPosition(joints[0], sim.handle_world)
base = sim.getObjectPosition(ur5, sim.handle_world)
sim.setObjectPosition(ur5, [base[i] + BASE_TARGET[i] - cur[i] for i in range(3)], sim.handle_world)

# ---- RG2 on the wrist ----
conn = find("connection", ur5)
if conn is None:
    sys.exit("Dummy 'connection' not found inside the UR5.")
if rg2 is None:
    rg2 = sim.loadModel(os.path.join(models, RG2_MODEL))
    print("RG2 loaded")
sim.setObjectParent(rg2, conn, True)
sim.setObjectPosition(rg2, [0, 0, 0], conn)
sim.setObjectOrientation(rg2, [0, 0, 0], conn)

# ---- patch of the RG2 script: 'RG2_hold' signal ----
script = sim.getScript(sim.scripttype_simulation, rg2)
text = sim.getScriptStringParam(script, sim.scriptstringparam_text)
if 'RG2_hold' in text:
    print("RG2 script already patched")
else:
    nl = "\r\n" if "\r\n" in text else "\n"          # the original script uses CRLF
    anchor = "    sim.setJointTargetForce(motorHandle,motorForce)"
    lines = [
        "    -- NEW: hold the requested position (micrometers + 100000), 0 = disabled",
        "    local hold=sim.getIntProperty(sim.handle_scene, 'signal.RG2_hold', {noError = true})",
        "    if hold and hold~=0 then",
        "        local target=(hold-100000)/1000000",
        "        local err=target-sim.getJointPosition(motorHandle)",
        "        v=math.max(-motorVelocity, math.min(motorVelocity, 10*err))",
        "    end",
    ]
    if text.count(anchor) != 1:
        sys.exit(f"Patch anchor found {text.count(anchor)} times (need exactly 1): RG2 script not modified.")
    new = text.replace(anchor, nl.join(lines) + nl + anchor, 1)
    try:
        sim.setScriptStringParam(script, sim.scriptstringparam_text, new)
    except Exception:
        sim.setStringProperty(script, 'code', new)
    print("RG2 script patched:", 'RG2_hold' in sim.getScriptStringParam(script, sim.scriptstringparam_text))

# ---- cube ----
cube = find("Cuboid")
if cube is None:
    cube = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [CUBE_SIZE] * 3, 0)
    sim.setObjectAlias(cube, "Cuboid")
    try:
        sim.setShapeColor(cube, None, sim.colorcomponent_ambient_diffuse, [0.45, 0.45, 0.9])
    except Exception:
        pass
    print("cube created")
sim.setObjectPosition(cube, CUBE_POS, sim.handle_world)
sim.setObjectOrientation(cube, [0, 0, 0], sim.handle_world)
sim.setObjectInt32Param(cube, sim.shapeintparam_static, 1)
sim.setObjectInt32Param(cube, sim.shapeintparam_respondable, 0)

# ---- summary ----
r = lambda v: [round(x, 3) for x in v]
print("\nScene ready.")
print("  joint 1 :", r(sim.getObjectPosition(joints[0], sim.handle_world)), "(expected", BASE_TARGET, ")")
print("  cube    :", r(sim.getObjectPosition(cube, sim.handle_world)))
print("  RG2 - connection offset:", r(sim.getObjectPosition(rg2, conn)))
print("Next: start the Modbus/PLC side and run  python3 cella_esecutore.py")

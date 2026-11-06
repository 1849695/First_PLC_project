# Keyboard commands to the PLC:
#   python3 plc_cmd.py run | stop | reset | stato | avanti
#   python3 plc_cmd.py pezzi N | modo auto | modo man
from pymodbus.client import ModbusTcpClient
import sys, time

STATE, PEZZI_FATTI, RUN, RESET, PEZZI_TARGET, MODO, AVANTI = 1, 2, 17, 18, 19, 20, 21
NOMI_STATO = {0: "IDLE", 1: "RUNNING", 2: "STOPPED", 3: "ALARM"}
USO = "usage: python3 plc_cmd.py run|stop|reset|stato|avanti | pezzi N | modo auto|man"

args = sys.argv[1:]
if not args:
    raise SystemExit(USO)
comando = args[0]
valido = (
    (comando in ("run", "stop", "reset", "stato", "avanti") and len(args) == 1)
    or (comando == "pezzi" and len(args) == 2 and args[1].isdigit())
    or (comando == "modo" and len(args) == 2 and args[1] in ("auto", "man"))
)
if not valido:
    raise SystemExit(USO)

plc = ModbusTcpClient("localhost", port=502)
if not plc.connect():
    raise SystemExit("PLC not reachable on localhost:502")

def leggi(addr):
    return plc.read_holding_registers(addr, count=1).registers[0]

if comando == "run":
    plc.write_register(RUN, 1); print("run = 1")
elif comando == "stop":
    plc.write_register(RUN, 0); print("run = 0")
elif comando == "reset":
    plc.write_register(RESET, 1); time.sleep(0.3)
    plc.write_register(RESET, 0); print("reset sent")
elif comando == "pezzi":
    plc.write_register(PEZZI_TARGET, int(args[1])); print(f"target = {args[1]} pieces")
elif comando == "modo":
    plc.write_register(MODO, 0 if args[1] == "auto" else 1); print(f"mode = {args[1]}")
elif comando == "avanti":
    plc.write_register(AVANTI, 1); print("advance: one step (manual mode only)")

s = leggi(STATE)
modo = "AUTOMATIC" if leggi(MODO) == 0 else "MANUAL"
print(f"PLC state: {s} ({NOMI_STATO.get(s, '?')})  mode: {modo}  pieces: {leggi(PEZZI_FATTI)}/{leggi(PEZZI_TARGET)}")
plc.close()

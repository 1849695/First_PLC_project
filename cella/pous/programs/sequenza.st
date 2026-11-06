PROGRAM sequenza
  VAR
    cmd_step : INT AT %QW0;
    done_step : INT AT %MW0;
    run : INT AT %MW1;
    state : INT AT %QW1;
    pezzi_fatti : INT AT %QW2;
    slot : INT AT %QW3;
    ultimo : INT AT %QW4;
    reset : INT AT %MW2;
    pezzi_target : INT AT %MW3 := 1;
    modo : INT AT %MW4 := 0;
    avanti : INT AT %MW5 := 0;
    changed : BOOL;
    permesso : BOOL;
    last_done : INT;
    t : TON;
  END_VAR

changed := done_step <> last_done;
last_done := done_step;

IF avanti = 1 THEN
  IF state = 1 AND modo = 1 THEN permesso := TRUE; END_IF;
  avanti := 0;
END_IF;
IF changed THEN permesso := FALSE; END_IF;

t(IN := (state = 1) AND NOT changed AND (modo = 0 OR permesso = TRUE), PT := T#30s);

slot := pezzi_fatti MOD 10;

IF pezzi_fatti + 1 >= pezzi_target THEN ultimo := 1;
ELSE ultimo:=0;
END_IF;

CASE state OF
    0: // IDLE
    cmd_step := 0;
    IF run = 1 AND done_step < 10 THEN state := 1; pezzi_fatti := 0; END_IF;
    1: // RUNNING
    IF modo=0 OR permesso = TRUE THEN cmd_step := done_step + 1;
    ELSE cmd_step := 0;
    END_IF;
    IF run = 0 THEN state := 2; END_IF;
    IF t.Q THEN state := 3; END_IF;
    IF done_step >= 10 THEN pezzi_fatti := pezzi_fatti + 1;
    IF pezzi_fatti < pezzi_target THEN done_step := 0;
    ELSE state := 0; run := 0;
    END_IF;
    END_IF;
    2: // FERMO
    cmd_step := 0;
    IF run = 1 THEN state := 1; END_IF;
    3: // ALLARME
    cmd_step := 0;
    IF reset = 1 THEN state := 0; END_IF;
END_CASE;

END_PROGRAM
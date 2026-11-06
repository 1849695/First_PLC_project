PROGRAM main
  VAR
    enable : BOOL AT %IX0.3 := TRUE;
    dur : TIME;
    green : BOOL AT %QX0.0;
    yellow : BOOL AT %QX0.1;
    red : BOOL AT %QX0.2;
    state : INT;
    t : TON;
  END_VAR

t(IN := enable AND NOT t.Q, PT := dur);

CASE state OF
  0: // verde
     dur:= T#5s;
     green := TRUE;  yellow := FALSE;  red := FALSE;
     IF t.Q THEN state := 1; END_IF;
  1: // giallo
     dur:= T#3s;
     green := FALSE;  yellow := TRUE;  red := FALSE;
     IF t.Q THEN state := 2; END_IF;
  2: // rosso
     dur:= T#5s;
     green := FALSE;  yellow := FALSE;  red := TRUE;
     IF t.Q THEN state := 0; END_IF;
END_CASE;

END_PROGRAM
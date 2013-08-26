
# Possible errors of the MS5611

| Error | Short name | Diagnosis  | Mitigation |
| --- | --- | --- | --- |
| SCL or SDA stuck when initializing I2C bus|TWI-Init-Fehler|watch dog reset and setting the right flag|automatic (power cycle at setup) but avoid looping! |
| SCL or SDA stuck when measuring | TWI-Messfehler | as above  | as above | 
| PROM-Id is 0| PROM-Id-Fehler | =0 | power cycle |
| PROM CRC test fails | CRC-Fehler | CRC test fails | power cycle |
| Temp data is implausible <-20° or >100° | TEMP-Fehler | compare data | power cycle |
| Pressure data is implausible | PRESSURE-Fehler | compare data | power cycle |

Allow 5 such recovery events per run. Afterwards stop and report error.

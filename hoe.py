#!/usr/bin/env python3
import sys

def hoehenstufe(h, t):
    return (7.9+0.0008*h+0.03*t)

hs = hoehenstufe(float(sys.argv[1]), float(sys.argv[2]))
print("Höhenstufe: %.2f m/hPa" % hs);
print("Dezimeter:  %.4f Pa/dm" % (0.1/hs))

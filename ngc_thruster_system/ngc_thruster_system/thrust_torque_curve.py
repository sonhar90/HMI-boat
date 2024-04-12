#!/usr/bin/env python3

import numpy as np

"""
This class holds the methods to return the KT curves
"""
class ThrustTorqueCurve():
        def __init__(self) -> None:
                pass
        
        def my_KT_curve(self, J, pitch):
                """
                Basert på wagner B-series 3-blade AE/AO =0.5, P/D (pitch) 0.5-1.4. 
                The values in the diagram are linearzed.
                """
                Kt0 = 0.38
                dKt = 0.35
                return Kt0 * pitch - dKt *J

def main():
        pass

if __name__ == '__main__':
    main()
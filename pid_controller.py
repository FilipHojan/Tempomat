"""
Moduł definiuje klasycznego regulatora PID (Proportional–Integral–Derivative)
wykorzystywanego do sterowania prędkością pojazdu w tempomacie.

Regulator oblicza sygnał sterujący na podstawie uchybu prędkości, jego
całki oraz pochodnej, z uwzględnieniem ograniczeń sygnału sterującego
(0–1) oraz mechanizmu anti-windup.

Moduł współpracuje z:
- modelem obiektu (car_plant.py),
- funkcją symulacji (simulation.py),
- interfejsem graficznym (ui.py).
"""
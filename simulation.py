"""
simulation.py
--------------

Moduł odpowiedzialny za przeprowadzenie symulacji układu regulacji tempomatu.

Łączy:
- model pojazdu (car_plant.py),
- regulator PID (pid_controller.py) lub regulator rozmyty (fuzzy_controller.py),
- parametry użytkownika takie jak prędkość zadana, czas symulacji, krok czasowy.

Zwraca przebiegi czasowe:
- t_arr      – czas,
- v_arr      – prędkość pojazdu [m/s],
- u_arr      – sygnał sterujący [0–1],
- v_set_arr  – zadana prędkość [m/s].

Jest to „silnik” symulacji wykorzystywany przez interfejs graficzny (ui.py)
oraz opcjonalnie przez tryb konsolowy w main.py.
"""

from __future__ import annotations

import numpy as np
from typing import Tuple

from car_plant import CarPlant


def mps_to_kph(v: float | np.ndarray) -> float | np.ndarray:
    """
    Konwersja prędkości z [m/s] na [km/h].
    """
    return v * 3.6


def simulate(
    vehicle: CarPlant,
    controller,
    v_set: float,
    t_sim: float = 60.0,
    dt: float = 0.05
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Główna funkcja symulacji.

    :param vehicle: instancja klasy CarPlant lub pochodnej (Ferrari, Motocykl, Czołg)
    :param controller: regulator PID lub regulator rozmyty (musi mieć .step() i .reset())
    :param v_set: prędkość zadana [m/s]
    :param t_sim: czas symulacji [s]
    :param dt: krok czasowy [s]

    :return:
        t_arr     – tablica czasu
        v_arr     – prędkość pojazdu [m/s]
        u_arr     – sygnał sterujący [0–1]
        v_set_arr – zadana prędkość [m/s] (stała)
    """

    # Resetujemy stan pojazdu i regulatora
    vehicle.reset(0.0)
    controller.reset()

    steps = int(t_sim / dt)

    t_arr = np.linspace(0.0, t_sim, steps)
    v_arr = np.zeros(steps)
    u_arr = np.zeros(steps)
    v_set_arr = np.full(steps, v_set)

    for i in range(steps):
        v = vehicle.get_speed()
        e = v_set - v

        # wyznaczenie sterowania
        u = controller.step(e, dt)

        # przejście modelu
        v_next = vehicle.step(u, dt)

        # zapis wyników
        v_arr[i] = v_next
        u_arr[i] = u

    return t_arr, v_arr, u_arr, v_set_arr


# Tryb testowy dla developmentu (nieużywany przez GUI)
if __name__ == "__main__":
    from pid_controller import PIDController
    from car_plant import FerrariCar

    vehicle = FerrariCar()
    controller = PIDController()

    t, v, u, vsp = simulate(vehicle, controller, v_set=30/3.6)
    print("Symulacja testowa zakończona.")
    print("Prędkość końcowa:", mps_to_kph(v[-1]), "km/h")

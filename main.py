"""
main.py
--------

Główny plik uruchomieniowy projektu tempomatu.

Odpowiada za:
- uruchomienie interfejsu graficznego (ui.py),
- możliwość testowego uruchomienia symulacji w konsoli (bez GUI),
- spójne powiązanie struktury projektu.

Zgodnie z architekturą projektu:
    main.py -> ui.py -> (simulation, vehicle, controller)
"""

from __future__ import annotations

from ui import App
from simulation import simulate, mps_to_kph
from pid_controller import PIDController
from fuzzy_controller import FuzzyCruiseController
from car_plant import create_vehicle


def demo_console() -> None:
    """
    Tryb konsolowy – opcjonalny.
    Umożliwia przetestowanie symulacji bez używania GUI.
    """

    print("\n=== TRYB KONSOLI – DEMO SYMULACJI TEMPOMATU ===")

    # parametry
    v_set_kph = 80
    v_set = v_set_kph / 3.6
    t_sim = 20.0
    dt = 0.05

    vehicle = create_vehicle("Ferrari")

    print(f"\nWybrany model pojazdu: {vehicle.params.name}")
    print(f"Prędkość zadana: {v_set_kph} km/h")

    # PID
    pid = PIDController()
    t, v, u, vsp = simulate(vehicle, pid, v_set, t_sim, dt)
    print(f"Prędkość końcowa (PID): {mps_to_kph(v[-1]):.2f} km/h")

    # Regulator rozmyty
    fuzzy = FuzzyCruiseController()
    vehicle.reset()
    t, v, u, vsp = simulate(vehicle, fuzzy, v_set, t_sim, dt)
    print(f"Prędkość końcowa (Rozmyty): {mps_to_kph(v[-1]):.2f} km/h")

    print("\nAby zobaczyć wykresy i pełny tryb, uruchom GUI.")


def main() -> None:
    """
    Główna funkcja programu.
    Można tu przełączać typ uruchomienia.
    """

    USE_CONSOLE_MODE = False  # ustaw True jeśli chcesz tryb terminalowy

    if USE_CONSOLE_MODE:
        demo_console()
    else:
        app = App()
        app.mainloop()


if __name__ == "__main__":
    main()

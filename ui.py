"""
ui.py
------

Graficzny interfejs użytkownika (GUI) aplikacji tempomatu.

Wykorzystuje:
- Tkinter jako framework GUI,
- Matplotlib (FigureCanvasTkAgg) do rysowania wykresów.

UI odpowiada za:
- pobieranie parametrów użytkownika (prędkość zadana, czas symulacji, model pojazdu, regulator),
- uruchamianie symulacji (wywołanie simulation.py),
- wyświetlanie wyników w formie wykresów,
- przełączanie między regulatorami PID / Rozmyty / OBA,
- przełączanie między modelami: Ferrari / Motocykl / Czołg.

Dane wejściowe -> UI -> simulation.py -> wyniki -> wykresy.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from car_plant import create_vehicle
from pid_controller import PIDController
from fuzzy_controller import FuzzyCruiseController
from simulation import simulate, mps_to_kph


class App(tk.Tk):
    """
    Główna klasa aplikacji GUI.

    Tworzy okno, pola wejściowe, kontrolki
    oraz obszar wykresów. Obsługuje zdarzenia
    związane z kliknięciem przycisku "Uruchom symulację".
    """

    def __init__(self) -> None:
        super().__init__()

        self.title("Tempomat – PID & Regulator Rozmyty")
        self.geometry("1000x700")
        self.configure(background="#f1f1f1")

        # domyślne parametry
        self.default_speed_kph = 100.0
        self.default_t_sim = 60.0
        self.default_dt = 0.05

        # budowanie GUI
        self._build_controls()
        self._build_figure()

    # ---------------------------------------------------------------------

    def _build_controls(self):
        """
        Tworzy górny panel sterowania (pola wejściowe, listy, przyciski).
        """
        frame = ttk.Frame(self)
        frame.pack(side=tk.TOP, fill=tk.X, pady=10)

        # Prędkość zadana
        ttk.Label(frame, text="Prędkość zadana [km/h]:").grid(row=0, column=0, padx=5)
        self.entry_speed = ttk.Entry(frame, width=10)
        self.entry_speed.insert(0, str(self.default_speed_kph))
        self.entry_speed.grid(row=0, column=1, padx=5)

        # Czas symulacji
        ttk.Label(frame, text="Czas symulacji [s]:").grid(row=0, column=2, padx=5)
        self.entry_time = ttk.Entry(frame, width=10)
        self.entry_time.insert(0, str(self.default_t_sim))
        self.entry_time.grid(row=0, column=3, padx=5)

        # Wybór regulatora
        ttk.Label(frame, text="Regulator:").grid(row=1, column=0, padx=5)
        self.reg_mode = tk.StringVar(value="PID")
        self.combo_reg = ttk.Combobox(
            frame, textvariable=self.reg_mode,
            values=["PID", "Rozmyty", "Oba"],
            width=10, state="readonly"
        )
        self.combo_reg.grid(row=1, column=1, padx=5)

        # Wybór pojazdu
        ttk.Label(frame, text="Model pojazdu:").grid(row=1, column=2, padx=5)
        self.vehicle_mode = tk.StringVar(value="Ferrari")
        self.combo_vehicle = ttk.Combobox(
            frame, textvariable=self.vehicle_mode,
            values=["Ferrari", "Motocykl", "Czołg"],
            width=10, state="readonly"
        )
        self.combo_vehicle.grid(row=1, column=3, padx=5)

        # Przycisk
        btn = ttk.Button(frame, text="Uruchom symulację", command=self.run_simulation)
        btn.grid(row=0, column=4, rowspan=2, padx=20)

    # ---------------------------------------------------------------------

    def _build_figure(self):
        """
        Tworzy obszar wykresów (Matplotlib w Tkinter).
        """
        frame = ttk.Frame(self)
        frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=10)

        self.figure = Figure(figsize=(8, 6), dpi=100)

        self.ax_speed = self.figure.add_subplot(211)
        self.ax_speed.set_ylabel("Prędkość [km/h]")

        self.ax_control = self.figure.add_subplot(212)
        self.ax_control.set_ylabel("Sterowanie u(t)")
        self.ax_control.set_xlabel("Czas [s]")

        self.canvas = FigureCanvasTkAgg(self.figure, master=frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ---------------------------------------------------------------------

    def run_simulation(self):
        """
        Odczytuje dane użytkownika, tworzy obiekt pojazdu i regulatora,
        wykonuje symulację oraz rysuje wyniki.
        """
        try:
            v_set_kph = float(self.entry_speed.get())
            t_sim = float(self.entry_time.get())
        except ValueError:
            messagebox.showerror("Błąd", "Wprowadź poprawne wartości liczbowe.")
            return

        reg_mode = self.reg_mode.get()
        vehicle_name = self.vehicle_mode.get()

        # Tworzenie modelu pojazdu
        vehicle = create_vehicle(vehicle_name)

        # Konwersja prędkości zadanej
        v_set = v_set_kph / 3.6  # km/h → m/s

        # Czyścimy wykresy
        self.ax_speed.clear()
        self.ax_control.clear()

        self.ax_speed.set_ylabel("Prędkość [km/h]")
        self.ax_control.set_ylabel("Sterowanie u(t)")
        self.ax_control.set_xlabel("Czas [s]")

        # ========== TRYB: PID ==========
        if reg_mode in ("PID", "Oba"):
            pid = PIDController()
            t, v, u, vsp = simulate(vehicle, pid, v_set, t_sim, self.default_dt)

            self.ax_speed.plot(t, mps_to_kph(v), label="PID")
            self.ax_control.plot(t, u, label="PID")

        # ========== TRYB: ROZMYTY ==========
        if reg_mode in ("Rozmyty", "Oba"):
            fuzzy = FuzzyCruiseController()
            t, v, u, vsp = simulate(vehicle, fuzzy, v_set, t_sim, self.default_dt)

            self.ax_speed.plot(t, mps_to_kph(v), linestyle="--", label="Rozmyty")
            self.ax_control.plot(t, u, linestyle="--", label="Rozmyty")

        # Prędkość zadana
        self.ax_speed.plot(t, mps_to_kph(vsp), linestyle=":", label="Zadana")

        # Wykończenie wykresów
        self.ax_speed.grid(True)
        self.ax_control.grid(True)

        self.ax_speed.legend()
        self.ax_control.legend()

        self.figure.suptitle(f"Tempomat – Model: {vehicle_name}", fontsize=12)

        self.canvas.draw()


# ---------------------------------------------------------------------
# Tryb autonomiczny – uruchomienie GUI po odpaleniu pliku
# ---------------------------------------------------------------------
if __name__ == "__main__":
    app = App()
    app.mainloop()

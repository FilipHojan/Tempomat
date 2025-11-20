"""
pid_controller.py
------------------

Moduł definiuje klasycznego regulatora PID (Proportional–Integral–Derivative)
wykorzystywanego do sterowania prędkością pojazdu w tempomacie.

Regulator oblicza sygnał sterujący na podstawie:
- uchybu prędkości e,
- całki uchybu,
- pochodnej uchybu,

z uwzględnieniem:
- ograniczeń sygnału sterującego (0–1),
- prostego mechanizmu anti-windup.

Dane przechowywane w obiekcie:
- wzmocnienia: Kp, Ki, Kd,
- zakres sterowania: u_min, u_max,
- stan wewnętrzny: integral (∫e dt), e_prev (uchyb z poprzedniego kroku).
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class PIDParams:
    """
    Parametry regulatora PID.

    Kp    - wzmocnienie proporcjonalne
    Ki    - wzmocnienie całkujące
    Kd    - wzmocnienie różniczkujące
    u_min - dolne ograniczenie sterowania
    u_max - górne ograniczenie sterowania
    """
    Kp: float = 0.8
    Ki: float = 0.4
    Kd: float = 0.05
    u_min: float = 0.0
    u_max: float = 1.0


class PIDController:
    """
    Implementacja regulatora PID.

    Sygnał sterujący:
        u(t) = Kp * e(t) + Ki * ∫ e(t) dt + Kd * de(t)/dt

    Główne metody:
    - reset() – kasuje stan wewnętrzny regulatora,
    - step(e, dt) – wyznacza nową wartość sterowania u na podstawie uchybu e.

    Dane wewnętrzne:
    - params    – struktura z parametrami PID,
    - integral  – całka uchybu,
    - e_prev    – uchyb z poprzedniego kroku (do obliczenia pochodnej).
    """

    def __init__(self, params: PIDParams | None = None) -> None:
        self.params: PIDParams = params if params is not None else PIDParams()
        self.integral: float = 0.0
        self.e_prev: float = 0.0

    def reset(self) -> None:
        """
        Resetuje stan wewnętrzny regulatora PID (integral, e_prev).
        """
        self.integral = 0.0
        self.e_prev = 0.0

    def step(self, e: float, dt: float) -> float:
        """
        Oblicza nową wartość sterowania u na podstawie aktualnego uchybu e
        oraz kroku czasowego dt.

        :param e: uchyb regulacji (np. e = v_zad - v_rzecz)
        :param dt: krok czasowy [s]
        :return: sygnał sterujący u w zakresie [u_min, u_max]
        """
        Kp = self.params.Kp
        Ki = self.params.Ki
        Kd = self.params.Kd
        u_min = self.params.u_min
        u_max = self.params.u_max

        # człon całkujący
        self.integral += e * dt
        # prosty anti-windup (ograniczenie całki)
        self.integral = float(np.clip(self.integral, -100.0, 100.0))

        # człon różniczkujący
        if dt > 0.0:
            de = (e - self.e_prev) / dt
        else:
            de = 0.0
        self.e_prev = e

        # prawo regulacji PID
        u = Kp * e + Ki * self.integral + Kd * de

        # saturacja sygnału sterującego
        u = float(np.clip(u, u_min, u_max))
        return u

    def set_gains(self, Kp: float | None = None,
                  Ki: float | None = None,
                  Kd: float | None = None) -> None:
        """
        Ustawia (opcjonalnie) nowe wartości wzmocnień regulatora.

        :param Kp: nowe Kp (lub None, aby nie zmieniać)
        :param Ki: nowe Ki (lub None, aby nie zmieniać)
        :param Kd: nowe Kd (lub None, aby nie zmieniać)
        """
        if Kp is not None:
            self.params.Kp = float(Kp)
        if Ki is not None:
            self.params.Ki = float(Ki)
        if Kd is not None:
            self.params.Kd = float(Kd)

    def set_limits(self, u_min: float | None = None,
                   u_max: float | None = None) -> None:
        """
        Ustawia ograniczenia sygnału sterującego.

        :param u_min: dolne ograniczenie (lub None)
        :param u_max: górne ograniczenie (lub None)
        """
        if u_min is not None:
            self.params.u_min = float(u_min)
        if u_max is not None:
            self.params.u_max = float(u_max)

    def __repr__(self) -> str:
        p = self.params
        return (f"<PIDController Kp={p.Kp}, Ki={p.Ki}, Kd={p.Kd}, "
                f"u_min={p.u_min}, u_max={p.u_max}>")

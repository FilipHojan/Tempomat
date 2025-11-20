"""
fuzzy_controller.py
--------------------

Moduł implementuje regulator rozmyty typu Mamdaniego przeznaczony do
sterowania prędkością pojazdu w układzie tempomatu.

Regulator wykorzystuje dwie zmienne wejściowe:
- uchyb prędkości (e),
- pochodna uchybu (de),

oraz zestaw lingwistycznych reguł sterowania o postaci:
    JEŚLI e = ... ORAZ de = ... TO u = ...

Zbiory rozmyte wejść:
    NB – Very Negative (bardzo ujemny)
    NS – Negative Small (mały ujemny)
    Z  – Zero (zerowy)
    PS – Positive Small (mały dodatni)
    PB – Positive Big (duży dodatni)

Zbiory wyjścia:
    Z – Zero
    S – Small
    M – Medium
    B – Big

Defuzyfikacja: metoda środka ciężkości (COG – Center of Gravity).

Moduł definiuje:
- trzy funkcje przynależności (MF),
- regulator FuzzyCruiseController,
- bazę reguł,
- logikę agregacji i defuzyfikacji.
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass


# -------------------------------------------------------------------------
# Funkcje przynależności (Membership Functions – MF)
# -------------------------------------------------------------------------

def triangular_mf(x: float, a: float, b: float, c: float) -> float:
    """
    Trójkątna funkcja przynależności.

    Parametry:
        a - początek
        b - środek (maksimum)
        c - koniec

    Zwraca wartość przynależności x do zbioru w [0, 1].
    """
    if a == b == c:
        return 1.0 if x == a else 0.0
    if x <= a or x >= c:
        return 0.0
    if x == b:
        return 1.0
    if x < b:
        return (x - a) / (b - a)
    return (c - x) / (c - b)


# -------------------------------------------------------------------------
# Parametry regulatora rozmytego
# -------------------------------------------------------------------------

@dataclass
class FuzzyParams:
    """
    Parametry regulatora rozmytego:

    e_range  - zakres uchybu prędkości [m/s]
    de_range - zakres pochodnej uchybu [m/s^2]
    u_range  - zakres wyjścia sterującego (0–1)
    resolution - liczba punktów do defuzyfikacji
    """
    e_range: tuple[float, float] = (-10.0, 10.0)
    de_range: tuple[float, float] = (-3.0, 3.0)
    u_range: tuple[float, float] = (0.0, 1.0)
    resolution: int = 101


# -------------------------------------------------------------------------
# Regulator rozmyty Mamdaniego
# -------------------------------------------------------------------------

class FuzzyCruiseController:
    """
    Regulator rozmyty typu Mamdani stosowany do tempomatu.

    Wejścia:
        e  – uchyb prędkości (m/s)
        de – pochodna uchybu (m/s^2)

    Wyjście:
        u  – sterowanie [0, 1]

    Logika:
        - fuzzification,
        - rule evaluation,
        - aggregation,
        - defuzzification (COG).

    Stan wewnętrzny regulatora:
        e_prev – uchyb z poprzedniego kroku (do obliczenia de)
    """

    def __init__(self, params: FuzzyParams | None = None) -> None:
        self.params = params if params is not None else FuzzyParams()
        self.e_prev: float = 0.0

        # uniwersum wyjścia do defuzyfikacji
        u_min, u_max = self.params.u_range
        self.u_universe = np.linspace(u_min, u_max, self.params.resolution)

    # ---------------------------------------------------------------------
    # Rozmycie wejść – zbiory lingwistyczne e i de
    # ---------------------------------------------------------------------

    def _e_terms(self, e: float) -> dict[str, float]:
        """
        Zbiory rozmyte uchybu (e):
            NB, NS, Z, PS, PB
        """
        e_min, e_max = self.params.e_range
        width = (e_max - e_min) / 4.0

        centers = {
            "NB": e_min,
            "NS": e_min / 2.0,
            "Z":  0.0,
            "PS": e_max / 2.0,
            "PB": e_max,
        }

        return {
            term: triangular_mf(e, c - width, c, c + width)
            for term, c in centers.items()
        }

    def _de_terms(self, de: float) -> dict[str, float]:
        """
        Zbiory rozmyte pochodnej uchybu (de):
            NB, NS, Z, PS, PB
        """
        de_min, de_max = self.params.de_range
        width = (de_max - de_min) / 4.0

        centers = {
            "NB": de_min,
            "NS": de_min / 2.0,
            "Z":  0.0,
            "PS": de_max / 2.0,
            "PB": de_max,
        }

        return {
            term: triangular_mf(de, c - width, c, c + width)
            for term, c in centers.items()
        }

    def _u_mf(self, label: str, u: float) -> float:
        """
        Funkcja przynależności wyjścia u dla zbiorów:
            Z, S, M, B
        """
        u_min, u_max = self.params.u_range
        width = (u_max - u_min) / 4.0

        centers = {
            "Z": u_min,
            "S": u_min + width,
            "M": u_min + 2 * width,
            "B": u_max,
        }

        c = centers[label]
        return triangular_mf(u, c - width, c, c + width)

    # ---------------------------------------------------------------------
    # Rdzeń regulatora – obliczanie sterowania
    # ---------------------------------------------------------------------

    def reset(self) -> None:
        """Resetuje pamięć regulatora (e_prev)."""
        self.e_prev = 0.0

    def step(self, e: float, dt: float) -> float:
        """
        Oblicza sterowanie u na podstawie uchybu e oraz kroku dt.

        :param e: uchyb prędkości
        :param dt: krok czasowy
        :return: sterowanie u w zakresie [0, 1]
        """
        if dt > 0.0:
            de = (e - self.e_prev) / dt
        else:
            de = 0.0
        self.e_prev = e

        # fuzzification
        e_terms = self._e_terms(e)
        de_terms = self._de_terms(de)

        # rule-base (zgodna z logiką tempomatu):
        rules = {
            ("PB", "NB"): "M",
            ("PB", "NS"): "B",
            ("PB", "Z"):  "B",
            ("PB", "PS"): "B",
            ("PB", "PB"): "M",

            ("PS", "NB"): "S",
            ("PS", "NS"): "M",
            ("PS", "Z"):  "M",
            ("PS", "PS"): "B",
            ("PS", "PB"): "M",

            ("Z",  "NB"): "Z",
            ("Z",  "NS"): "S",
            ("Z",  "Z"):  "M",
            ("Z",  "PS"): "S",
            ("Z",  "PB"): "Z",

            ("NS", "NB"): "Z",
            ("NS", "NS"): "Z",
            ("NS", "Z"):  "S",
            ("NS", "PS"): "S",
            ("NS", "PB"): "Z",

            ("NB", "NB"): "Z",
            ("NB", "NS"): "Z",
            ("NB", "Z"):  "Z",
            ("NB", "PS"): "Z",
            ("NB", "PB"): "Z",
        }

        # agregacja wyników reguł
        mu_out = {label: 0.0 for label in ["Z", "S", "M", "B"]}

        for (e_label, de_label), u_label in rules.items():
            activation = min(e_terms[e_label], de_terms[de_label])
            mu_out[u_label] = max(mu_out[u_label], activation)

        # defuzyfikacja (środek ciężkości)
        num = 0.0
        den = 0.0

        for u in self.u_universe:
            mu = 0.0
            for label, strength in mu_out.items():
                if strength > 0.0:
                    mu = max(mu, min(strength, self._u_mf(label, u)))
            num += u * mu
            den += mu

        if den == 0.0:
            return 0.0  # brak aktywacji – brak sterowania

        return float(num / den)

    # ---------------------------------------------------------------------

    def __repr__(self) -> str:
        return f"<FuzzyCruiseController e_range={self.params.e_range}, de_range={self.params.de_range}>"

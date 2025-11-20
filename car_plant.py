"""
car_plant.py
------------

Moduł zawiera implementację uproszczonego modelu dynamiki pojazdu
wykorzystywanego w symulacji tempomatu.

Model opisuje ruch pojazdu wzdłuż kierunku jazdy przy założeniu działania:
- siły trakcyjnej (zależnej od sterowania u),
- sił oporów ruchu (proporcjonalnych do prędkości).

Równanie ruchu (model ciągły):
    m * dv/dt = F_trakcji - F_oporów

gdzie:
    m           - masa pojazdu [kg]
    v           - prędkość [m/s]
    F_trakcji   - siła napędowa [N]
    F_oporów    - siła oporów ruchu [N]

W implementacji przyjmujemy:
    F_trakcji = u * F_max
    F_oporów  = c_r * v

gdzie:
    u    - sygnał sterujący w zakresie [0, 1] (pozycja "gazu")
    F_max - maksymalna siła napędowa [N]
    c_r   - współczynnik oporów ruchu

Moduł definiuje:
- klasę bazową CarPlant,
- trzy konkretne modele:
    * FerrariCar
    * Motorcycle
    * TankCar

oraz funkcję create_vehicle(model_name), która zwraca wybrany model.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np


@dataclass
class VehicleParams:
    """
    Struktura parametrów pojazdu.

    mass   - masa [kg]
    F_max  - maksymalna siła napędowa [N]
    c_r    - współczynnik oporu (N / (m/s))
    name   - nazwa pojazdu (opisowa)
    """
    mass: float
    F_max: float
    c_r: float
    name: str


class CarPlant:
    """
    Ogólny model pojazdu wykorzystywany jako obiekt regulacji.

    Wewnętrznie przechowuje prędkość w [m/s].
    """

    def __init__(self, params: VehicleParams) -> None:
        self.params = params
        self.v: float = 0.0  # aktualna prędkość [m/s]

    def reset(self, v0: float = 0.0) -> None:
        """
        Resetuje stan pojazdu (prędkość początkowa).

        :param v0: prędkość początkowa [m/s]
        """
        self.v = float(v0)

    def step(self, u: float, dt: float) -> float:
        """
        Jedno przejście modelu w czasie dt.

        :param u: sygnał sterujący w zakresie [0, 1]
        :param dt: krok czasowy [s]
        :return: nowa prędkość [m/s]
        """
        # saturacja sterowania
        u = float(np.clip(u, 0.0, 1.0))

        m = self.params.mass
        F_max = self.params.F_max
        c_r = self.params.c_r

        # siła trakcyjna i opory
        F_tr = u * F_max
        F_res = c_r * self.v

        # przyspieszenie
        a = (F_tr - F_res) / m

        # całkowanie (Euler prosty)
        self.v += a * dt
        if self.v < 0.0:
            self.v = 0.0

        return self.v

    def get_speed(self) -> float:
        """
        Zwraca aktualną prędkość pojazdu [m/s].
        """
        return self.v

    def get_speed_kph(self) -> float:
        """
        Zwraca aktualną prędkość pojazdu [km/h].
        """
        return self.v * 3.6

    def __repr__(self) -> str:
        return f"<CarPlant name={self.params.name!r}, v={self.v:.2f} m/s>"


class FerrariCar(CarPlant):
    """
    Model sportowego auta (Ferrari) – duża moc, relatywnie niska masa, niskie opory.
    """

    def __init__(self) -> None:
        params = VehicleParams(
            mass=1400.0,       # [kg]
            F_max=7000.0,      # [N] duża siła napędowa
            c_r=40.0,          # mniejsze opory niż domyślnie
            name="Ferrari"
        )
        super().__init__(params)


class Motorcycle(CarPlant):
    """
    Model motocykla – bardzo niska masa, umiarkowana siła napędowa, małe opory.
    """

    def __init__(self) -> None:
        params = VehicleParams(
            mass=250.0,        # [kg]
            F_max=3000.0,      # [N]
            c_r=30.0,          # małe opory
            name="Motocykl"
        )
        super().__init__(params)


class TankCar(CarPlant):
    """
    Model czołgu – bardzo duża masa, stosunkowo niewielka siła napędowa,
    duże opory ruchu.

    Układ o dużej bezwładności, trudny do szybkiej regulacji.
    """

    def __init__(self) -> None:
        params = VehicleParams(
            mass=30000.0,      # [kg] duża masa
            F_max=15000.0,     # [N]
            c_r=200.0,         # duże opory
            name="Czołg"
        )
        super().__init__(params)


VehicleName = Literal["ferrari", "motorcycle", "motocykl", "tank", "czolg", "czołg"]


def create_vehicle(model_name: VehicleName) -> CarPlant:
    """
    Fabryka obiektów pojazdów na podstawie nazwy modelu.

    Do wykorzystania np. w GUI lub w kodzie main.

    :param model_name: nazwa modelu:
        - "ferrari"
        - "motorcycle" / "motocykl"
        - "tank" / "czolg" / "czołg"
    :return: instancja CarPlant (odpowiedniej klasy)
    """
    name = model_name.lower()

    if name == "ferrari":
        return FerrariCar()
    if name in ("motorcycle", "motocykl"):
        return Motorcycle()
    if name in ("tank", "czolg", "czołg"):
        return TankCar()

    # domyślnie – Ferrari, żeby nie wywalać programu
    return FerrariCar()

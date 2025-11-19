"""
Moduł implementuje regulator rozmyty typu Mamdaniego przeznaczony do
sterowania prędkością pojazdu w układzie tempomatu.

Regulator wykorzystuje dwie zmienne wejściowe:
- uchyb prędkości (e),
- pochodna uchybu (de),

oraz zestaw lingwistycznych reguł sterowania o postaci:
    JEŚLI e = ... ORAZ de = ... TO u = ...

Defuzyfikacja realizowana jest metodą środka ciężkości (COG).
Moduł zapewnia funkcję step(), która zwraca sygnał sterujący w zakresie 0–1.

Współpracuje z:
- modelem pojazdu (car_plant.py),
- symulatorem (simulation.py),
- interfejsem użytkownika (ui.py).
"""
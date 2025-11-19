"""
Moduł implementuje graficzny interfejs użytkownika (GUI) dla aplikacji
tempomatu, wykorzystując bibliotekę Tkinter oraz matplotlib.

Interfejs umożliwia:
- wybór typu regulatora (PID / rozmyty / oba),
- ustawienie prędkości zadanej,
- uruchomienie symulacji,
- wizualizację przebiegów prędkości oraz sygnałów sterowania.

Moduł integruje wszystkie pozostałe komponenty:
- regulator PID (pid_controller.py),
- regulator rozmyty (fuzzy_controller.py),
- model pojazdu (car_plant.py),
- symulację (simulation.py).
"""
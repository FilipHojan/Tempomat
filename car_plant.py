"""
Moduł zawiera implementację uproszczonego modelu dynamiki pojazdu
wykorzystywanego w symulacji tempomatu. Model opisuje ruch pojazdu
wzdłuż kierunku jazdy przy założeniu działania siły trakcyjnej oraz
sił oporów ruchu.

Model wykorzystywany jest jako obiekt regulacji w układach:
- regulatora PID,
- regulatora rozmytego (Mamdani).

Funkcja step() realizuje dyskretną symulację zmian prędkości pojazdu.
"""

"""
Moduł zawiera funkcje pomocnicze do realizacji symulacji układu regulacji
tempomatu. Odpowiada za połączenie wybranego regulatora (PID lub rozmytego)
z modelem pojazdu oraz za generowanie przebiegów czasowych:

- prędkości pojazdu,
- sygnału sterującego,
- prędkości zadanej.

Moduł jest wykorzystywany przez interfejs graficzny (ui.py) oraz przez
główne wejście programu (main.py).
"""

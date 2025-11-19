# Tempomat – PID & Regulator Rozmyty  

**Symulacja sterowania prędkością dla różnych modeli pojazdów (Ferrari, Motocykl, Czołg)**  

Projekt w Pythonie z GUI (Tkinter + Matplotlib)

## Jak uruchomić projekt

### 1. Zainstaluj wymagane biblioteki:

pip install matplotlib

---

### 2. Uruchom aplikację:

python main.py


Po uruchomieniu pojawi się graficzne okno aplikacji.

---

## Obsługa interfejsu (GUI)

Po uruchomieniu programu możesz:

- ustawić **prędkość zadaną [km/h]**
- ustawić **czas symulacji [s]**
- wybrać **rodzaj regulatora**:
  - PID
  - Rozmyty
  - Oba (porównanie)
- wybrać **model pojazdu**:
  - Ferrari  
  - Motocykl  
  - Czołg  

Aby wykonać symulację:  
uzupełnij parametry → kliknij **„Uruchom symulację”**

Program wyświetli dwa wykresy:

1. **Prędkość pojazdu vs prędkość zadana**  
2. **Sygnał sterujący u(t)**  

---

## Struktura projektu

```
Tempomat/
├── main.py                 
├── ui.py                   
├── simulation.py           
├── car_plant.py            
├── pid_controller.py       
├── fuzzy_controller.py     
└── README.md               
```

---

# Opis plików

### main.py
Główny plik startowy. Uruchamia interfejs graficzny aplikacji.

### ui.py
Graficzny interfejs użytkownika:
- wybór regulatora,
- wybór modelu pojazdu,
- ustawienia symulacji,
- generowanie wykresów,
- integracja symulacji z GUI.

### car_plant.py
Modele dynamiki pojazdów:

#### 1. Ferrari
- duża moc,
- szybka reakcja,
- niskie opory.

#### 2. Motocykl
- niska masa,
- małe opory,
- wysoka dynamika.

#### 3. Czołg
- ogromna masa,
- wolna reakcja,
- duże opory.

Wszystkie modele bazują na równaniu:
```
m * dv/dt = F_trakcji - F_oporów
```

### pid_controller.py
Regulator PID:
- sygnał: u = Kp*e + Ki*∫e + Kd*de/dt
- saturacja 0–1
- anti-windup
- klasyczne sterowanie tempomatem

### fuzzy_controller.py
Regulator rozmyty (Mamdani):
- wejścia: uchyb e i pochodna uchybu de
- zbiory rozmyte: NB, NS, Z, PS, PB
- reguły typu:  
  "JEŚLI e = PB ORAZ de = NS TO u = B"
- defuzyfikacja środka ciężkości

### simulation.py
Łączy regulator i model pojazdu:
- generuje przebiegi prędkości
- przebieg sterowania
- przebieg wartości zadanej
- stosowany przez UI

---

## Wyniki symulacji

Dwa wykresy:

### 1. Prędkość pojazdu:
- PID  
- Rozmyta  
- Zadana  

Pozwala ocenić jakość regulacji.

### 2. Sterowanie u(t):
Pokazuje jak intensywnie działa regulator.

---

# Opis teoretyczny

Projekt analizuje układ regulacji prędkości (tempomat) w dwóch wariantach:

---

## 1. Regulator PID
Składniki:
- **P** – natychmiastowa reakcja,
- **I** – redukcja uchybu ustalonego,
- **D** – tłumienie zmian.

Zalety:
- prosty
- skuteczny dla większości obiektów  

Wady:
- podatny na przeregulowanie  
- wymaga strojenia  

---

## 2. Regulator Rozmyty (Mamdani)

Działa jak człowiek opisujący zachowanie:

- „Jedziesz za wolno → dodaj gazu.”
- „Jedziesz za szybko → odejmij gazu.”

Zbiory lingwistyczne:
- NB, NS, Z, PS, PB

Sterowanie jest:
- gładkie,  
- odporne na zakłócenia,  
- mniej agresywne niż PID.  

---

# Dlaczego 3 modele pojazdów?

Każdy pojazd ma inne wymagania sterowania:

| Model     | Cechy                         | Trudność regulacji |
|-----------|-------------------------------|---------------------|
| Ferrari   | bardzo szybki, dynamiczny     | łatwy               |
| Motocykl  | lekki, szybka reakcja         | średnia             |
| Czołg     | ciężki, powolny               | trudny              |

Pozwala to ocenić:
- stabilność regulatorów,
- dynamikę układu,
- wpływ parametrów obiektu,
- zachowanie regulatora przy różnych warunkach.

---

# Podsumowanie

Projekt zawiera:

✔ regulator PID  
✔ regulator rozmyty  
✔ trzy modele pojazdów  
✔ graficzny interfejs użytkownika  
✔ wykresy porównawcze  
✔ modularną architekturę  
---

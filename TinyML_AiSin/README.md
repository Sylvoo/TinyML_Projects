# TinyML_STM_SIN – generator sinusoidy z inferencją NN na STM32

**Autor:** Sylwester Ślusarczyk  
**Płytka/MCU:** STM32F401CEU6 (rodzina F4)  
**Model:** sieć neuronowa przewidująca wartość sin(x) dla `x ∈ [-2π, 2π]`  
**Wyświetlacz:** OLED SSD1306 (I²C, 128×64) – przebieg + czas inferencji
**Lokalizacja** Core -> Src

> Projekt prezentuje prosty „tiny” model ML działający w MCU: dla kolejnych wartości `x` sieć oblicza `sin(x)` i rysuje wynik na OLED-zie wraz z linią odniesienia i czasem inferencji w mikrosekundach.

---

## ✨ Funkcjonalność

- Generacja próbek wejścia `x` w zakresie `0 … 2π` **(krok ≈ `2π/64`)**.
- **Inferencja** sieci neuronowej (wygenerowanej X-CUBE-AI) dla każdej próbki:
  - wejście: skalar `x` (`ai_float`)
  - wyjście: skalar `sin(x)` (przybliżony przez NN)
- Pomiar czasu wykonania inferencji przy pomocy **TIM11** (1 µs tick).
- **Wizualizacja** na OLED SSD1306:
  - przewijany wykres wartości wyjścia,
  - pozioma oś „0”,
  - etykieta „AI_SIN”,
  - czas pojedynczej inferencji `t=… us`.


---

## 🧱 Architektura & ważniejsze pliki

- `main.c` – inicjalizacja HAL, timerów, I²C, OLED; pętla główna z inferencją i rysowaniem.
- `ssd1306.h/.c`, `fonts.h` – sterownik i czcionki dla OLED.
- `ai_platform.h`, `ai_datatypes_defines.h` – nagłówki biblioteki inferencji (X-CUBE-AI).
- `sine_model_own.h`, `sine_model_own_data.h` – interfejs i dane wygenerowanego modelu.
- Funkcje pomocnicze do rysowania:
  - `Graph_Init()` – czyszczenie bufora i ekranu,
  - `Graph_PushAndDraw(y, y_min, y_max, label, dur)` – dodanie punktu, przewinięcie i odrys.

---

## 🖥️ Wyświetlacz OLED (SSD1306, I²C 128×64)

- Interfejs: **I²C** (z kodu: `I2C1` z szybkością **400 kHz**).
- Biblioteka: `ssd1306` używana przez `SSD1306_Init/UpdateScreen/Puts/DrawLine/DrawPixel`.
- **Zasilanie:** `VCC 3.3 V`, `GND` do GND mikrokontrolera.
- **Linie I²C:** `SCL`/`SDA` → piny I²C1 skonfigurowane w projekcie (`.ioc`).
  - Typowo na F401CE: **PB8=SCL, PB9=SDA** *lub* **PB6=SCL, PB7=SDA** (zależnie od konfiguracji w `.ioc`).  
  - Sprawdź w `.ioc` i schemacie płytki; adres SSD1306 zwykle `0x3C`.

> Na starcie wyświetla się ekran „AI_SINUS WELCOME!”, po ~2 s uruchamia się wykres z etykietą „AI_SIN” i **czasem inferencji** w µs w dolnej części ekranu.




https://github.com/user-attachments/assets/c17c8d68-77ed-4d0f-9301-eb7e68390b83



---

## 🔢 Model i pętla inferencji

- Wejście: `x` – rośnie o `step = 2π/64` (~0.098 rad) i zawija się do zakresu `[0, 2π]`.
- Przed inferencją zapamiętywana jest wartość licznika `TIM11`.
- Wywołanie:
  ```c
  nbatch = ai_sine_model_own_run(sine_model, &ai_input[0], &ai_output[0]);

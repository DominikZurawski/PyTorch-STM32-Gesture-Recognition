import serial
import csv
import time
import os
from datetime import datetime

def collect_data(port='COM3', baud_rate=115200, duration=10, gesture_name='unknown'):
    """
    Zbiera dane z akcelerometru przez określony czas i zapisuje je do pliku CSV.

    Args:
        port: Port szeregowy (domyślnie COM3)
        baud_rate: Prędkość transmisji (domyślnie 115200)
        duration: Czas zbierania danych w sekundach
        gesture_name: Nazwa wykonywanego gestu
    """
    os.makedirs('gesture_data', exist_ok=True)

    # Nazwa pliku z datą i nazwą gestu
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"gesture_data/{gesture_name}_{timestamp}.csv"

    ser = serial.Serial(port, baud_rate, timeout=1)
    time.sleep(1)

    # Przygotowanie pliku CSV
    with open(filename, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['timestamp', 'acc_x', 'acc_y', 'acc_z'])

        print(f"Zbieranie danych dla gestu '{gesture_name}'. Wykonaj gest...")
        time.sleep(1)  # Daj czas użytkownikowi na przygotowanie

        start_time = time.time()
        while time.time() - start_time < duration:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    try:
                        x, y, z = map(int, line.split(','))
                        timestamp_ms = int((time.time() - start_time) * 1000)
                        csv_writer.writerow([timestamp_ms, x, y, z])
                        print(f"\rOdczyt: x={x}, y={y}, z={z}", end='')
                    except ValueError:
                        pass  # Ignoruj nieprawidłowe dane

    ser.close()
    print(f"\nDane zapisane do {filename}")
    return filename

def main():
    """
    Główna funkcja do zbierania danych dla różnych gestów.
    """
    gestures = [
        "przesuniecie_w_prawo",
        "przesuniecie_w_lewo",
        "przesuniecie_w_gore",
        "przesuniecie_w_dol",
        "okrag_zgodnie_z_ruchem_wskazowek",
        "okrag_przeciwnie_do_ruchu_wskazowek",
        "litera_Z",
        "litera_V"
    ]

    port = input("Podaj port szeregowy (domyślnie COM3): ") or "COM3"
    baud_rate = int(input("Podaj prędkość transmisji (domyślnie 115200): ") or "115200")
    duration = int(input("Podaj czas trwania nagrania w sekundach (domyślnie 2): ") or "2")
    repetitions = int(input("Podaj liczbę powtórzeń każdego gestu (domyślnie 5): ") or "5")

    for gesture in gestures:
        for i in range(repetitions):
            input(f"Naciśnij Enter, aby rozpocząć nagrywanie gestu '{gesture}' (powtórzenie {i+1}/{repetitions})...")
            collect_data(port, baud_rate, duration, gesture)
            print("Czekaj 1 sekundy przed kolejnym nagraniem...")
            time.sleep(1)

if __name__ == "__main__":
    main()

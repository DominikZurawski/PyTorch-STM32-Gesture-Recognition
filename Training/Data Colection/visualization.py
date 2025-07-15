import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import time

# Konfiguracja portu szeregowego
port = 'COM3'  # Zmień na odpowiedni port
baud_rate = 115200

# Konfiguracja wykresu
max_points = 100  # Maksymalna liczba punktów na wykresie
x_data = deque(maxlen=max_points)
y_data = deque(maxlen=max_points)
z_data = deque(maxlen=max_points)
time_data = deque(maxlen=max_points)

# Inicjalizacja portu szeregowego
ser = serial.Serial(port, baud_rate, timeout=1)

# Inicjalizacja wykresu
fig, ax = plt.subplots(figsize=(10, 6))
line_x, = ax.plot([], [], 'r-', label='X')
line_y, = ax.plot([], [], 'g-', label='Y')
line_z, = ax.plot([], [], 'b-', label='Z')

ax.set_title('Dane z akcelerometru ADXL345 w czasie rzeczywistym')
ax.set_xlabel('Czas [s]')
ax.set_ylabel('Przyspieszenie')
ax.set_ylim(-2000, 2000)  # Dostosuj zakres do danych
ax.legend()
ax.grid(True)

start_time = None

def init():
    """Inicjalizacja animacji."""
    line_x.set_data([], [])
    line_y.set_data([], [])
    line_z.set_data([], [])
    return line_x, line_y, line_z

def update(frame):
    """Aktualizacja wykresu."""
    global start_time

    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        if line:
            try:
                x, y, z = map(int, line.split(','))

                if start_time is None:
                    start_time = time.time()

                current_time = time.time() - start_time

                time_data.append(current_time)
                x_data.append(x)
                y_data.append(y)
                z_data.append(z)

                # Aktualizacja danych na wykresie
                line_x.set_data(time_data, x_data)
                line_y.set_data(time_data, y_data)
                line_z.set_data(time_data, z_data)

                # Dostosowanie zakresu osi X
                ax.set_xlim(max(0, current_time - 10), current_time + 0.5)

            except ValueError:
                pass

    return line_x, line_y, line_z

# Uruchomienie animacji
ani = FuncAnimation(fig, update, init_func=init, interval=20, blit=True)
plt.tight_layout()
plt.show()

# Zamknięcie portu szeregowego po zakończeniu
ser.close()

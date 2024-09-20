import numpy as np
import matplotlib.pyplot as plt

# Datei einlesen, die Werte durch ein Semikolon getrennt
data = np.loadtxt('data.txt', delimiter=';')

# Beide Spalten als y-Werte interpretieren
y1 = data[:, 0]  # Erste Spalte
y2 = data[:, 1]  # Zweite Spalte
y3 = data[:, 2]  # Zweite Spalte

# x-Achse erstellen: Fortlaufende Werte basierend auf der Länge der Daten
x = np.arange(len(y1))

# Plotten der beiden y-Werte gegen die impliziten x-Werte
plt.plot(x, y1, label="sp")
plt.plot(x, y2, label="v")
plt.plot(x, y3, label="v_filter")

# Titel und Achsenbeschriftungen hinzufügen
#plt.title("Darstellung der y-Werte")
plt.xlabel("time")
plt.ylabel("speed")

# Legende und Gitter hinzufügen
plt.legend()
plt.grid(True)
plt.show()

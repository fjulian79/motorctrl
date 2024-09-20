import numpy as np
import matplotlib.pyplot as plt

# Daten einlesen
data = np.loadtxt('data.txt', delimiter=';')

# Wähle das Signal (z.B. die erste Spalte y1)
y1 = data[:, 1]

# Anzahl der Datenpunkte und Abtastrate (z.B. 1000 Hz)
n = len(y1)
fs = 200  # Abtastrate in Hz

# Berechne die FFT des Signals
y1_fft = np.fft.fft(y1)

# Berechne die zugehörigen Frequenzen
frequencies = np.fft.fftfreq(n, d=1/fs)

# Plotte das Frequenzspektrum (nur die positive Hälfte)
plt.plot(frequencies[1:n // 2], np.abs(y1_fft)[1:n // 2])
plt.title("Frequenzspektrum von y1")
plt.xlabel("Frequenz (Hz)")
plt.ylabel("Amplitude")
plt.grid(True)
plt.show()

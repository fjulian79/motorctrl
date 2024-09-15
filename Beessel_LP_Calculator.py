from scipy import signal

# Abtastrate und Grenzfrequenz
fs = 1000  # Abtastrate
cutoff = 30  # Grenzfrequenz

# Berechnung eines Bessel-Tiefpassfilters 2. Ordnung
b, a = signal.bessel(N=2, Wn=cutoff, btype='low', fs=fs)

print("Numerator coefficients (b):", b)
print("Denominator coefficients (a):", a)
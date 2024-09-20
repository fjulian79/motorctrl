from scipy import signal

# Abtastrate und Grenzfrequenz
fs = 500  # Abtastrate in Hz
cutoff = 30  # Grenzfrequenz in Hz
order = 1  # Ordnung des Filters

# Berechnung eines Butterworth-Tiefpassfilters
b, a = signal.butter(N=order, Wn=cutoff, btype='low', fs=fs)

# Ausgabe der Koeffizienten im C-Format
print("float b[{}] = {{ {:.8f}".format(len(b), b[0]), end="")
for coef in b[1:]:
    print(", {:.8f}".format(coef), end="")
print(" };")

print("float a[{}] = {{ {:.8f}".format(len(a), a[0]), end="")
for coef in a[1:]:
    print(", {:.8f}".format(coef), end="")
print(" };")

# C Code first order
# float filter_butterworth(float input) 
# {
#     float b[2] = { 0.13672874, 0.13672874 };
#     float a[2] = { 1.00000000, -0.72654253 };
#     static float prev_input = 0.0;
#     static float prev_output = 0.0;
# 
#     float output = b[0] * input + b[1] * prev_input - a[1] * prev_output;
#     prev_input = input;
#     prev_output = output;
# 
#     return output;
# }


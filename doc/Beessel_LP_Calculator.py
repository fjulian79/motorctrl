from scipy import signal

# Abtastrate und Grenzfrequenz
fs = 500  # Abtastrate in Hz
cutoff = 25  # Grenzfrequenz in Hz
order = 2  # Ordnung des Filters

# Berechnung eines Bessel-Tiefpassfilters 2. Ordnung
b, a = signal.bessel(N=order, Wn=cutoff, btype='low', fs=fs)

# Ausgabe der Koeffizienten im C-Format
print("float b[{}] = {{ {:.8f}".format(len(b), b[0]), end="")
for coef in b[1:]:
    print(", {:.8f}".format(coef), end="")
print(" };")

print("float a[{}] = {{ {:.8f}".format(len(a), a[0]), end="")
for coef in a[1:]:
    print(", {:.8f}".format(coef), end="")
print(" };")

# C Code
# float filter_bessel(float input) 
# {
#     const float b[3] = {0.06745527, 0.13491055, 0.06745527};
#     const float a[3] = {1.0, -1.1429805, 0.4128016};
#     static float lastInput[2] = {0.0, 0.0};
#     static float lastOutput[2] = {0.0, 0.0};
# 
#     float output = b[0] * input + b[1] * lastInput[0] + b[2] * lastInput[1] 
#             - a[1] * lastOutput[0] - a[2] * lastOutput[1];
# 
#     lastInput[1] = lastInput[0];
#     lastInput[0] = input;
#     lastOutput[1] = lastOutput[0];
#     lastOutput[0] = output;
# 
#     return output;
# }
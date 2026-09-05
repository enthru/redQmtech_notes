library(signal)

# CIC filter parameters
R <- 160                       # Decimation factor, mid range
# The CIC rate is programmable from 5 (6144 kHz) to 640 (48 kHz); the
# compensation term converges to its R -> inf limit fast enough that one
# set of coefficients serves every rate -- the worst case, R=5, deviates
# by 0.17 dB at the band edge, and R >= 20 by under 0.01 dB.
M <- 1                         # Differential delay
N <- 6                         # Number of stages

Fo <- 0.22                     # Pass band edge

# fir2 parameters
k <- kaiserord(c(Fo, Fo+0.02), c(1, 0), 1/(2^16), 1)
L <- k$n                       # Filter order
Beta <- k$beta                 # Kaiser window parameter

# FIR filter design using fir2
s <- 0.001                     # Step size
fp <- seq(0.0, Fo, by=s)       # Pass band frequency samples
fs <- seq(Fo+0.02, 0.5, by=s)  # Stop band frequency samples
f <- c(fp, fs)*2               # Normalized frequency samples; 0<=f<=1

Mp <- matrix(1, 1, length(fp)) # Pass band response; Mp[1]=1
Mp[-1] <- abs(M*R*sin(pi*fp[-1]/R)/sin(pi*M*fp[-1]))^N
Mf <- c(Mp, matrix(0, 1, length(fs)))

h <- fir2(L, f, Mf, window=kaiser(L+1, Beta))

h <- h / sum(h)

# Print filter coefficients
paste(sprintf("%.10e", h), collapse=", ")

fh <- freqz(h)

op <- par(mfrow = c(1, 2))

plot(f, Mf, type = "b", ylab = "magnitude", xlab = "Frequency", xlim = c(0, 0.5))
lines(fh$f / pi, abs(fh$h), col = "blue")

plot(f, 20*log10(Mf+1e-5), type = "b", ylab = "dB", xlab = "Frequency", xlim = c(0, 0.5))
lines(fh$f / pi, 20*log10(abs(fh$h)), col = "blue")

par(op)

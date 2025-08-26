%%  PRÁCTICA 5: Beamforming Delay-and-Sum para realce de voz en ambiente ruidoso
% --------------------------------------------------------------
% Autores: Enrique Alcalá-Zamora Castro
%          Andrea Bañón Triguero
% --------------------------------------------------------------
% Descripción:
% En esta práctica se implementa un beamformer Delay-and-Sum (D&S)
% en el dominio de la frecuencia para mejorar la calidad de una
% señal de voz capturada por un array lineal de micrófonos, en
% presencia de ruido interferente.
%
% - Transformación de las señales al dominio frecuencial mediante STFT.
% - Aplicación de beamforming D&S sobre cada frecuencia (modelo banda estrecha).
% - Reconstrucción temporal mediante IFFT y Overlap-Add (OLA).
% - Cálculo de la mejora de la SNR antes y después del procesamiento.
% - Representación de los patrones de directividad del array para
%   distintas frecuencias y análisis del comportamiento espacial.
%
% Objetivo:
% Realzar la señal procedente de una dirección deseada (φ_s = π/4),
% atenuando las interferencias desde otras direcciones.
%
% --------------------------------------------------------------
clear; close all; clc;

%% === PARÁMETROS DEL SISTEMA ===
Fs = 16000;              % Frecuencia de muestreo [Hz]
phi_s = pi/4;            % Dirección (ángulo en radianes) de procedencia de la voz
c = 340;                 % Velocidad del sonido [m/s]
N = 7;                   % Número de micrófonos del array
d = 0.04;                % Separación entre micrófonos [m]

%% === PARÁMETROS DE STFT ===
L = 500;                 % Tamaño de cada ventana (número de muestras por frame)
Lshift = L/2;            % Desplazamiento entre ventanas (50% de solapamiento)
Lfft = 512;              % Número de puntos FFT por ventana (>= L + Lfilt - 1)
win = sqrt(hanning(L, 'periodic'));  % Ventana de análisis (Hanning modificada con sqrt para cumplir COLA)

%% === CARGA DE SEÑALES ===
load('signals_array.mat');   % Carga las señales xc{i} (i = 1 a N) y la señal original xorg16
for i = 1:N
    Xc(i,:) = xc{i}(:).';    % Convierte cada celda a fila de la matriz Xc
end
len = size(Xc, 2);           % Longitud total de la señal (en muestras)

% Cálculo del número de tramas
numFrames = floor((len - L)/Lshift);
Xout = zeros(1, len + Lfft);   % Inicializa la señal de salida (con espacio para OLA)

%% === STFT + BEAMFORMING DELAY-AND-SUM + IFFT + OLA ===
for t = 1:numFrames
    idx = (t-1)*Lshift + (1:L);  % Índices del frame actual
    
    % === STFT por canal ===
    for i = 1:N
        frame = Xc(i, idx) .* win';        % Aplica ventana a la señal de cada micrófono
        Xi(:, i) = fft(frame, Lfft);       % FFT del frame (convertido a dominio de la frecuencia)
    end

    % === BEAMFORMING D&S EN FRECUENCIA ===
    Xbeam = zeros(Lfft, 1);  % Inicializa el espectro de salida del frame
    for k = 1:Lfft/2+1       % Solo se calcula hasta Nyquist (espectro real es simétrico)
        freq = 2*pi*(k-1)/Lfft*Fs;  % Frecuencia angular ω_k correspondiente al bin k

        % Vector de retardos (steering vector) para la dirección deseada (phi_s)
        steering = exp(-1j * freq * (0:N-1)' * d * cos(phi_s) / c);

        % Combinación lineal ponderada de las señales de los N micrófonos
        % conj(steering)' alinea las fases para la fuente deseada
        Xbeam(k) = (conj(steering)' / N) * Xi(k, :).';  % Delay-and-Sum en frecuencia
    end

    % === COMPLETAR EL ESPECTRO CON SIMETRÍA CONJUGADA ===
    Xbeam(Lfft/2+2:Lfft) = conj(Xbeam(Lfft/2:-1:2));
    %plot(abs(Xbeam));

    % === IFFT PARA PASAR A DOMINIO TEMPORAL ===
    xframe = real(ifft(Xbeam));  % Recupera el frame procesado en el tiempo (solo parte real)

    % === SÍNTESIS OVERLAP-ADD (OLA) ===
    Xout((t-1)*Lshift + (1:Lfft)) = Xout((t-1)*Lshift + (1:Lfft)) + xframe';
end

% Recorta la señal de salida a su longitud original
xout = Xout(1:len);

%% === CÁLCULO DE SNR ANTES Y DESPUÉS DEL BEAMFORMING ===

% Igualar longitudes de entrada y salida por seguridad
minLen = min(length(Xc(2,:)), length(xout));
xout = xout(1:minLen);

% Define regiones para ruido y señal
noise_seg = 1:3000;             % Primeras 3000 muestras → solo ruido
signal_seg = 3001:minLen;       % El resto → voz + ruido

% Cálculo de potencia antes del beamforming (usando micrófono 2 como referencia)
P_signal_in = var(Xc(2,signal_seg));
P_noise_in  = var(Xc(2,noise_seg));
snr_in_db   = 10 * log10(P_signal_in / P_noise_in);

% Cálculo de potencia después del beamforming
P_signal_out = var(xout(signal_seg));
P_noise_out  = var(xout(noise_seg));
snr_out_db   = 10 * log10(P_signal_out / P_noise_out);

% Muestra los resultados por consola
fprintf('SNR antes del beamforming: %.2f dB\n', snr_in_db);
fprintf('SNR después del beamforming: %.2f dB\n', snr_out_db);


%% === PATRÓN DE DIRECTIVIDAD DEL BEAMFORMER ===
frecs = [100, 400, 700, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000];  % Frecuencias a evaluar
theta = linspace(0, 2*pi, 360);  % Ángulos de incidencia (en radianes) de 0 a 2π

figure;
for f = frecs
    steering = zeros(length(theta), 1);  % Vector para la directividad a cada ángulo
    
    for i = 1:length(theta)
        tau = (0:N-1) * d * cos(theta(i)) / c;        % Retardos desde cada dirección θ_i
        delays = exp(-1j * 2*pi*f * tau);             % Vector de retardos para esa dirección
        w = exp(1j * 2*pi*f * (0:N-1) * d * cos(phi_s) / c);  % Pesos del beamformer (dirección deseada)
        steering(i) = abs(sum(delays .* w));          % Valor absoluto de la respuesta del array
    end
    
    steering = steering / max(steering);  % Normalización del patrón
    polarplot(theta, steering);           % Gráfica polar del patrón
    hold on;
end
title('Patrones de directividad del beamformer');
legend(arrayfun(@(f) sprintf('%d Hz', f), frecs, 'UniformOutput', false), 'Location', 'southoutside');

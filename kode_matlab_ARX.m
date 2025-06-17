% Program MATLAB untuk Penyetelan PID berdasarkan Fungsi Transfer
% Fungsi Transfer: G(s) = (0.2088s^2 + 1.315s + 1.686) / (s^3 + 0.6671s^2 + 9.981s + 0.01418)

% Langkah 1: Definisikan fungsi transfer sistem
num = [0.2088 1.315 1.686]; % Numerator
den = [1 0.6671 9.981 0.01418]; % Denominator
G = tf(num, den);

% Langkah 2: Hitung parameter awal menggunakan pendekatan IMC sederhana
% Aproksimasi konstanta waktu dominan berdasarkan koefisien
% Asumsi tau1 dan tau2 dari kutub dominan (perkiraan berdasarkan orde)
tau1 = 0.1; % Konstanta waktu awal (estimasi)
tau2 = 0.1; % Konstanta waktu awal (estimasi)
tau_c = 1;  % Waktu closed-loop desired (pilih 1-2 kali konstanta waktu)

% Gain statis (DC gain) diperkirakan dari rasio konstanta
K = dcgain(G); % Hitung gain statis sebenarnya

% Parameter IMC
Kp = (1/K) * (tau1 + tau2) / (2*tau_c + tau1 + tau2);
Ti = tau1 + tau2;
Td = (tau1 * tau2) / (tau1 + tau2);

% Konversi ke bentuk PID standar
Ki = Kp / Ti;
Kd = Kp * Td;

% Buat objek PID
C = pid(Kp, Ki, Kd);

% Langkah 3: Simulasi sistem tertutup
T = feedback(G*C, 1); % Sistem tertutup dengan umpan balik unity

% Langkah 4: Analisis respons
% Plot respons langkah
figure;
step(T);
title('Respons Langkah Sistem Tertutup dengan PID');
xlabel('Waktu (detik)');
ylabel('Amplitudo');
grid on;

% Tampilkan informasi
disp('Parameter PID Awal (IMC):');
disp(['Kp = ', num2str(Kp)]);
disp(['Ki = ', num2str(Ki)]);
disp(['Kd = ', num2str(Kd)]);
disp('Catatan: Sesuaikan parameter jika respons tidak memuaskan.');

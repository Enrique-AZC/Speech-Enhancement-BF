# Delay-and-Sum Beamforming for Speech Enhancement in Noisy Environments

## Overview  
This project implements a **Delay-and-Sum (D&S) beamformer** in the frequency domain to enhance the quality of a speech signal captured by a **linear microphone array** in the presence of interfering noise sources.  

The system applies **frequency-domain beamforming** under the narrowband assumption and reconstructs the time-domain signal using **Inverse FFT (IFFT)** and **Overlap-Add (OLA)**. Performance is assessed by computing the **Signal-to-Noise Ratio (SNR)** before and after beamforming, and the spatial behavior of the array is analyzed through **directivity patterns** at different frequencies.  

---

## Authors  
- Enrique Alcalá-Zamora Castro  
- Andrea Bañón Triguero  

---

## Objectives  

- Implement **Delay-and-Sum beamforming** in the frequency domain.  
- Enhance a desired speech source arriving from a known direction \(\phi_s = \pi/4\).  
- Attenuate interference and noise from other directions.  
- Evaluate **SNR improvement** after beamforming.  
- Analyze the **directivity patterns** of the microphone array across frequencies.  

---

## Methodology  

### System Parameters  
- **Sampling rate:** \(Fs = 16 \, \text{kHz}\)  
- **Sound speed:** \(c = 340 \, \text{m/s}\)  
- **Microphones:** \(N = 7\), uniformly spaced  
- **Microphone spacing:** \(d = 0.04 \, \text{m}\)  
- **Target source direction:** \(\phi_s = \pi/4\) radians  

### STFT Parameters  
- Frame length: \(L = 500\) samples  
- Overlap: \(50\%\) (\(L/2\))  
- FFT size: \(L_{fft} = 512\)  
- Window: Modified Hanning window (\(\sqrt{\text{Hanning}}\)) to satisfy COLA property  

### Processing Pipeline  
1. **Short-Time Fourier Transform (STFT):**  
   - Transform each microphone signal into the frequency domain per frame.  

2. **Frequency-Domain Delay-and-Sum:**  
   - For each frequency bin:  
     - Compute **steering vector** based on expected phase delays for the target direction.  
     - Align phases across microphones.  
     - Average signals coherently to enhance the desired source.  

3. **IFFT + Overlap-Add Reconstruction:**  
   - Convert processed frames back to time domain.  
   - Reconstruct the enhanced signal using OLA.  

4. **SNR Evaluation:**  
   - Compute input and output SNR using noise-only and speech+noise segments.  
   - Compare SNR improvement achieved by beamforming.  

5. **Directivity Analysis:**  
   - Compute **array beampatterns** at multiple frequencies (100 Hz – 8 kHz).  
   - Plot polar responses to visualize spatial selectivity.  

---

## Results  

- **Speech Enhancement:**  
  - Input SNR is computed from a reference microphone.  
  - Output SNR shows significant improvement after beamforming.  

- **Directivity Patterns:**  
  - At low frequencies, the array exhibits broad lobes (poor spatial selectivity).  
  - At higher frequencies, sharper main lobes and stronger side-lobe suppression are observed.  

- **Practical Insight:**  
  - Beamforming improves robustness against directional interference, but its effectiveness strongly depends on frequency and array configuration.  

---

## Technical Relevance  

This project demonstrates:  
- **Frequency-domain beamforming** under the narrowband approximation.  
- **Spatial filtering** for speech enhancement using microphone arrays.  
- **Trade-offs** between resolution, frequency, and array geometry.  
- **SNR-based evaluation** of array signal processing techniques.  
- Practical application of **STFT and OLA reconstruction** in multichannel speech processing.  

---

## Future Work  

- Implement **Minimum Variance Distortionless Response (MVDR) beamformer** for adaptive interference suppression.  
- Incorporate **time-varying noise sources** and moving speakers.  
- Compare D&S with **subspace methods** (MUSIC, ESPRIT) for direction-of-arrival (DOA) estimation.  
- Extend to **2D microphone arrays** for spatial filtering in both azimuth and elevation.  
- Explore **real-time implementation** in embedded audio systems.  

---

## Requirements  

- **MATLAB R2021a+** or **GNU Octave 6.0+**  
- Input file: `signals_array.mat` containing:  
  - Microphone signals `xc{i}` (i = 1 … N)  
  - Original clean speech `xorg16`  

---

## License  

This work is distributed for academic and research purposes. Attribution is required.  

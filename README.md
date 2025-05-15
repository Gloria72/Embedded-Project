# Embedded-Project
# Tremor & Dyskinesia Detector  
Real‑time embedded system that classifies hand‑motion abnormalities (Parkinsonian tremor vs. levodopa‑induced dyskinesia) from wrist‑mounted IMU data and signals the result with distinct LED blink patterns.

---

## 1. How It Works
1. **Sampling** – The LSM6DSL IMU streams Z‑axis acceleration at **100 Hz**.  
2. **Buffering** – A 256‑sample circular buffer (≈ 2.56 s) is continuously filled.  
3. **Window Processing** – When full, the buffer is windowed (Hamming) and fed to an in‑place **256‑pt FFT**.  
4. **Peak Analysis** –  
   * Tremor band : 2.7 – 4.5 Hz (bins 7 – 10)  
   * Dyskinesia band : 4.7 – 7.0 Hz (bins 12 – 18)  
   The larger band‑peak above **1500 ADU** triggers classification.  
5. **Feedback** –  
   * **Tremor** → LED1 blinks at **2 Hz** (250 ms toggle)  
   * **Dyskinesia** → LED2 blinks at **8 Hz** (63 ms toggle)  
   * No event → LEDs off  

---




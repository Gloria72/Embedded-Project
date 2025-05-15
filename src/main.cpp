#include "mbed.h"
#include "DevI2C.h"
#include "LSM6DSLSensor.h"
#include "ArduinoFFT.h"
#include <cmath>

using namespace std::chrono;


/* ===== Parameters ===== */
constexpr uint32_t SAMPLE_RATE_HZ = 100;     // Sampling rate in Hz
constexpr uint32_t N_SAMPLES      = 256;     /// Number of samples, must be power of 2
constexpr float    WINDOW_SEC     = N_SAMPLES / (float)SAMPLE_RATE_HZ; // ≈2.56 s

constexpr float    MAG_THRESHOLD  = 1500.0f;       // 峰值幅度阈值，可现场调





/* ===== Hardware Objects ===== */
static BufferedSerial pc(USBTX, USBRX, 115200);
static DevI2C devI2C(PB_11, PB_10);                // I²C2
static LSM6DSLSensor imu(&devI2C, 0xD4);           // 0x6A<<1
DigitalOut led(LED1);
Ticker      tickSample;
Ticker      tickLed;

/* ===== Sample Buffer ===== */
static int16_t  ringBuf[N_SAMPLES];
static uint32_t head = 0;
static volatile bool sample_flag  = false;
static volatile bool window_ready = false;

/* ===== LED Modes ===== */
enum class LedMode { OFF, TREMOR, DYSK };
static volatile LedMode ledMode = LedMode::OFF;

/* ===== Sampling Ticker Callback: triggered every 10 ms ===== */
void tickSampleCB()
{
    sample_flag = true;    
}



/* ===== LED Blinking Callback ===== */
void tickLedCB()
{
    static bool state = false;
    switch (ledMode) {
        case LedMode::TREMOR: state = !state; led = state; break;
        case LedMode::DYSK:   state = !state; led = state; break;
        case LedMode::OFF:    led = 0; break;
    }
    
}

/* ===== Utility: Update LED blink frequency based on mode ===== */
void setLedMode(LedMode mode)
{
    ledMode = mode;
    tickLed.detach();

    if (mode == LedMode::TREMOR) {
        tickLed.attach(tickLedCB, 250ms);   // 2 Hz
    } else if (mode == LedMode::DYSK) {
        tickLed.attach(tickLedCB, 63ms);    // 8 Hz
    } else {
        led = 0;                            // 熄灭
    }
}
/* ===== Main Program ===== */
int main()
{
    // 强制链接器包含浮点格式化代码
    __asm__(".global _printf_float");
    printf("\r\n=== Tremor / Dyskinesia Detector ===\r\n");

    if (imu.init(NULL) != 0 || imu.enable_x() != 0) {
        printf("IMU init FAILED!\r\n");
        while (true) { led = !led; ThisThread::sleep_for(100ms); }
    }

    tickSample.attach(callback(tickSampleCB), 0.01f);   // 0.01 s = 10 ms

    /* FFT 缓冲 */
    static double vReal[N_SAMPLES];
    static double vImag[N_SAMPLES];
    arduinoFFT FFT(vReal, vImag, N_SAMPLES, SAMPLE_RATE_HZ);

    int32_t acc[3];

    static int dbgCnt = 0;
    while (true) {


        /* ---------- 采样 ---------- */
        if (sample_flag) {
            sample_flag = false;
        
            imu.get_x_axes(acc);
            ringBuf[head] = (int16_t)acc[2];
            head = (head + 1) % N_SAMPLES;
            if (head == 0) window_ready = true;   // 这次写到 299 → 0
        }

        /* ---------- FFT 与判定 ---------- */
        if (window_ready) {
            window_ready = false;
        
            printf("[DBG] window ready! do FFT\r\n");   // ← 只打这一行

            /* 把环形缓冲复制到 vReal[]，并做 Hamming 窗 */
            for (uint32_t i = 0; i < N_SAMPLES; ++i) {
                uint32_t idx = (head + i) % N_SAMPLES;     // 从最新写指针往后拼
                vReal[i] = (double)ringBuf[idx];
                vImag[i] = 0.0;
            }

            FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
            FFT.Compute(FFT_FORWARD);
            FFT.ComplexToMagnitude();
                   

            /* 频率 bin 区间：bin = f * N / Fs */
            constexpr uint16_t BIN_START_TREMOR =  7;   //  ≈2.7 Hz
            constexpr uint16_t BIN_END_TREMOR   = 10;   //  <4.5 Hz
            constexpr uint16_t BIN_START_DYSK   = 12;   //  ≈4.7 Hz
            constexpr uint16_t BIN_END_DYSK     = 18;   //  <7.0 Hz

            double peakTremor = 0, peakDysk = 0;
            for (uint16_t i = BIN_START_TREMOR; i <= BIN_END_TREMOR; ++i)
                if (vReal[i] > peakTremor) peakTremor = vReal[i];
            for (uint16_t i = BIN_START_DYSK; i <= BIN_END_DYSK; ++i)
                if (vReal[i] > peakDysk) peakDysk = vReal[i];

            printf("[FFT] peakTremor=%d  peakDysk=%d\r\n", (int)peakTremor, (int)peakDysk);

            /* 判定 + LED */
            if (peakTremor > MAG_THRESHOLD && peakTremor > peakDysk) {
                printf(">> Tremor detected!\r\n");
                setLedMode(LedMode::TREMOR);
            } else if (peakDysk > MAG_THRESHOLD && peakDysk > peakTremor) {
                printf(">> Dyskinesia detected!\r\n");
                setLedMode(LedMode::DYSK);
            } else {
                setLedMode(LedMode::OFF);
            }
        }

        ThisThread::sleep_for(2ms);   // 让出 CPU
    }
}

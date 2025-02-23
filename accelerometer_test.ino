#include <Wire.h>

// MMA8452Q I2C Address (Check if yours is 0x1C or 0x1D)
#define MMA8452Q_ADDR 0x1D  

void setup() {
    Serial.begin(9600);
    Wire.begin();  // Initialize I2C

    if (!initMMA8452Q()) {
        Serial.println("MMA8452Q not detected!");
        while (1);  // Stop execution if sensor is not found
    }
    Serial.println("MMA8452Q Initialized.");
}

void loop() {
    float x, y, z;
    readAccelerometer(x, y, z);

    Serial.print("X: "); Serial.print(x, 3); 
    Serial.print(" Y: "); Serial.print(y, 3);
    Serial.print(" Z: "); Serial.println(z, 3);

    delay(500);  // Read every 500ms
}

// Initialize MMA8452Q
bool initMMA8452Q() {
    Wire.beginTransmission(MMA8452Q_ADDR);
    Wire.write(0x2A);  // Control register
    Wire.write(0x01);  // Set to active mode
    return Wire.endTransmission() == 0;
}

// Read acceleration data
void readAccelerometer(float &x, float &y, float &z) {
    Wire.beginTransmission(MMA8452Q_ADDR);
    Wire.write(0x01);  // Start at OUT_X_MSB register
    Wire.endTransmission(false);
    Wire.requestFrom(MMA8452Q_ADDR, 6);  // Request 6 bytes (X, Y, Z)

    if (Wire.available() == 6) {
        int16_t rawX = (Wire.read() << 8 | Wire.read()) >> 4;
        int16_t rawY = (Wire.read() << 8 | Wire.read()) >> 4;
        int16_t rawZ = (Wire.read() << 8 | Wire.read()) >> 4;

        x = rawX * 0.000244;  // Convert to g (assuming ±2g range)
        y = rawY * 0.000244;
        z = rawZ * 0.000244;
    }
}
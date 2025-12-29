#include "Motor.h"
#include "Sender.h"
#include "DistanceSensor.h"
#include "HallSensor.h"
#include "PINS.h"

#define numRecords 100

Sender sender;
Motor motor(M_STEP, M_DIR, M_EN, M_MS1, M_MS2);
DistanceSensor TFLuna(TF_RX, TF_TX, Serial1);
HallSensor Hall(HS_ANALOG);

int count = 0;
TaskHandle_t senderTaskHandle;
std::vector<uint8_t> data_to_send = {};

void sendTask(void *parameter) {
    while (1) {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
            sender.checkIPs();
            sender.sendParameters(data_to_send);
            data_to_send.clear();
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n--- gLIDAR starting ---");

    sender.begin("gLIDAR", "12345678", 1234);
    motor.setDirection(0);
    motor.setVelocity(300);
    TFLuna.begin();
    
    xTaskCreatePinnedToCore(
        sendTask,
        "SendTask",
        4096,
        NULL,
        1,
        &senderTaskHandle, 
        1
    );
}

void loop() {
    TFLuna.collectData(motor, numRecords);
    motor.rotate();
    int hall_value = analogReadRaw(32);
    if (hall_value == 0 && motor.getAngle() > 90) {
        data_to_send.push_back('L');
        data_to_send.push_back('I');
        data_to_send.push_back('D');
        data_to_send.push_back('A');
        data_to_send.push_back('R');
        auto tf_luna_data = TFLuna.getData();
        auto size = tf_luna_data.size();
        uint8_t lsb_size = size & 0xFF;
        uint8_t msb_size = (size >> 8) & 0xFF;
        data_to_send.push_back(lsb_size);
        data_to_send.push_back(msb_size);
        data_to_send.insert(data_to_send.end(), tf_luna_data.begin(), tf_luna_data.end());
        data_to_send.push_back('E');
        data_to_send.push_back('N');
        data_to_send.push_back('D');
        data_to_send.push_back('\n');
        data_to_send.push_back('\r');
        TFLuna.resetData();
        xTaskNotifyGive(senderTaskHandle);
        motor.resetAngle();
    } else if (motor.getAngle() > 550) {
        motor.resetAngle();
        TFLuna.resetData();
    }
}
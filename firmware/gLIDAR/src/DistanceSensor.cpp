#include "DistanceSensor.h"

DistanceSensor::DistanceSensor(int rx, int tx, HardwareSerial &serial)
    : RX(rx), TX(tx), lidarSerial(serial) {}

void DistanceSensor::begin()
{
    lidarSerial.begin(115200, SERIAL_8N1, RX, TX);
    Serial.println("Konfiguracja czujnika");
    delay(100);

    int retries = 5;
    while (retries-- > 0 && !disableDataOutput()) {
        Serial.println("Ponowna próba wyłączenia strumienia danych");
        delay(100);
        if(retries == 1) lidarSerial.updateBaudRate(921600);
    }
    if (retries < 0 && !disableDataOutput()) { 
         Serial.println("Nie udało się wyłączyć strumienia danych");
         return;
    }
    Serial.println("Strumień danych pomyślnie wyłączony");
    delay(100);

    setBaudrate(); 

    lidarSerial.flush();
    lidarSerial.updateBaudRate(921600);
    Serial.println("Zmieniono Baud Rate na 921600");
    delay(100); 

    checkFirmwareVersion();
    delay(100);

    if (!setSampleFrequency()) {
        Serial.println("Nie udało się ustawić częstotliwości próbkowania");
    }
    delay(100);
    
    if (!enableDataOutput()) {
        Serial.println("Nie udało się włączyć strumienia danych");
    }

    Serial.println("Konfiguracja czujnika zakończona");
}

void print_buffer_hex(HardwareSerial& serial_port) {
    if (serial_port.available() > 0) {
        Serial.print("Dane w buforze (HEX): ");
        while(serial_port.available()) Serial.printf("%02X ", serial_port.read());
        Serial.println();
    }
}

void DistanceSensor::checkFirmwareVersion() {
    uint8_t command[] = {0x5A, 0x04, 0x01, 0x5F};
    Serial.println("Sprawdzanie wersji oprogramowania");
    
    while(lidarSerial.available()) lidarSerial.read();
    lidarSerial.write(command, sizeof(command));

    unsigned long startTime = millis();
    const unsigned long timeout = 200;
    const int expected_len = 7;
    
    while (lidarSerial.available() < expected_len) {
        if (millis() - startTime > timeout) {
            Serial.println("Timeout przy odczycie wersji oprogramowania");
            print_buffer_hex(lidarSerial);
            return;
        }
    }

    uint8_t response[expected_len];
    lidarSerial.readBytes(response, expected_len);

    Serial.print("Odpowiedź na wersję oprogramowania (RAW HEX): ");
    for (int i=0; i<expected_len; i++) {
        Serial.printf("%02X ", response[i]);
    }
    Serial.println();

    if (response[0] != 0x5A || response[1] != 0x07 || response[2] != 0x01) {
        Serial.println("Otrzymano nieprawidłową odpowiedź wersji oprogramowania");
        return;
    }
    
    uint8_t checksum = 0;
    for (int i = 0; i < expected_len - 1; i++) {
        checksum += response[i];
    }

    if (checksum != response[expected_len - 1]) {
        Serial.printf("Nieprawidłowa suma kontrolna odpowiedzi wersji oprogramowania (Oczekiwano: 0x%02X, Otrzymano: 0x%02X).\n", response[expected_len-1], checksum);
        return;
    }

    Serial.printf("Wersja oprogramowania czujnika: V%d.%d.%d\n", response[5], response[4], response[3]);
}


bool DistanceSensor::disableDataOutput() {
    uint8_t command[] = {0x5A, 0x05, 0x07, 0x00, 0x66};
    Serial.println("Wyłączanie strumienia danych");
    return sendCommand(command, sizeof(command));
}

bool DistanceSensor::enableDataOutput() {
    uint8_t command[] = {0x5A, 0x05, 0x07, 0x01, 0x67};
    Serial.println("Włączanie strumienia danych");
    return sendCommand(command, sizeof(command));
}

void DistanceSensor::setBaudrate()
{
    uint8_t command[] = {0x5A, 0x08, 0x06, 0x00, 0x10, 0x0E, 0x00, 0x86};
    Serial.println("Zmiana Baud Rate na 921600");
    
    while(lidarSerial.available()) lidarSerial.read();
    lidarSerial.write(command, sizeof(command));
    delay(50);
}

bool DistanceSensor::setSampleFrequency()
{
    uint8_t command[] = {0x5A, 0x06, 0x03, 0xFA, 0x00, 0x5D};
    Serial.println("Ustawianie częstotliwości próbkowania");
    return sendCommand(command, sizeof(command));
}

bool DistanceSensor::sendCommand(uint8_t *command, size_t length)
{
    while(lidarSerial.available()) lidarSerial.read();
    lidarSerial.write(command, length);

    unsigned long startTime = millis();
    const unsigned long timeout = 250;
    bool headerFound = false;
    while (millis() - startTime < timeout) {
        if (lidarSerial.available()) {
            if (lidarSerial.read() == 0x5A) {
                headerFound = true;
                break;
            }
        }
    }

    if (!headerFound) {
        Serial.println("Timeout - nie znaleziono nagłówka odpowiedzi 0x5A");
        return false;
    }

    uint8_t response[32]; 
    response[0] = 0x5A;

    startTime = millis();
    while (lidarSerial.available() < 1) {
        if (millis() - startTime > 50) {
            Serial.println("Timeout - brak bajtu długości po nagłówku");
            return false;
        }
    }
    uint8_t len = lidarSerial.read(); 
    response[1] = len;

    if (len > sizeof(response) || len < 4) {
        Serial.printf("Nieprawidłowa długość ramki w odpowiedzi (długość: %d)\n", len);
        print_buffer_hex(lidarSerial);
        return false;
    }

    size_t bytesToRead = len - 2;
    startTime = millis();
    while (lidarSerial.available() < bytesToRead) {
        if (millis() - startTime > 100) {
            Serial.println("Timeout - niekompletna ramka odpowiedzi");
            print_buffer_hex(lidarSerial);
            return false;
        }
    }
    lidarSerial.readBytes(&response[2], bytesToRead);

    uint8_t checksum = 0;
    for (int i = 0; i < len - 1; i++) {
        checksum += response[i];
    }

    if (checksum != response[len - 1]) {
        Serial.printf("Suma kontrolna się nie zgadza (Oczekiwano: 0x%02X, Otrzymano: 0x%02X)\n", response[len-1], checksum);
        return false;
    }

    uint8_t commandID = command[2];
    uint8_t responseID = response[2];
    if (commandID != responseID) {
        Serial.printf("Niezgodne ID polecenia (Wysłano: 0x%02X, Otrzymano: 0x%02X)\n", commandID, responseID);
        return false;
    }

    Serial.printf("Otrzymano poprawną odpowiedź dla polecenia 0x%02X.\n", commandID);

    Serial.print("Cała ramka odpowiedzi (HEX): ");
    for (int i = 0; i < len; i++) {
        Serial.printf("%02X ", response[i]);
    }
    Serial.println();

    return true;
}

void DistanceSensor::readData() {
    uint8_t data[9];
  
    while (true) {  
        if (lidarSerial.available()) {
            lidarSerial.readBytes(data, 1);
            if (data[0] != 0x59) {
                continue;
            }
  
            if (lidarSerial.available() >= 8) {
                lidarSerial.readBytes(data + 1, 8);
                
                if (data[1] == 0x59) {
                    int distance = data[2] | (data[3] << 8);
                    Serial.printf("Odległość: %d cm\n", distance);
                }
            }
        }
    }
  }

Point DistanceSensor::receiveData()
{
    Point point;
    static uint8_t buffer[9];
    static int frame_idx = 0;
    const uint8_t HEADER = 0x59;

    while (lidarSerial.available()) {
        uint8_t incoming_byte = lidarSerial.read();

        if (frame_idx == 0) {
            if (incoming_byte == HEADER) {
                buffer[frame_idx] = incoming_byte;
                frame_idx++;
            }
        } else if (frame_idx == 1) {
            if (incoming_byte == HEADER) {
                buffer[frame_idx] = incoming_byte;
                frame_idx++;
            } else {
                frame_idx = 0; 
            }
        } else { 
            buffer[frame_idx] = incoming_byte;
            frame_idx++;

            if (frame_idx >= 9) {
                frame_idx = 0;

                uint8_t checksum = 0;
                for (int i = 0; i < 8; i++) {
                    checksum += buffer[i];
                }

                if (checksum == buffer[8]) {
                    int distance = buffer[2] | (buffer[3] << 8);
                    int strength = buffer[4] | (buffer[5] << 8);
                    point.setDistance(distance);
                    point.setStrength(strength);
                }
                return point;
            }
        }
    }
    
    return point;
}

void DistanceSensor::collectData(Motor motor, int numRecords)
{
    if (motor.getAngle() != 0)
    {
        float motorAngle = motor.getAngle();
        float stepPerRecord = 413.7 / numRecords;
        float tolerance = 0.9;
        if (fmod(motorAngle, stepPerRecord) < tolerance) {
            Point point = this->receiveData();
            point.setAngle(motor.getAngle());

            uint16_t strength = static_cast<uint16_t>(point.getStrength());

            uint16_t distance = static_cast<uint16_t>(point.getDistance());

            uint8_t lsb_distance = distance & 0xFF;
            uint8_t msb_distance = (distance >> 8) & 0xFF;

            uint8_t lsb_strength = strength & 0xFF;
            uint8_t msb_strength = (strength >> 8) & 0xFF;

            data.push_back(lsb_distance);
            data.push_back(msb_distance);
            data.push_back(lsb_strength);
            data.push_back(msb_strength);
        }
    }
    else
    {
        data.clear();
    }
}

std::vector<uint8_t> DistanceSensor::getData()
{
    return data;
}

std::vector<Point> DistanceSensor::getPoints()
{
    return points;
}

void DistanceSensor::resetData()
{
    data.clear();
}

void DistanceSensor::resetPoints()
{
    points.clear();
}

void DistanceSensor::addPoint()
{
    Point point = this->receiveData();
    points.push_back(point);
}
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <SoftwareSerial.h>
#include <U8g2lib.h>

// ==================== 配置设置 ====================
const char* const MAC_ADDRESS = "c6:86:a1:06:c3:88";
const char* const TARGET_SERVICE_UUID = "8650";
const char* const TARGET_CHARACTERISTIC_UUID = "8651";

constexpr uint16_t DEFAULT_STICK_VALUE = 1024;
constexpr uint16_t MAX_DBUS = 1684;
constexpr uint16_t MIN_DBUS = 364;

constexpr uint32_t SCAN_DURATION_MS = 5000;

uint16_t pressedKeys = 0;
uint16_t position = 0;
uint16_t leftDrag = 1024, leftPush = 1024, rightDrag = 1024, rightPush = 1024;
uint8_t s1 = 3, s2 = 2;
bool boardMode = true;

EspSoftwareSerial::UART swSer1;
EspSoftwareSerial::UART swSer2;

U8X8_SSD1306_128X64_NONAME_HW_I2C oledDisplay(U8X8_PIN_NONE);
uint8_t chassisDbusBuffer[18] = {0};
uint8_t armDbusBuffer[18] = {0};

NimBLEAdvertisedDevice* advertisedDevice = nullptr;
bool shouldConnect = false;
bool shouldUpdateDisplay = false;
bool isConnected = false;

// ==================== 函数声明 ====================
void setupNotifications(NimBLERemoteCharacteristic* characteristic);
void connectToDevice();
void startScan();
void succeedConnect();
void failConnect();

// ==================== BLE 回调 ====================
class ClientCallback : public NimBLEClientCallbacks {
public:
    void onConnect(NimBLEClient* client) override {
        Serial.println("Connected to device");
        succeedConnect();
    }

    void onDisconnect(NimBLEClient* client, int reason) override {
        Serial.printf("Disconnected from %s, reason: %d - Restarting scan\n",
                      client->getPeerAddress().toString().c_str(), reason);
        startScan();
        failConnect();
    }
} clientCallback;

class ScanCallback : public NimBLEScanCallbacks {
public:
    void onResult(const NimBLEAdvertisedDevice* device) override {
        Serial.printf("Found device: %s\n", device->toString().c_str());

        if (device->getAddress().equals(NimBLEAddress(MAC_ADDRESS, BLE_ADDR_RANDOM))) {
            Serial.println("Target device found!");
            NimBLEDevice::getScan()->stop();
            advertisedDevice = const_cast<NimBLEAdvertisedDevice*>(device);
            shouldConnect = true;
        }
    }

    void onScanEnd(const NimBLEScanResults& results, int reason) override {
        Serial.printf("Scan ended, reason: %d, devices found: %d - Restarting...\n", reason, results.getCount());
        startScan();
    }
} scanCallback;

// ==================== 数据解析 & 打包 ====================
void mapStickToDbus(uint16_t rawValue, uint16_t* mappedValue) {
    constexpr uint16_t range = MAX_DBUS - MIN_DBUS;
    *mappedValue = MAX_DBUS - ((rawValue * range) / 1023);
}

void packDbusData(uint16_t rightDrag, uint16_t rightPush,
                  uint16_t leftDrag, uint16_t leftPush,
                  uint8_t* buffer) {
    buffer[0] = rightDrag & 0xFF;
    buffer[1] = (rightDrag >> 8) | ((rightPush << 3) & 0xF8);
    buffer[2] = (rightPush >> 5) | ((leftDrag << 6) & 0xC0);
    buffer[3] = leftDrag >> 2;
    buffer[4] = (leftDrag >> 10) | ((leftPush << 1) & 0xFE);
    buffer[5] = (s2 << 6)|(s1 << 4)|(leftPush >> 7);
    buffer[17] = 0x00;
}

// ==================== BLE 特征通知回调 ====================
void notificationHandler(NimBLERemoteCharacteristic* characteristic,
                         uint8_t* data, size_t length, bool isNotify) {

    if (length < 19) return;

    uint16_t statusCode = (data[1] << 8) | data[0];
    if (statusCode != 50593) return;

    // Update global variables
    pressedKeys = (data[10] << 8) | data[9];
    position = (data[12] << 8) | data[11];

    uint8_t barRaw[5];
    memcpy(barRaw, data + 2, 5);

    uint16_t lDragRaw = 1024 - (barRaw[0] << 2) | (barRaw[1] >> 6);
    uint16_t lPushRaw = ((barRaw[1] & 0x3F) << 4) | (barRaw[2] >> 4);
    uint16_t rDragRaw = ((barRaw[2] & 0x0F) << 6) | (barRaw[3] >> 2);
    uint16_t rPushRaw = ((barRaw[3] & 0x03) << 8) | barRaw[4];

    if(!boardMode){
        rDragRaw = 512 + rDragRaw >> 1;
        rPushRaw = 512 + rPushRaw >> 1;
    }

    mapStickToDbus(lDragRaw, &leftDrag);
    mapStickToDbus(lPushRaw, &leftPush);
    mapStickToDbus(rDragRaw, &rightDrag);
    mapStickToDbus(rPushRaw, &rightPush);

    // 按键定义
    // A        1   （1 << 0）
    // B        2   （1 << 1）
    // MENU     4   （1 << 2）
    // X        8   （1 << 3）
    // Y       16   （1 << 4）
    // L1      64   （1 << 6）
    // R1     128   （1 << 7）
    // L2     256   （1 << 8）
    // R2     512   （1 << 9）
    // c1    1024   （1 << 10）
    // c2    2048   （1 << 11）
    // PWR   4096   （1 << 12）

    // 上   65536    0x10000
    // 右  196608    0x30000
    // 下  327680    0x50000
    // 左  458752    0x70000
    // 执行特定操作
    s1 = 1, s2 = 1;
    if (pressedKeys & (1 << 0)) { // A 键被按下
        // 执行A键对应的操作
    }else{

    }
    if (pressedKeys & (1 << 1)) { // B 键被按下
        // 执行B键对应的操作
    }else{
      
    }
    if (pressedKeys & (1 << 2)) { // MENU 键被按下
        // 执行MENU键对应的操作
    }else{
      
    }
    if (pressedKeys & (1 << 3)) { // X 键被按下
        // 执行X键对应的操作
    }else{
      
    }
    if (pressedKeys & (1 << 4)) { // Y 键被按下
        // 执行Y键对应的操作
    }else{
      
    }
    if (pressedKeys & (1 << 6)) { // L1 键被按下
        // 执行L1键对应的操作
        s1 = 2;
    }else if (pressedKeys & (1 << 8)) { // L2 键被按下
        // 执行L2键对应的操作
        s1 = 1;
    }else{
        s1 = 3;
    }

    if (pressedKeys & (1 << 7)) { // R1 键被按下
        // 执行R1键对应的操作
        s2 = 3;
    }else if (pressedKeys & (1 << 9)) { // R2 键被按下
        // 执行R2键对应的操作
        s2 = 1;
    }else{
        s2 = 2;
    }
    if (pressedKeys & (1 << 10)) { // c1 键被按下
        // 执行c1键对应的操作
        boardMode = true;
    }else{
      
    }
    if (pressedKeys & (1 << 11)) { // c2 键被按下
        // 执行c2键对应的操作
        boardMode = false;
    }else{
      
    }

    if (pressedKeys & (1 << 16)) { // 上方向键被按下
        // 执行向上键对应的操作
    }else if (position == 1){
        
    }else if (position == 2){
        
    }else if (position == 3){
        
    }else if (position == 4){
        
    }else if (position == 5){
        
    }else if (position == 6){
        
    }else if (position == 7){
        
    }else if (position == 8){
        
    }else{

    }

    packDbusData(rightDrag, rightPush, leftDrag, leftPush, boardMode ? chassisDbusBuffer : armDbusBuffer);

    shouldUpdateDisplay = true; // Mark display update needed
}

void succeedConnect(){
  oledDisplay.clear();
  isConnected = true;
  shouldUpdateDisplay = true;
}
void failConnect(){
  oledDisplay.clear();
  isConnected = false;
  shouldUpdateDisplay = true;
}
// ==================== 中断函数 ====================
void onTimerInterrupt(void *parameter) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while(1){
        if(isConnected){
            swSer1.write(chassisDbusBuffer, sizeof(chassisDbusBuffer));
            swSer2.write(armDbusBuffer, sizeof(armDbusBuffer));
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(14));
    }
}
// ==================== 显示更新函数 ====================
void updateDisplay(void *parameter) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while(1){
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));

        if (isConnected) {
            if (!shouldUpdateDisplay) {
                continue;
            }

            shouldUpdateDisplay = false;
            char buffer[32];

            snprintf(buffer, sizeof(buffer), "Key: %6u", pressedKeys);
            oledDisplay.drawString(0, 0, buffer);

            snprintf(buffer, sizeof(buffer), "Key: %6u", position);
            oledDisplay.drawString(0, 1, buffer);

            snprintf(buffer, sizeof(buffer), "LD:%4u LP:%4u", leftDrag, leftPush);
            oledDisplay.drawString(0, 2, buffer);

            snprintf(buffer, sizeof(buffer), "RD:%4u RP:%4u", rightDrag, rightPush);
            oledDisplay.drawString(0, 3, buffer);
        } else {
            // Display disconnection message
            oledDisplay.drawString(0, 0, "Disconnected");
            oledDisplay.drawString(0, 1, "Searching...");
            oledDisplay.drawString(0, 2, "Please wait.");
        }
        
    }
}
// ==================== 主逻辑 ====================
void setup() {
    Serial.begin(115200);

    pinMode(5, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);
    digitalWrite(5, LOW);
    digitalWrite(4, LOW);		

    swSer1.begin(100000, EspSoftwareSerial::SWSERIAL_8E2, 2, 0, true, 256);
    swSer1.enableIntTx(false);
    swSer2.begin(100000, EspSoftwareSerial::SWSERIAL_8E2, 10, 1, true, 256);
    swSer2.enableIntTx(false);

    oledDisplay.begin();
    oledDisplay.setPowerSave(false);
    oledDisplay.setFont(u8x8_font_chroma48medium8_r);
    oledDisplay.drawString(0, 1, "Loading...");

    packDbusData(DEFAULT_STICK_VALUE, DEFAULT_STICK_VALUE,
                 DEFAULT_STICK_VALUE, DEFAULT_STICK_VALUE, chassisDbusBuffer);
    packDbusData(DEFAULT_STICK_VALUE, DEFAULT_STICK_VALUE,
                 DEFAULT_STICK_VALUE, DEFAULT_STICK_VALUE, armDbusBuffer);

    xTaskCreate(
      updateDisplay,  // 任务函数指针
      "FreshOLED",     // 任务名称
      2048,           // 堆栈大小（字数，具体数值依赖于架构）
      NULL,           // 传递给任务函数的参数
      2,              // 任务优先级
      NULL);          // 任务句柄

    xTaskCreate(
      onTimerInterrupt,  // 任务函数指针
      "SendMSG",     // 任务名称
      2048,           // 堆栈大小（字数，具体数值依赖于架构）
      NULL,           // 传递给任务函数的参数
      2,              // 任务优先级
      NULL);          // 任务句柄

    NimBLEDevice::init("ESP32-GameController");

    auto scanner = NimBLEDevice::getScan();
    scanner->setScanCallbacks(&scanCallback, false);
    scanner->setActiveScan(true);
    scanner->setInterval(100);
    scanner->setWindow(100);
    scanner->start(SCAN_DURATION_MS);

    Serial.println("Scanning for controller...");
}

void loop() {
    if (shouldConnect) {
        shouldConnect = false;
        connectToDevice();
    }
    delay(10);
}

// ==================== 工具函数实现 ====================
void connectToDevice() {
    NimBLEClient* client = NimBLEDevice::createClient();
    client->setClientCallbacks(&clientCallback, false);
    client->setConnectionParams(12, 12, 0, 150);
    client->setConnectTimeout(5000);

    if (!client->connect(advertisedDevice)) {
        Serial.println("Failed to connect");
        NimBLEDevice::deleteClient(client);
        return;
    }

    Serial.printf("Connected to %s, RSSI: %d\n",
                  client->getPeerAddress().toString().c_str(),
                  client->getRssi());

    auto service = client->getService(TARGET_SERVICE_UUID);
    if (!service) {
        Serial.println("Service not found");
        client->disconnect();
        return;
    }

    auto characteristic = service->getCharacteristic(TARGET_CHARACTERISTIC_UUID);
    if (!characteristic) {
        Serial.println("Characteristic not found");
        client->disconnect();
        return;
    }

    if (characteristic->canNotify()) {
        if (!characteristic->subscribe(true, notificationHandler)) {
            Serial.println("Subscribe failed");
            client->disconnect();
            return;
        }
        Serial.println("Subscribed to notifications");
    } else {
        Serial.println("Characteristic does not support notifications");
        client->disconnect();
    }
}

void startScan() {
    NimBLEDevice::getScan()->start(SCAN_DURATION_MS, false, true);
}




#include <Arduino.h>
#include <NimBLEDevice.h>

// ==================== 配置设置 ====================
const char* const MAC_ADDRESS = "c6:86:a1:06:c3:88";
const char* const TARGET_SERVICE_UUID = "8650";
const char* const TARGET_CHARACTERISTIC_UUID = "8651";

constexpr uint32_t SCAN_DURATION_MS = 5000;

uint8_t Buffer[18] = {0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint16_t pressedKeys = 0;
uint16_t position = 0;
uint8_t s1 = 3, s2 = 2;

NimBLEAdvertisedDevice* advertisedDevice = nullptr;
bool shouldConnect = false;
bool isConnected = false;

//==================== 函数声明 ======================
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

//数据处理
void packData(uint16_t rightDrag, uint16_t rightPush,
                  uint16_t leftDrag, uint16_t leftPush,
                  uint8_t* buffer) {
    buffer[0] = (rightDrag & 0xFF);
    buffer[1] = (rightDrag >> 8);
    buffer[2] = (rightPush & 0xFF);
    buffer[3] = (rightPush >> 8);
    buffer[4] = (leftDrag & 0xFF);
    buffer[5] = (leftDrag >> 8);
    buffer[6] = (leftPush & 0xFF);
    buffer[7] = (leftPush >> 8);
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
    }else{
      
    }
    if (pressedKeys & (1 << 11)) { // c2 键被按下

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
    packData(rDragRaw, rPushRaw, lDragRaw, lPushRaw, Buffer);

}

void succeedConnect(){
  isConnected = true;
}
void failConnect(){
  isConnected = false;
}
void startScan() {
    NimBLEDevice::getScan()->start(SCAN_DURATION_MS, false, true);
}

void onTimerInterrupt(void *parameter) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while(1){
        if(isConnected){
            //Serial.write(Buffer, sizeof(Buffer));
            Serial1.write(Buffer, sizeof(Buffer));
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

void setup()
{
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, 4, 5);//通信串口初始化
    delay(1000);
    Serial.println("Bluetooth Remote Control");//硬件调试串口初始化

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

void loop()
{
    if (shouldConnect) {
        shouldConnect = false;
        connectToDevice();
    }
    delay(10);
}
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

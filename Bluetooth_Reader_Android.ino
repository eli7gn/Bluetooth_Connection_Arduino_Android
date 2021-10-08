
/* ===== pfod Command for GMM Reader ====
pfodApp msg {.} --> {,~<GMM reader>`0~V1|!A`3~Instrument ~`4`1~4~1~|!B<bg s>`3~Acompaniment ~`4`1~4~1~ |!C`1~Power ~~Off\On~}
 */
// Using Arduino Nano 33 BLE Board
// Use Arduino V1.8.9+ IDE


#include <ArduinoBLE.h>
#include <pfodParser.h>
// download the libraries from http://www.forward.com.au/pfod/pfodParserLibraries/index.html
// pfodParser.zip V3.48+ contains pfodParser, pfodSecurity, pfodDelay, pfodBLEBufferedSerial, pfodSMS and pfodRadio

#include <pfodBLEBufferedSerial.h>
int swap01(int); // method prototype for slider end swaps

// =========== pfodBLESerial definitions
const char* localName = "Nano 33 BLE";  // <<<<<<  change this string to customize the adverised name of your board (max 31 chars)
class pfodBLESerial : public BLELocalDevice, public Stream {
  public:
    pfodBLESerial(); bool begin(); void poll(); size_t write(uint8_t); size_t write(const uint8_t*, size_t); int read();
    int available(); void flush(); int peek(); void close(); bool isConnected();
  private:
    const static uint8_t pfodEOF[1]; const static char* pfodCloseConnection;  static const int BLE_MAX_LENGTH = 20;
    static const int BLE_RX_MAX_LENGTH = 256; static volatile size_t rxHead; static volatile size_t rxTail;
    volatile static uint8_t rxBuffer[BLE_RX_MAX_LENGTH];  volatile static bool connected;
    size_t txIdx;  uint8_t txBuffer[BLE_MAX_LENGTH]; static void connectHandler(BLEDevice central);
    static void disconnectHandler(BLEDevice central); static void receiveHandler(BLEDevice central, BLECharacteristic rxCharacteristic);
    static void addReceiveBytes(const uint8_t* bytes, size_t len); BLEService uartService = BLEService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    BLEDescriptor uartNameDescriptor = BLEDescriptor("2901", localName);
    BLECharacteristic rxCharacteristic = BLECharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite, BLE_MAX_LENGTH);
    BLEDescriptor rxNameDescriptor = BLEDescriptor("2901", "RX - (Write)");
    BLECharacteristic txCharacteristic = BLECharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLEIndicate | BLERead | BLENotify, BLE_MAX_LENGTH);
    BLEDescriptor txNameDescriptor = BLEDescriptor("2901", "TX - (Indicate|Notify)");
};
volatile size_t pfodBLESerial::rxHead = 0; volatile size_t pfodBLESerial::rxTail = 0;
volatile uint8_t pfodBLESerial::rxBuffer[BLE_RX_MAX_LENGTH]; const uint8_t pfodBLESerial::pfodEOF[1] = {(uint8_t) - 1};
const char* pfodBLESerial::pfodCloseConnection = "{!}"; volatile bool pfodBLESerial::connected = false;
// =========== end pfodBLESerial definitions

pfodParser parser("V2"); // create a parser to handle the pfod messages
pfodBLESerial bleSerial; // create a BLE serial connection
pfodBLEBufferedSerial bleBufferedSerial;

// give the board pins names, if you change the pin number here you will change the pin controlled
int cmd_A_var; // name the variable for 'Instrument'
pfodDelay cmd_A_adcTimer; // ADC timer
unsigned long cmd_A_ADC_READ_INTERVAL = 1000;// 1sec, edit this to change adc read interval
const int cmd_A_pin = A0; // name the pin for 'Instrument'
int cmd_B_var; // name the variable for 'Acompaniment'
pfodDelay cmd_B_adcTimer; // ADC timer
unsigned long cmd_B_ADC_READ_INTERVAL = 1000;// 1sec, edit this to change adc read interval
const int cmd_B_pin = A1; // name the pin for 'Acompaniment'
int cmd_C_var; // name the variable for 'Power'  0=Off 1=On 

unsigned long plot_mSOffset = 0; // set by {@} response
bool clearPlot = false; // set by the {@} response code

// the setup routine runs once on reset:
void setup() {
  cmd_A_var = 0;
  cmd_B_var = 0;

  // set advertised local name and service UUID
  // begin initialization
  if (!bleSerial.begin()) {
    //  Serial.println("starting ble failed!");
    while (1);
  }
  parser.connect(bleBufferedSerial.connect(&bleSerial));

  cmd_A_adcTimer.start(cmd_A_ADC_READ_INTERVAL); // start ADC timer
  cmd_B_adcTimer.start(cmd_B_ADC_READ_INTERVAL); // start ADC timer

  // <<<<<<<<< Your extra setup code goes here
}

// the loop routine runs over and over again forever:
void loop() {
  uint8_t cmd = parser.parse(); // parse incoming data from connection
  // parser returns non-zero when a pfod command is fully parsed
  if (cmd != 0) { // have parsed a complete msg { to }
    uint8_t* pfodFirstArg = parser.getFirstArg(); // may point to \0 if no arguments in this msg.
    pfod_MAYBE_UNUSED(pfodFirstArg); // may not be used, just suppress warning
    long pfodLongRtn; // used for parsing long return arguments, if any
    pfod_MAYBE_UNUSED(pfodLongRtn); // may not be used, just suppress warning
    if ('.' == cmd) {
      // pfodApp has connected and sent {.} , it is asking for the main menu
      if (!parser.isRefresh()) {
        sendMainMenu(); // send back the menu designed
      } else {
        sendMainMenuUpdate(); // menu is cached just send update
      }

      // handle {@} request
    } else if('@'==cmd) { // pfodApp requested 'current' time
      plot_mSOffset = millis(); // capture current millis as offset rawdata timestamps
      clearPlot = true; // clear plot on reconnect as have new plot_mSOffset
      parser.print(F("{@`0}")); // return `0 as 'current' raw data milliseconds
    

    // now handle commands returned from button/sliders
//    } else if('A'==cmd) { // this is a label. pfodApp NEVER sends this cmd -- 'Instrument'
//      // in the main Menu of GMM Reader 

//    } else if('B'==cmd) { // this is a label. pfodApp NEVER sends this cmd -- 'Acompaniment'
//      // in the main Menu of GMM Reader 

//    } else if('C'==cmd) { // this is a label. pfodApp NEVER sends this cmd -- 'Power'
//      // in the main Menu of GMM Reader 

    } else if ('!' == cmd) {
      // CloseConnection command
      closeConnection(parser.getPfodAppStream());
    } else {
      // unknown command
      parser.print(F("{}")); // always send back a pfod msg otherwise pfodApp will disconnect.
    }
  }
  cmd_A_readADC(); 
  cmd_B_readADC(); 
  //  <<<<<<<<<<<  Your other loop() code goes here 
  
}

void closeConnection(Stream *io) {
  // add any special code here to force connection to be dropped
  ((pfodBLESerial*)io)->close();
}
void cmd_A_readADC() {
  if (cmd_A_adcTimer.justFinished()) {
    cmd_A_adcTimer.repeat(); // restart timer, without drift
    cmd_A_var = analogRead(cmd_A_pin);  // read ADC input
  }
}

void cmd_B_readADC() {
  if (cmd_B_adcTimer.justFinished()) {
    cmd_B_adcTimer.repeat(); // restart timer, without drift
    cmd_B_var = analogRead(cmd_B_pin);  // read ADC input
  }
}


void sendMainMenu() {
  // !! Remember to change the parser version string
  //    every time you edit this method
  parser.print(F("{,"));  // start a Menu screen pfod message
  // send menu background, format, prompt, refresh and version
  parser.print(F("~<GMM reader>`0"));
  parser.sendVersion(); // send the menu version 
  // send menu items
  parser.print(F("|!A"));
  parser.print('`');
  parser.print(cmd_A_var); // output the current ADC reading
  parser.print(F("~Instrument ~`4`1~4~1~"));
  parser.print(F("|!B<bg s>"));
  parser.print('`');
  parser.print(cmd_B_var); // output the current ADC reading
  parser.print(F("~Acompaniment ~`4`1~4~1~ "));
  parser.print(F("|!C"));
  parser.print('`');
  parser.print(cmd_C_var); // output the current value 
  parser.print(F("~Power ~~Off\\On~"));
  // Note the \\ inside the "'s to send \ ... 
  parser.print(F("}"));  // close pfod message
}

void sendMainMenuUpdate() {
  parser.print(F("{;"));  // start an Update Menu pfod message
  // send menu items
  parser.print(F("|!A"));
  parser.print('`');
  parser.print(cmd_A_var); // output the current ADC reading
  parser.print(F("|!B"));
  parser.print('`');
  parser.print(cmd_B_var); // output the current ADC reading
  parser.print(F("|!C"));
  parser.print('`');
  parser.print(cmd_C_var); // output the current value 
  parser.print(F("}"));  // close pfod message
  // ============ end of menu ===========
}
// ========== pfodBLESerial methods
pfodBLESerial::pfodBLESerial() : BLELocalDevice() {
  setConnectionInterval(80,160); // 160 => 200mS max connection interval to match default pfodBLEBufferedSerial setting
};

bool pfodBLESerial::isConnected() {
  return (connected && txCharacteristic.subscribed());
}

bool pfodBLESerial::begin() {
  // begin initialization
  //  BLELocalDevice is BLE 
  if (!BLELocalDevice::begin()) { // returns 0 on failure
    return false;
  }

  // set the local name peripheral advertises
  setLocalName(localName);
  // set the UUID for the service this peripheral advertises
  setAdvertisedService(uartService);

  // add the characteristic to the service
  uartService.addCharacteristic(rxCharacteristic);
  uartService.addCharacteristic(txCharacteristic);
  rxCharacteristic.addDescriptor(rxNameDescriptor);
  txCharacteristic.addDescriptor(txNameDescriptor);

  // add service
  addService(uartService);

  // assign event handlers for connected, disconnected to peripheral
  setEventHandler(BLEConnected, connectHandler);
  setEventHandler(BLEDisconnected, disconnectHandler);

  // assign event handlers for characteristic
  rxCharacteristic.setEventHandler(BLEWritten, receiveHandler);

  // start advertising
  advertise();
  return true;
}

void pfodBLESerial::close() {
  BLELocalDevice::disconnect();
}

void pfodBLESerial::poll() {
  BLELocalDevice::poll();
}

size_t pfodBLESerial::write(const uint8_t* bytes, size_t len) {
  for (size_t i = 0; i < len; i++) {  write(bytes[i]);  }
  return len; // just assume it is all written
}

size_t pfodBLESerial::write(uint8_t b) {
  poll();
  if (!isConnected()) { return 1; }
  txBuffer[txIdx++] = b;
  if ((txIdx == sizeof(txBuffer)) || (b == ((uint8_t)'\n')) || (b == ((uint8_t)'}')) ) {
    flush(); // send this buffer if full or end of msg or rawdata newline
  }
  return 1;
}

int pfodBLESerial::read() {
  if (rxTail == rxHead) { return -1; }
  // note increment rxHead befor writing
  // so need to increment rxTail befor reading
  rxTail = (rxTail + 1) % sizeof(rxBuffer);
  uint8_t b = rxBuffer[rxTail];
  return b;
}

// called as part of parser.parse() so will poll() each loop()
int pfodBLESerial::available() {
  poll();
  flush(); // send any pending data now. This happens at the top of each loop()
  int rtn = ((rxHead + sizeof(rxBuffer)) - rxTail ) % sizeof(rxBuffer);
  return rtn;
}

void pfodBLESerial::flush() {
  if (txIdx == 0) { return; }
  txCharacteristic.setValue(txBuffer, txIdx);
  txIdx = 0;
  poll();
}

int pfodBLESerial::peek() {
  poll();
  if (rxTail == rxHead) { return -1; }
  size_t nextIdx = (rxTail + 1) % sizeof(rxBuffer);
  uint8_t byte = rxBuffer[nextIdx];
  return byte;
}

void pfodBLESerial::connectHandler(BLEDevice central) {
  (void)(central); // may not be used, just suppress warning
  connected = true;
}

void pfodBLESerial::disconnectHandler(BLEDevice central) {
  (void)(central); // may not be used, just suppress warning
  // parser.closeConnection();
  connected = false;
}

void pfodBLESerial::addReceiveBytes(const uint8_t* bytes, size_t len) {
  // note increment rxHead befor writing
  // so need to increment rxTail befor reading
  for (size_t i = 0; i < len; i++) {
    rxHead = (rxHead + 1) % sizeof(rxBuffer);
    rxBuffer[rxHead] = bytes[i];
  }
}

void pfodBLESerial::receiveHandler(BLEDevice central, BLECharacteristic rxCharacteristic) {
  (void)(central); // may not be used, just suppress warning
  size_t len = rxCharacteristic.valueLength();
  const unsigned char *data = rxCharacteristic.value();
  addReceiveBytes((const uint8_t*)data, len);
}
//======================= end pfodBLESerial methods


int swap01(int in) {
  return (in==0)?1:0;
}
// ============= end generated code =========
 

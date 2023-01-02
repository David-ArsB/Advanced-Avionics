#include <SPI.h>   // spi library for connecting nrf
#include <RF24.h>  // nrf library

#define   MAX_DATA_LEN    (32)
#define   TERMINATOR_CHAR ('\0')
char message2Transmit[32] = { 0 };
char ack_buf[] = {'a','c','k'};

RF24 radio(9, 10);  // ce, csn pins

void setup(void) {
  pinMode(6, OUTPUT);
  

  while (!Serial);

  Serial.begin(115200);  // start serial monitor baud rate

  //Serial.println("Starting.. Setting Up.. Radio on..");  // debug message

  radio.begin();  // start radio at ce csn pin 9 and 10

  radio.setPALevel(RF24_PA_MIN);  // set power level

  radio.setChannel(0x36);  // set channel at 36

  const uint64_t r_pipe = 0xE0E0E0E0E0LL;  // pipe address same as sender i.e. raspberry pi
  const uint64_t w_pipe = 0xF0F0F0F0F0LL;  // pipe address same as receiver i.e. raspberry pi

  radio.openReadingPipe(1, r_pipe);  // start reading pipe
  radio.openWritingPipe(w_pipe);

  radio.enableDynamicPayloads();
  radio.setPayloadSize(32);
  radio.setDataRate(RF24_250KBPS);
  radio.powerUp();

  radio.startListening();
  //radio.stopListening();  // stop listening 
  
}


void loop(void) {
  

  listenToPA();
  transmitToPA();

  delay(25);
}


void listenToPA(void){
  //radio.startListening();
  
  char receivedMessage[32] = { 0 };  // set incmng message for 32 bytes
  
  if (radio.available()) {   // check if message is coming
    radio.read(receivedMessage, sizeof(receivedMessage));

    if (strstr(receivedMessage, "#b") == NULL) {
      Serial.println(receivedMessage);
    }

    while (strstr(receivedMessage, "EOF") == NULL) {
      if (radio.available()) {

        radio.read(receivedMessage, sizeof(receivedMessage));  // read the message and save
        //radio.writeAckPayload(0, ack_buf, sizeof(ack_buf));

        Serial.println(receivedMessage);  // print message on serial monitor
        
      }
    }
    Serial.println(' ');              // print message on serial monitor
  }

  //radio.stopListening();  // stop listening 
}

void transmitToPA(void){
  //radio.stopListening();  // stop listening 
  char inByte;
  bool dataReady;
  
  
  while (Serial.available() > 0) {
    // read the incoming byte:
    digitalWrite(6, HIGH);
      delay(10);
      digitalWrite(6, LOW);
    inByte = Serial.read();
    dataReady = addData((char)inByte);  
    if ( dataReady )
      
      
      
      radio.stopListening();
      radio.write(&message2Transmit, sizeof(message2Transmit));
      radio.startListening();
      //Serial.println(message2Transmit);

  }
}

bool addData(char nextChar)
{  
  // This is position in the buffer where we put next char.
  // Static var will remember its value across function calls.
  static uint8_t currentIndex = 0;

    // Ignore some characters - new line, space and tabs
    if (nextChar == '\t' || nextChar == '\n'){
      return false;
    }
      

    // If we receive Enter character...
    if (nextChar == TERMINATOR_CHAR) {
        // ...terminate the string by NULL character "\0" and return true
        message2Transmit[currentIndex] = '\0';
        currentIndex = 0;
        return true;
    }

    // For normal character just store it in the buffer and move
    // position to next
    message2Transmit[currentIndex] = nextChar;
    currentIndex++;

    // Check for too many chars
    if (currentIndex >= MAX_DATA_LEN) {
      // The data too long so reset our position and return true
      // so that the data received so far can be processed - the caller should
      // see if it is valid command or not...
      message2Transmit[MAX_DATA_LEN] = TERMINATOR_CHAR;
      currentIndex = 0;
      return true;
    }

    return false;
}
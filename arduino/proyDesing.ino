#include <TinyWireM.h> 
#include <SoftwareSerial.h>
#include "ADS1X15.h"
#include "tinySPI.h"
#include "LoRaWAN.h"
#include "Arduino.h"

ADS1014 ADS(0x48);
#define DIO0 9
#define NSS  7
RFM95 rfm(DIO0, NSS);

LoRaWAN lora = LoRaWAN(rfm);
SoftwareSerial mySerial(8, 10);

byte ADDR_H = 0x40; //Sensor de Humedad
int counter = 38;
int Y0,Y1,Y2;
double Y,Y_out1,Y_out2;
float tempInt, tempExt, hum;
unsigned int Frame_Counter_Tx = 0x0000;

unsigned char NwkSkey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char AppSkey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char DevAddr[4]  = { 0x00, 0x00, 0x00, 0x00 };

void setUnusedPins();

void setup()
{
  rfm.init();
  lora.setKeys(NwkSkey, AppSkey, DevAddr);
  
  initADC();
  mySerial.begin(9600);

  // Humedad
  TinyWireM.begin();    //Initialize I2C
  delay(100);  
  TinyWireM.beginTransmission(ADDR_H);
  TinyWireM.endTransmission(); 
  delay(100);  

  // ADC Externo
  ADS.begin();
  ADS.setGain(0);      // 6.144 volt
  ADS.setDataRate(7);  // fast
  ADS.setMode(0);      // continuous mode
  ADS.readADC(0);      // first read to trigger
  
}

void loop()
{
  delay(500);
  counter++;
  
  if (counter >= 35){   // Cada 5 mins operar
    
    // Humedad
    TinyWireM.beginTransmission(ADDR_H);
    TinyWireM.write(0xE5);                    //Ask for Relatative humidity
    TinyWireM.endTransmission();
    TinyWireM.requestFrom(ADDR_H,5);

    if (TinyWireM.available() <= 5){
      TinyWireM.read();
      TinyWireM.read();
      TinyWireM.read();

      Y0 = TinyWireM.read();
      //mySerial.print(Y0);
      //mySerial.println(" ");
      Y2 = Y0/100; 
      Y0 = Y0%100;
      Y1 = TinyWireM.read();
      //mySerial.print(Y1);
      //mySerial.println(" ");
      Y_out1 = Y2*25600;
      Y_out2 = Y0*256 + Y1;
    
      Y_out1 = (125*Y_out1)/65536; 
      Y_out2 = (125*Y_out2)/65536;
    
      Y = Y_out1 + Y_out2;
      hum = Y - 6;

      //if (hum >= 0){
        //mySerial.print("Humedad: ");
        //mySerial.print(hum);
        //mySerial.println(" %");
      //} 
    }

    // ADC Externo
    float val_0 = ADS.readADC(0);  
    tempInt = 0.106437681 * val_0 - 20.21287539;
    //mySerial.print("Temperatura ADC Ext: "); mySerial.print(tempInt, 5); 
    //mySerial.write(0xF8);mySerial.println("C");

    // ADC Interno
    ADCSRA |= (1 << ADSC);         // start ADC measurement
    while (ADCSRA & (1 << ADSC) ); // wait till conversion complete 
    float t = (ADCH << 8) | ADCL;
    tempExt = 0.119224026 * t - 20.23844807;
    //mySerial.print("Temperatura ADC Int: "); mySerial.print(tempExt, 5);
    //mySerial.write(0xF8);mySerial.println("C");

    // LoRa
    hum = 10;
    tempInt = 30;
    tempExt = 31;

    byte *humB;
    byte *tempIntB;
    byte *tempExtB;
    
    uint8_t Data_Length = 0x06;
    uint8_t Data[Data_Length];

    humB     = (byte*) & hum;
    tempIntB = (byte*) & tempInt;
    tempExtB = (byte*) & tempExt;

    Data[0] = humB;
    Data[1] = tempIntB;
    Data[2] = tempExtB;

    //mySerial.print("Enviando datos... ");mySerial.println(" ");
    lora.Send_Data(Data, Data_Length, Frame_Counter_Tx);
    Frame_Counter_Tx++;

    counter = 0;  //reset counter
    
  }
}

void initADC()
{
   ADMUX =
            (0 << REFS1) |     // Sets ref. voltage to Vcc, bit 1   
            (0 << REFS0) |     // Sets ref. voltage to Vcc, bit 0
            (0 << MUX5)  |     // use ADC1 for input (PA1), MUX bit 5
            (0 << MUX4)  |     // use ADC1 for input (PA1), MUX bit 4
            (0 << MUX3)  |     // use ADC1 for input (PA1), MUX bit 3
            (0 << MUX2)  |     // use ADC1 for input (PA1), MUX bit 2
            (0 << MUX1)  |     // use ADC1 for input (PA1), MUX bit 1
            (1 << MUX0);       // use ADC1 for input (PA1), MUX bit 0

  ADCSRA = 
            (1 << ADEN)  |     // Enable ADC 
            (1 << ADPS2) |     // set prescaler to 128, bit 2 
            (1 << ADPS1) |     // set prescaler to 128, bit 1 
            (1 << ADPS0);      // set prescaler to 128, bit 0 
      
  ADCSRB = 
  //          (1 << ADLAR);      // left shift result (for 8-bit values)
          (0 << ADLAR);      // right shift result (for 10-bit values)
}

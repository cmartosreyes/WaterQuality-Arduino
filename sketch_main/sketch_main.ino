

/***************************************************************
 Connection:
 1.Plugable Terminal Sensor Adapter & Waterproof DS18B20 Digital Temperature Sensor
                 A   ---->     Blue(DATA SIGNAL)
                 B   ---->     RED   (VIN)
                 C   ---->     Black (GND)

 2.Waterproof DS18B20 Digital Temperature Sensor & Arduino board
              1(A)   ---->     Digital Pin2
              2(B)   ---->     5V/3.3V
              3(C)   ---->     GND

 Setting for the Pull-up Register/Pull-down Register Selection Jumpers
     When connect DS18B20 with the adapter,please choose to use the
     Pull-up Register Jumper
 ***************************************************************/

#include <OneWire.h>
#define SensorPin A2            //pH meter Analog output to Arduino Analog Input 2
#define Offset 0.00            //deviation compensate
#define LED 13
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex = 0;

int DS18S20_Pin = 2; //DS18S20 Signal pin on digital 2

//Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 2

char recibido = ' ';

void setup(void) {
  Serial1.begin(9600);
  Serial.begin(9600);
}

void loop(void) {
  float temperature = getTemp();
  float turbidity = getTurbidity();
  float pH = getPH();
  
  Serial.print("Temperature "); Serial.println(temperature);
  Serial.print("NTU "); Serial.println(turbidity); // print out the value you read:
 
  delay(2000);
  //ser = Serial.read();
  recibido = Serial1.read();
  if(recibido == 'a'){
    String tempS = String(temperature);
    String turbidityS = String(turbidity);
    String phS = String(pH);
    Serial1.print("#");
   // String tempS = String(temperature);
    Serial1.print(tempS);
    Serial1.print("+");
    Serial1.print(turbidityS);
    Serial1.print("+");
    Serial1.print(phS);
    Serial1.print("~");
    recibido = ' ';
  }
}

float getTurbidity(){

  int sensorValue = analogRead(A0);// read the input on analog pin 0:
  float voltage = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  
 // Serial.print("Voltaje "); Serial.println(voltage); // print out the value you read:
  float turbidity = 461.18*pow(voltage,2)-4737.3*voltage+11961;
  
  if(voltage>=4.47){turbidity=0;}
  if(voltage<3.74){turbidity=3000;}
  //if(turbidity<0){turbidity=0;};
  return turbidity;
}

float getPH(){

  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  float pHValue, voltage;
  if (millis() - samplingTime > samplingInterval)
  {
    pHArray[pHArrayIndex++] = analogRead(A2);
    if (pHArrayIndex == ArrayLenth)pHArrayIndex = 0;
    voltage = avergearray(pHArray, ArrayLenth) * 5.0 / 1024 - 0.10;
    pHValue = 3.5 * voltage + Offset;
    samplingTime = millis();
  }
  if (millis() - printTime > printInterval)  //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    //Serial.print("Voltage:");
    //Serial.print(voltage, 2);
    Serial.print("pH value: ");
    Serial.println(pHValue, 2);
    printTime = millis();
  }

  return pHValue;
  
}


float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius
  byte data[12];
  byte addr[8];
  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      Serial.println("no more sensors on chain, reset search!");
      ds.reset_search();
      return -1000;
  }
  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }
  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad

  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;

}

double avergearray(int* arr, int number) //Para el cálculo del pH, media del array de mediciones.
{
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if (number < 5) { //less than 5, calculated directly statistics
    for (i = 0; i < number; i++){
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  }else{
    if (arr[0] < arr[1]){
      min = arr[0]; max = arr[1];
    }else{
      min = arr[1]; max = arr[0];
    }
    for (i = 2; i < number; i++){
      if (arr[i] < min){
        amount += min;      //arr<min
        min = arr[i];
      }else{
        if (arr[i] > max){
          amount += max;  //arr>max
          max = arr[i];
        }else{
          amount += arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount / (number - 2);
  }//if
  return avg;
}

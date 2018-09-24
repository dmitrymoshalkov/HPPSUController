
#include <Arduino.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Bounce2.h>
#include <avr/wdt.h>

#define MY_RADIO_RF24
#define MY_RF24_CHANNEL	(81)
#define MY_NODE_ID 30
#define MY_DEBUG // Enables debug messages in the serial log
#define NDEBUG
#define MY_BAUD_RATE 115200
//#define MY_OTA_FIRMWARE_FEATURE

  //#define MY_RF24_CE_PIN 4

#define MY_TRANSPORT_WAIT_READY_MS 10000


#include <MySensors.h>



 bool powerON(boolean bState);
 bool getPSUOutState();
 void reportPowerState();
 void checkTemp();
 unsigned long calculateRPM();
 void reportAlert (uint8_t errID);
 void reportRPM();

#define SKETCH_NAME "HP PSU Controller"
#define SKETCH_MAJOR_VER "1"
#define SKETCH_MINOR_VER "0"


//#define FANPOWER_PIN 5
#define FANSPEED_PIN 3
#define PSUPOWER_PIN A1
#define TEMP_PIN 	4
#define BUTTON_PIN 	A2
#define DCOK_PIN 	A4


#define PSUPOWER_ID 1
#define DCOK_ID	2
#define FANSPEED_ID	10
#define TEMP1_ID	11
#define TEMP2_ID	12
#define REBOOT_CHILD_ID    100
#define ALERT_ID  150

#define RADIO_RESET_DELAY_TIME 25 //Задержка между сообщениями
#define MESSAGE_ACK_RETRY_COUNT 5  //количество попыток отсылки сообщения с запросом подтверждения
#define DATASEND_DELAY  10
#define GWSTATUSCHECK_TIME 150000


#define TEMP1CYCLECOUNT 180
#define TEMP2CYCLECOUNT 180
#define FANCYCLECOUNT 180
#define TEMP1WARNWAL 70
#define TEMP1CRITVAL 80
#define TEMP2WARNWAL 70
#define TEMP2CRITVAL 80
#define ZERORPMFAIL  20

boolean gotAck=false; //подтверждение от гейта о получении сообщения

int iCount = MESSAGE_ACK_RETRY_COUNT;

Bounce debouncer = Bounce();

byte oldButtonVal=5;

float lastTemp1Value=0;
float lastTemp2Value=0;
long lastFanSpeedValue=0;
long previousTempMillis = 0;        // last time the sensors are updated
long TempsensorInterval = 1000;     // interval at which we will take a measurement ( 30 seconds)
uint16_t temp1CycleCount = 0;
uint16_t temp2CycleCount = 0;
uint16_t FanCycleCount = 0;

long previousPSMillis = 0;        // last time the sensors are updated
long PSInterval = 2000;     // interval at which we will take a measurement ( 30 seconds)

boolean isOn = false;

boolean isTempReceived = false;  // set to True after reading temp, for correct RPM reading

unsigned long previousRPMMillis;
float RPM;

volatile unsigned long pulses=0;
unsigned long lastRPMmillis = 0;
unsigned long interval = 62.53;
uint8_t rpmZeroState = 0;

//if true = all OK
boolean platformErrorState = true;
boolean prevPlatformErrorState = true;

boolean platformTemp1Error = false;
boolean platformTemp2Error = false;
boolean platformFanError = false;
boolean platformPSError = false;

OneWire oneWire(TEMP_PIN); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature.


MyMessage msgPSUPowerStatus(PSUPOWER_ID, V_STATUS);
//MyMessage msgDCOutState(DCOK_ID, V_STATUS);
MyMessage msgFanSpeed(FANSPEED_ID, V_HVAC_SPEED);
MyMessage msgTemp1(TEMP1_ID, V_TEMP);
MyMessage msgTemp2(TEMP2_ID, V_TEMP);
MyMessage msgAlertState(ALERT_ID, V_VAR1);

/*
Alert description

0 - All OK
1 - Temp warning
2 - Temp critical
3 - FAN OFF
4 - Cant switch power
5 - Unknown error
*/

#define NOERROR         0
#define OVERHEATWARN    1
#define OVERHEATCRIT    2
#define FANMAILFUNCTION 3
#define POWERSWITCHERR  4
#define UNKNOWNERROR    5


void countPulse() {
  // just count each pulse we see
  // ISRs should be short, not like
  // these comments, which are long.
  pulses++;
}




void setup() {

  #ifdef NDEBUG
  Serial.begin(MY_BAUD_RATE);
  #endif

  TCCR0B = TCCR0B & B11111000 | B00000101;

  #ifdef NDEBUG
  Serial.print("Power state = "); //вывод сообщения о нажатии
  Serial.println(isOn); //вывод сообщения о нажатии
  #endif


}



void before()
{

  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH);
  debouncer.attach(BUTTON_PIN);
  debouncer.interval(5);
  oldButtonVal = debouncer.read();

	pinMode(FANSPEED_PIN, INPUT_PULLUP);
	digitalWrite(FANSPEED_PIN, HIGH);

  pinMode(DCOK_PIN, INPUT);


  pinMode(PSUPOWER_PIN, OUTPUT);
  if (!powerON(true))
  {
    #ifdef NDEBUG
    Serial.println("Error powering ON.");
    #endif

    //Alert - Error
  }

  #ifdef NDEBUG
  Serial.print("isOn = "); //вывод сообщения о нажатии
  Serial.println(isOn); //вывод сообщения о нажатии
  #endif


  pinMode(FANSPEED_PIN, INPUT_PULLUP);

  digitalWrite(FANSPEED_PIN, HIGH);

  //attachInterrupt(1, rpm, FALLING); //RISING
  attachInterrupt(digitalPinToInterrupt(FANSPEED_PIN), countPulse, RISING);

//reread temp sensors
          sensors.requestTemperatures();
    float temperature = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(0) * 10.)) / 10.;
          temperature = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(1) * 10.)) / 10.;

}



void presentation()
{






					    iCount = MESSAGE_ACK_RETRY_COUNT;

	                    while( !sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER"."SKETCH_MINOR_VER) && iCount > 0 )
	                      {
	                         wait(RADIO_RESET_DELAY_TIME);
	                        iCount--;
	                       }

						wait(RADIO_RESET_DELAY_TIME);




					    iCount = MESSAGE_ACK_RETRY_COUNT;

	                    while( !present(PSUPOWER_ID, S_BINARY, "PSUPowerp") && iCount > 0 )
	                      {
	                         wait(RADIO_RESET_DELAY_TIME);
	                        iCount--;
	                       }

						wait(RADIO_RESET_DELAY_TIME);


					    iCount = MESSAGE_ACK_RETRY_COUNT;

	                    while( !present(REBOOT_CHILD_ID, S_BINARY, "Reboot sensor") && iCount > 0 )
	                      {
	                         wait(RADIO_RESET_DELAY_TIME);
	                        iCount--;
	                       }

						wait(RADIO_RESET_DELAY_TIME);


    					request(PSUPOWER_ID, V_STATUS);
  						wait(1000);


}






void receive(const MyMessage &message)
{
 // Handle incoming message

  //if (message.isAck())
  //{
  //  gotAck = true;
  //  return;
  //}

    if ( message.sensor == REBOOT_CHILD_ID && message.getBool() == true && strlen(message.getString())>0 ) {
    	   	  #ifdef NDEBUG
      			Serial.println("Received reboot message");
   	  			#endif
             //wdt_enable(WDTO_30MS);
              while(1) {

              };

     return;

     }


   if ( message.sensor == PSUPOWER_ID && strlen(message.getString())>0 )
   {


  	 	if (message.type == V_STATUS)
  	 	{
        switch (message.getBool())
        {
          case true:
            if ( !isOn )
            {
                if (!powerON(true) )
                {
                  reportAlert(POWERSWITCHERR);
                  platformPSError = true;
                }
                else
                {
                  platformPSError = false;
                  reportAlert(NOERROR);

                }

            }
          break;

          case false:
          if ( isOn )
          {
              if (!powerON(false) )
              {
                reportAlert(POWERSWITCHERR);
                platformPSError = true;
              }
              {
                platformPSError = false;
                reportAlert(NOERROR);

              }

          }
          break;
        }

		   }


       reportPowerState();
   }


 }






 void loop()
 {

   if (debouncer.update())
    { //если произошло событие
      byte currButtonVal = debouncer.read();
      #ifdef NDEBUG
      Serial.print("currButtonVal = "); //вывод сообщения о нажатии
      Serial.println(currButtonVal); //вывод сообщения о нажатии
      Serial.print("oldButtonVal = "); //вывод сообщения о нажатии
      Serial.println(oldButtonVal); //вывод сообщения о нажатии
      Serial.print("isOn = "); //вывод сообщения о нажатии
      Serial.println(isOn); //вывод сообщения о нажатии
      #endif

     if (currButtonVal==1 && oldButtonVal != currButtonVal )
     { //если кнопка нажата
       switch (isOn)
       {
        case true:
            if (!powerON(false) )
            {
              reportAlert(POWERSWITCHERR);
              platformPSError = true;
            }
            else
            {
              platformPSError = false;
              reportAlert(NOERROR);
            }
        break;

        case false:
          if (!powerON(true) )
          {
            reportAlert(POWERSWITCHERR);
            platformPSError = true;
          }
          else
          {
            platformPSError = false;
            reportAlert(NOERROR);
          }
        break;
      }



      #ifdef NDEBUG
      Serial.println("pressed"); //вывод сообщения о нажатии
      #endif

      oldButtonVal = 1;

     }
     else if (currButtonVal == 0 && currButtonVal != oldButtonVal)
     {
       oldButtonVal = 0;
       #ifdef NDEBUG
       Serial.println("released"); //вывод сообщения об отпускании
       #endif
     }
    }


 checkTemp();

 reportPowerState();

reportRPM();

}


void reportRPM()
{
  if (millis() - previousRPMMillis > interval)
  {

       unsigned long currRPM = calculateRPM();

       #ifdef NDEBUG
         Serial.print("RPM=");
         Serial.println(currRPM);
       #endif

       previousRPMMillis = millis();


       if (FanCycleCount == FANCYCLECOUNT)
       {
         FanCycleCount = 0;

         iCount = MESSAGE_ACK_RETRY_COUNT;

         while( !send(msgFanSpeed.set(currRPM,0)) && iCount > 0 )
         {

          wait(RADIO_RESET_DELAY_TIME);
          iCount--;
         }
       }
       else
       {
         FanCycleCount++;
       }


       if ( currRPM < 10 && isOn )
       {
           if ( rpmZeroState >= ZERORPMFAIL)
           {
             //power off

             if (!powerON(false) )
             {
               reportAlert(POWERSWITCHERR);
               platformPSError = true;
             }
             else
             {
               platformPSError = false;
               reportAlert(NOERROR);
             }

             rpmZeroState = 0;

             reportAlert(FANMAILFUNCTION);
             platformFanError = true;
           }
           else
           {
             rpmZeroState++;
           }

       }
       else if ( currRPM >= 10 && isOn && lastFanSpeedValue < 10 )
       {
         rpmZeroState=0;

         if (platformFanError)
         {
           platformFanError = false;
           reportAlert(NOERROR);
         }

       }
 lastFanSpeedValue = currRPM;

  }
}

 bool powerON(boolean bState)
 {

   #ifdef NDEBUG
   Serial.print("Powering = "); //вывод сообщения о нажатии
   Serial.println(bState); //вывод сообщения о нажатии
   #endif

   switch (bState)
   {
     case true:
       digitalWrite(PSUPOWER_PIN, HIGH);
       wait(100);
       if ( getPSUOutState() )
       {
         isOn = true;
         return true;
       }
       else
       {
         isOn = false;
         return false;
       }
     break;
     case false:
       digitalWrite(PSUPOWER_PIN, LOW);
       wait(100);
       if ( !getPSUOutState() )
       {
         isOn = false;
         return true;
       }
       else
       {
         isOn = true;
         return false;
       }
     break;
   }

 }


 bool getPSUOutState()
 {

   int relayState = digitalRead(DCOK_PIN);

   #ifdef NDEBUG
     Serial.print("PSU=");
     Serial.println(relayState);
   #endif

   if ( relayState == HIGH)
  {
    return false;
  }
  else if ( relayState == LOW)
  {
    return true;
  }

 }


void reportPowerState()
{

  unsigned long currentPSMillis = millis();
  if((currentPSMillis - previousPSMillis > PSInterval) )
  {
      // Save the current millis
      previousPSMillis = currentPSMillis;

        iCount = MESSAGE_ACK_RETRY_COUNT;

          while( !send(msgPSUPowerStatus.set(isOn,0)) && iCount > 0 )
          {

           wait(RADIO_RESET_DELAY_TIME);
           iCount--;
          }
 }

}




 void checkTemp()
 {

     unsigned long currentTempMillis = millis();
     if((currentTempMillis - previousTempMillis > TempsensorInterval) ) {
         // Save the current millis
         previousTempMillis = currentTempMillis;
         // take action here:

           // Fetch temperatures from Dallas sensors
           sensors.requestTemperatures();

         float Temp1 = 0, Temp2 = 0;

         Temp1 = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(0) * 10.)) / 10.; // Temp sensor relay

         if (Temp1 != -127)
         {

           if (abs(Temp1 - lastTemp1Value) >= 1 || temp1CycleCount == TEMP1CYCLECOUNT)
           {
              iCount = MESSAGE_ACK_RETRY_COUNT;

     	        while( !send(msgTemp1.set(Temp1,1)) && iCount > 0 )
     	        {

     	         wait(RADIO_RESET_DELAY_TIME);
     	         iCount--;
     	        }


             if (temp1CycleCount == TEMP1CYCLECOUNT)
             {
               temp1CycleCount = 0;

               iCount = MESSAGE_ACK_RETRY_COUNT;

               while( !send(msgTemp1.set(Temp1,1)) && iCount > 0 )
               {

                wait(RADIO_RESET_DELAY_TIME);
                iCount--;
               }
             }
             else
             {
               temp1CycleCount++;
             }

           }

           #ifdef NDEBUG
           Serial.print("Temp1: ");
           Serial.println(Temp1);
           #endif

           if ( Temp1 <= TEMP1WARNWAL && lastTemp1Value > TEMP1WARNWAL )
           {
             platformTemp1Error = false;
             reportAlert(NOERROR);
           }

           if ( Temp1 > TEMP1WARNWAL && Temp1 < TEMP1CRITVAL && lastTemp1Value <= TEMP1WARNWAL )
           {
             platformTemp1Error = true;
             reportAlert(OVERHEATWARN);
           }

           if ( Temp1 >= TEMP1CRITVAL && lastTemp1Value < TEMP1CRITVAL )
           {

             platformTemp1Error = true;
             reportAlert(OVERHEATCRIT);

             if ( isOn )
             {
               if (!powerON(false) )
               {
                 reportAlert(POWERSWITCHERR);
                 platformPSError = true;
               }
               else
               {
                 platformPSError = false;
                 reportAlert(NOERROR);
               }

             }
           }

                        lastTemp1Value = Temp1;

         }


         Temp2 = static_cast<float>(static_cast<int> (sensors.getTempCByIndex(1) * 10.)) / 10.; // Temp sensor on top

         if (Temp2 != -127)
         {
           if (abs(Temp2 - lastTemp2Value) >= 1  || temp2CycleCount == TEMP2CYCLECOUNT)
           {
              iCount = MESSAGE_ACK_RETRY_COUNT;

     	        while( !send(msgTemp2.set(Temp2,1)) && iCount > 0 )
     	        {

     	         wait(RADIO_RESET_DELAY_TIME);
     	         iCount--;
     	        }



             if (temp2CycleCount >= TEMP2CYCLECOUNT)
             {
               temp2CycleCount = 0;

               iCount = MESSAGE_ACK_RETRY_COUNT;

               while( !send(msgTemp2.set(Temp1,2)) && iCount > 0 )
               {

                wait(RADIO_RESET_DELAY_TIME);
                iCount--;
               }

             }
             else
             {
               temp2CycleCount++;
             }

           }

           #ifdef NDEBUG
           Serial.print("Temp2: ");
           Serial.println(Temp2);
           #endif

           if ( Temp2 <= TEMP2WARNWAL && lastTemp2Value > TEMP2WARNWAL )
           {
             platformTemp2Error = false;
             reportAlert(NOERROR);
           }

           if ( Temp2 > TEMP2WARNWAL && Temp2 < TEMP2CRITVAL && lastTemp2Value <= TEMP2WARNWAL )
           {
             platformTemp2Error = true;
             reportAlert(OVERHEATWARN);
           }

           if ( Temp2 >= TEMP2CRITVAL && lastTemp2Value < TEMP2CRITVAL )
           {

             platformTemp2Error = true;
             reportAlert(OVERHEATCRIT);

             if ( isOn )
             {
               if (!powerON(false) )
               {
                 reportAlert(POWERSWITCHERR);
                 platformPSError = true;
               }
               else
               {
                 platformPSError = false;
                 reportAlert(NOERROR);
               }

             }
           }

             lastTemp2Value = Temp2;

         }




         if ( Temp2 > TEMP2WARNWAL && Temp2 < TEMP2CRITVAL )
         {
           //Alert
         }



         if ( Temp2 >= TEMP2CRITVAL )
         {
           //Alert

           if ( isOn )
           {
               if (!powerON(false) )
               {
                 //Alert - Error
               }

           }
         }

         #ifdef NDEBUG
         Serial.print("Power state = "); //вывод сообщения о нажатии
         Serial.println(isOn); //вывод сообщения о нажатии
         #endif


     }


 }



 unsigned long calculateRPM() {
   unsigned long RPM;
   noInterrupts();
   float elapsedMS = (millis() - lastRPMmillis)/62.53;
   unsigned long revolutions = pulses/2;
   float revPerMS = revolutions / elapsedMS;
   RPM = revPerMS * 60.0;
   lastRPMmillis = millis();
   pulses=0;
   interrupts();

   return RPM;
 }


void reportAlert (uint8_t errID)
{

  iCount = MESSAGE_ACK_RETRY_COUNT;

  while( !send(msgAlertState.set(errID,0)) && iCount > 0 )
  {

   wait(RADIO_RESET_DELAY_TIME);
   iCount--;
  }

    prevPlatformErrorState = platformErrorState;

  if ( errID != NOERROR )
  {
    prevPlatformErrorState = false;
  }
  else
  {
    if ( platformTemp1Error == false && platformTemp2Error == false && platformFanError == false && platformPSError == false)
    {
      prevPlatformErrorState = true;
    }
  }

}

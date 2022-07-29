//designed for RFM23BP radio 

#include <SPI.h>   //just removed this (not sure the results) 
#include <RH_RF22.h>
//#include <RHDatagram.h>
#include <RHHardwareSPI1.h>   

//TEST CODE (RBF) 
int led = 13;
//END 


// Singleton instance of the radio driver
const int CS_PIN     = 38;    //38
const int INT_PIN    = 8;     //8
const int RST_PIN    = 7;     //7

const int SPI_MISO   = 39;    //39
const int SPI_MOSI   = 26;    //26
const int SPI_SCK    = 27;    //27


RH_RF22 rf22( CS_PIN, INT_PIN, hardware_spi1 ) ; 
//   RHDatagram manager ( rf22 ) ; //THIS IS THE NEWEST THREAD OF PROGRESS 

void setup() 
{
  //TEST CODE (RBF) 
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  //END 

  
  Serial.begin(9600);
  Serial.println ( "serial working" ); 
  digitalWrite(led, LOW);

  //assign our SPI pins, not SPI1 because primary SPI is used for something else
  SPI1.setMISO(SPI_MISO);
  SPI1.setMOSI(SPI_MOSI);
  SPI1.setSCK(SPI_SCK);
  //SPI1.setCS(CS_PIN);


   
  Serial.println ( "version #8" ); 
  Serial.println ( "this version worked!" ); 
  
  //make the reset pin an output 
  //turn on the device (low == on)
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, LOW);
 
 
  // manual reset
  Serial.println ( "manual reset starting" ); 
  delay(1000);
  digitalWrite(RST_PIN, HIGH);
  Serial.println ( "manual reset halfway" );
  
  delay(1000);
  digitalWrite(RST_PIN, LOW); //print statements stop working here
  Serial.println ( "manual reset complete" );
  digitalWrite(led, HIGH);
  delay(1000);
  //end manual reset 
 
  
 
  

  int x = 0;

  //while loop to give multiple init 
  while( x < 10 ){
    if ( !rf22.init() ){  // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
     
      Serial.print("driver init failed: ");
      Serial.println( x ); 
      digitalWrite(led, LOW);
      delay(1000);

         
        delay(500);
        digitalWrite(RST_PIN, HIGH);      
        delay(500);
        digitalWrite(RST_PIN, LOW); //print statements stop working here 


        //want to add something to reset the transaction of the teensy side 
        SPI1.end(); //this created some weird bevahior where the code gets stuck in random places 
                    //it does however reset the SPI on the teensy side of things
        delay(500);
       
  
      x += 1;
    }
    else
    {
      Serial.println("init successful");
      digitalWrite(led, HIGH);
      break;
     }
  }
  if (x > 8){
    Serial.println( "initialization failed, exiting" ); 
    exit (0);
  }
  
 /* if(!manager.init()){
    Serial.println("manager init failed");
    }*/
    

  Serial.println( "setting frequency, power and modulation " ); 
  rf22.setFrequency( 433 );  //frequency default is 434MHz 
  rf22.setTxPower( 11 );  //20 is the max
 
//new code start 

  //must put to sleep before switching modulation scheme
  if( !rf22.sleep() ){
    Serial.println( "failed to sleep" ); 
  }
  else {
    Serial.println( "succeeded to sleep"); 
  }
  int GFSK = 3; 
  if ( !rf22.setModemConfig(RH_RF22::GFSK_Rb2Fd5)){
    Serial.println( "modulation setting unsuccessful ");
  }
  else { 
    Serial.println( "modulation set to GFSK" ); 
  }

//new code end 
  
  Serial.println("made it to the loop"); 

}
void loop()
{
  
  Serial.println("Sending to rf22_server");
  // Send a message to rf22_server
  uint8_t data[] = "hello_world123456789123456789123456789123456789"; //this seems to be the max message size "hello_world123456789123456789123456789123456789"
            
  rf22.send(data, sizeof(data));
  //rf22.waitPacketSent(); //theory on why the serial is messed up 

  delay(1500);             //(300 GOOD!) delay worked as low as 200 but there may have been distortion in the signal (needs checking) 
 
  if(!rf22.sleep()){      //must sleep between each sent message!!!!
    
    Serial.println("failed to sleep");
    }
    else{
      Serial.println("sleep success");
    }
  Serial.println( "bottom of the loop");
  //delay(100); 
  
}
//trouble shooting:
//1 grounding error, make sure the teensy is properly grounded 
//2 power cycle from by unplugging the laptop 
//3 seems as though if it runs even once without proper grounding it needs to be power cycled or it will not work

//4 MOSI (26) should be high normally and dip to low briefly in sqaure waves 
//  The sqaure wave dips are the signal
//5 clock (27) seems (check this) low and goes high to transmit signal 
//  clock seems like the current issue, were only picking up one signal at a time 

//4 & 5 now seem irrelevant 
//6 either the end spi line or the longer transmitted message have slowed down the initialization process
//6 is no longer true, not sure why the process was slowed but now its fast again without any changes 

// things to do 
// DONE     MOSI never goes low when the radio resets, the only way for MOSI to go low is too power cycle the teeensy 
//          I want to write into the code a way to make the MOSI pin go low again every time we try to reset the 
//          radio for a start attempt. I think because the MOSI pin stays high while we use the reset pin to
//          reset the radio, the radio only gets ine legitamte attempt to initiallize. My guess is that when the radio 
//          is reset it takes the current value of the MOSI pin as low. This could help defeat the observed 
//          behvaior or needing a manual reset if the rf23 does initialize on the first try. 

// we think grounding may be the current issue. We're also not seeing what we expect to see with the clock 
// were only seeing one blip of the clock pin going from low (its normal) to high 

#include <SoftwareSerial.h>
#include "Wire.h"

#include <si5351.h>
Si5351 si5351;

#include <JTEncode.h>
//#include <rs_common.h>
//#include <int.h>
#include <string.h>
//#define WSPR_DEFAULT_FREQ       14097200UL
JTEncode jtencode;

#include <NMEAGPS.h>
//using namespace NeoGPS;
NMEAGPS gps;
gps_fix fix;

//#include <U8g2lib.h>
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

boolean tx_even = true, 
//        tx_enabled = true,
        tx_enabled = false,
        txing = false,
        toggle_gps_fix = false;
char  strCallsign[7] = "DL4OCE",
      strLocator[5] = "XX00",
      strFullLocator[10];

/*
 * DL4OCE JO52 11
 * uint8_t tx_buffer[255] = {
  1, 1, 0, 0, 0, 0, 2, 2, 1, 2, 2, 2, 3, 3, 1, 0, 2, 0, 3, 2, 0, 1, 2, 3, 1, 1, 1, 2, 2, 2,
  0, 2, 2, 2, 1, 2, 0, 3, 0, 3, 2, 2, 2, 2, 2, 2, 1, 0, 1, 3, 2, 0, 1, 3, 0, 3, 0, 2, 2, 1,
  3, 0, 1, 0, 2, 0, 2, 1, 3, 2, 3, 0, 1, 2, 1, 0, 1, 0, 2, 1, 2, 0, 3, 0, 3, 1, 2, 0, 2, 3,
  1, 2, 3, 2, 3, 2, 2, 2, 1, 2, 2, 2, 0, 0, 3, 2, 2, 1, 0, 0, 1, 3, 3, 0, 1, 1, 2, 0, 3, 3,
  0, 1, 0, 0, 2, 3, 1, 1, 0, 2, 2, 0, 0, 3, 2, 1, 2, 2, 1, 1, 0, 2, 2, 2, 0, 0, 2, 3, 3, 0,
  1, 0, 3, 3, 2, 2, 0, 3, 1, 2, 0, 2
  };
*/
      
//uint8_t tx_buffer[255];
uint8_t tx_buffer[WSPR_SYMBOL_COUNT]; // 162
//uint16_t tone_delay, tone_spacing;
uint8_t pwr_output = 11; // Si5351 will put ~ 11 dBm out
long  intBaseFrequency = 7040000,
      intTxFrequency = intBaseFrequency;

/*
 * Band Dial freq (MHz) Tx freq (MHz)
160m 1.836600 1.838000 - 1.838200
80m 3.592600 3.594000 - 3.594200
60m 5.287200 5.288600 - 5.288800
40m 7.038600 7.040000 - 7.040200
30m 10.138700 10.140100 - 10.140300
20m 14.095600 14.097000 - 14.097200
17m 18.104600 18.106000 - 18.106200
15m 21.094600 21.096000 - 21.096200
12m 24.924600 24.926000 - 24.926200
10m 28.124600 28.126000 - 28.126200
6m 50.293000 50.294400 - 50.294600
2m 144.488500 144.489900 - 144.490100
 */

SoftwareSerial gpsSerial(3, 4);

boolean debugging_enabled = true;

void debug(String message){
  if (debugging_enabled) Serial.print(message);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  //Begin serial communication with Arduino and Arduino IDE (Serial Monitor)
  Serial.begin(115200);
  while(!Serial);
   
  // Begin serial communication with Arduino and GPS module. Rate must be 9600
  // D3=TX-Arduino to RX-GPS, D4=RX-Arduino to TX-GPS
  gpsSerial.begin(9600);

  //Serial.println("WSPR beacon tbased on u-blox NEO-M8N-0-10");
  debug("WSPR beacon based on u-blox NEO-M8N-0-10\n");

  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other than 25 MHz
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  //symbol_count = WSPR_SYMBOL_COUNT; // From the library defines (162)
  //tone_spacing = 146;
  //tone_delay = 683;
  // Set CLK0 output
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power if desired
  si5351.output_enable(SI5351_CLK0, 0); // Disable the clock initially

  // set up the LCD's number of columns and rows: 
  //  lcd.begin(16, 2);
  //delay (200); // wait to allow LCD display to init

//  u8g2.begin();

}

void calcLocator(char *dst, double lat, double lon) {
  int o1, o2, o3;
  int a1, a2, a3;
  double remainder;
  remainder = lon + 180.0;
  o1 = (int)(remainder / 20.0);
  remainder = remainder - (double)o1 * 20.0;
  o2 = (int)(remainder / 2.0);
  remainder = remainder - 2.0 * (double)o2;
  o3 = (int)(12.0 * remainder);
  remainder = lat + 90.0;
  a1 = (int)(remainder / 10.0);
  remainder = remainder - (double)a1 * 10.0;
  a2 = (int)(remainder);
  remainder = remainder - (double)a2;
  a3 = (int)(24.0 * remainder);
  dst[0] = (char)o1 + 'A';
  dst[1] = (char)a1 + 'A';
  dst[2] = (char)o2 + '0';
  dst[3] = (char)a2 + '0';
  dst[4] = (char)o3 + 'a';
  dst[5] = (char)a3 + 'a';
  dst[6] = (char)0;
}

void updateLCD(){
  // update LC display with call, maidenhead locator, time hh:mm:ss, [g|G][ |t|T] (GPS fix/no fix, TX enabled/disabled/active), cursor showing progress
  //DL4OCE JO52ff GT
  //12:34:56
  String tmpStr = "";
  debug("[updateLCD]\n");
  //String tmpLoc = "";
  tmpStr += (String)strCallsign + " " + (String)strLocator + " "; 
  if (fix.valid.location) tmpStr += "G";
  else tmpStr += "g";
  if (tx_enabled && !txing) tmpStr += "t";
  else if (tx_enabled & txing) tmpStr += "T";
  else tmpStr += "_";
  debug(tmpStr+"\n");
  tmpStr = "";
  if (fix.dateTime.hours < 10) tmpStr += "0";
  tmpStr += (String)fix.dateTime.hours + ":";
  if (fix.dateTime.minutes < 10) tmpStr += "0";
  tmpStr += (String)fix.dateTime.minutes + ":";
  if (fix.dateTime.seconds < 10) tmpStr += "0";
  tmpStr += (String)fix.dateTime.seconds + " ";
  tmpStr += (String) pwr_output + " dBm\n";
  debug(tmpStr+"\n");

/*  
 *   TIDY UP FIRST!!
 *   program uses 110 % of program memory :-(
 *   u8g2.clearBuffer();  
  u8g2.setFontMode(1);
//  u8g2.setFont(u8g2_font_cu12_tr);      
  u8g2.setFont(u8g2_font_crox3c_mf);   
  u8g2.setCursor(0,15);
  u8g2.print(tmpStr);
  u8g2.sendBuffer();

  */
}

void do_WSPR(){
  uint8_t i;
  if (tx_enabled){
    debug("Starting transmission on Base=" + (String)intBaseFrequency + ", TxFreq=" + (String)intTxFrequency);
    // Reset the tone to the base frequency and turn on the output
    si5351.output_enable(SI5351_CLK0, 1);
    digitalWrite(LED_BUILTIN, HIGH);
    for(i = 0; i < WSPR_SYMBOL_COUNT; i++){
      si5351.set_freq((intTxFrequency * 100) + (tx_buffer[i] * 146), SI5351_CLK0);
      delay(683);
    }
    // Turn off the output
    si5351.output_enable(SI5351_CLK0, 0);
    digitalWrite(LED_BUILTIN, LOW);
    debug("Stopping transmission...");
  }
}

void set_tx_buffer(){
//  memset(tx_buffer, 0, 255);  
  memset(tx_buffer, 0, WSPR_SYMBOL_COUNT);  
/*  debug((String)sizeof(strCallsign) + ", ");
  debug((String)sizeof(strLocator) + ", ");
  debug((String)pwr_output + ", ");
  debug((String)sizeof(tx_buffer) + "\n");*/
  //jtencode.wspr_encode(strCallsign, strLocator, pwr_output, tx_buffer);
  //jtencode.wspr_encode("DL4OCE", "JO52", 11, tx_buffer);
}

void generateWSPRbuffer(float tmpLat, float tmpLon){
//  String tmpStr="";
  debug("\n[generateWSPRbuffer]\n");
 /* debug((String)tmpLat);
  debug(" ");
  debug((String)tmpLon);
  debug(" ");
*/  
  calcLocator(strFullLocator, tmpLat, tmpLon);
  strncpy(strLocator, strFullLocator, 4);
  strLocator[5] = char(0);
  //strncpy(strFullLocator, strFullLocator, 7);
  //strFullLocator[]="JO52ff";
/*  debug(strCallsign);
  debug(" ");
  debug(strLocator);
  debug(" ");
  debug((String)pwr_output);
  debug("\n");
  debug("Maidenhead locator: ");
  debug(strFullLocator); */
//  debug((String)tmpStr);
//  debug("\n");
  set_tx_buffer();

  //for(int i=0;i<sizeof(tx_buffer);i++) Serial.print(tx_buffer[i]);
  //Serial.println("");
  //Serial.println(tx_buffer);
}


void process_GPS( const gps_fix & fix ){
  String tmpStr="";
  float tmpLat=0, tmpLon=0;
  if (fix.valid.location) {
   /* tmpStr += (String) fix.satellites + " sats, ";
    tmpLat = fix.latitudeL()/10000000.0;
    tmpLon = fix.longitudeL()/10000000.0;
    tmpStr += String(tmpLat, 7) + ", ";
    tmpStr += String(tmpLon, 7) + ", ";
    tmpStr += (String) fix.alt.whole + " m üNN";
    tmpStr += "\n";
    debug(tmpStr);*/
    
//    if ( ( tx_even && (fix.dateTime.minutes % 2 == 1) && (fix.dateTime.seconds == 55) ) || ( !tx_even && (fix.dateTime.minutes % 2 == 0) && (fix.dateTime.seconds == 55) ) ){
//      Serial.print("preparation of upcoming transmission... ");
      intTxFrequency = intBaseFrequency + random(200);
      debug("New frequency: " + (String) intTxFrequency);
      //setRTC();
      //generateWSPRbuffer(tmpLat, tmpLon);
      generateWSPRbuffer(fix.latitudeL()/10000000.0, fix.longitudeL()/10000000.0);
      
      debug("\n");
//    }
    updateLCD();
    if ( ( tx_even && (fix.dateTime.minutes % 2 == 0) && (fix.dateTime.seconds == 0) ) || ( !tx_even && (fix.dateTime.minutes % 2 == 1) && (fix.dateTime.seconds == 0) ) ){
      do_WSPR();
    }
    
  } else {
    if (toggle_gps_fix) debug("?");
    else debug("-");
    toggle_gps_fix = !toggle_gps_fix;
  }
}
 
void loop() {
  while (gps.available( gpsSerial )) {
    fix = gps.read();
    process_GPS( fix );
  }
}

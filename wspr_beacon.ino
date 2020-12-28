//#include "Wire.h"
#include <si5351.h>
Si5351 si5351;
//#include <SPI.h>

#include <JTEncode.h>
//#include <rs_common.h>
//#include <int.h>
//#include <string.h>
//#define WSPR_DEFAULT_FREQ       14097200UL
JTEncode jtencode;

#include <SoftwareSerial.h>
SoftwareSerial gpsSerial(3, 4);
String gps_time, gps_date, gps_ns, gps_ew, gps_alt, gps_sats, gps_status, gps_strLat, gps_strLon;
double lat=0.0, lon=0.0;

//#include <U8g2lib.h>
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

boolean tx_even = true, 
        tx_enabled = false,
        txing = false,
        toggle_gps_fix = false;
char  strCallsign[7] = "DL4OCE",
      strLocator[5] = "XX00",
      strFullLocator[10];
      
//uint8_t tx_buffer[WSPR_SYMBOL_COUNT]; // 162
uint8_t tx_buffer[255];

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
  debug("\nWSPR beacon based on u-blox NEO-M8N-0-10\n");

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
//  if (fix.valid.location) tmpStr += "G";
//  else tmpStr += "g";
  if (tx_enabled && !txing) tmpStr += "t";
  else if (tx_enabled & txing) tmpStr += "T";
  else tmpStr += "_";
  debug(tmpStr+"\n");
  tmpStr = "";
/*  if (fix.dateTime.hours < 10) tmpStr += "0";
  tmpStr += (String)fix.dateTime.hours + ":";
  if (fix.dateTime.minutes < 10) tmpStr += "0";
  tmpStr += (String)fix.dateTime.minutes + ":";
  if (fix.dateTime.seconds < 10) tmpStr += "0";
  tmpStr += (String)fix.dateTime.seconds + " ";*/
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
  debug("do_WSPR()\n");
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

void generateWSPRbuffer(float tmpLat, float tmpLon){
  debug("\n[generateWSPRbuffer]\n");
  calcLocator(strFullLocator, tmpLat, tmpLon);
  strncpy(strLocator, strFullLocator, 4);
  //strLocator[5] = char(0);
  //strncpy(strFullLocator, strFullLocator, 7);
  //strFullLocator[]="JO52ff";
  debug(strCallsign);
  debug(" ");
  debug(strLocator);
  debug(" ");
  debug((String)pwr_output);
  debug("\n");
  debug("Maidenhead locator: ");
  debug(strFullLocator); 
  memset(tx_buffer, 0, WSPR_SYMBOL_COUNT);  
  //jtencode.wspr_encode(strCallsign, strLocator, pwr_output, tx_buffer);
  //jtencode.wspr_encode("DL4OCE", "JO52", 11, tx_buffer);
  
  //for(int i=0;i<sizeof(tx_buffer);i++) Serial.print(tx_buffer[i]);
  //Serial.println("");
  //Serial.println(tx_buffer);
}

String readParamGps() {
  return(gpsSerial.readStringUntil(','));
}

bool getGps() {
  gpsSerial.readStringUntil('$');
/*  char car = 0;
  while (car != '$') {
    if (gpsSerial.available() > 0) {
      car = gpsSerial.read();
      debug((String)car);
    }
  } //Wait for $ */
  String nmea = readParamGps();
  if (nmea == "GPGGA") {
    gps_time = readParamGps().substring(0,6);
    gpsSerial.readStringUntil('\n');
  }
  if (nmea == "GPRMC") {
    readParamGps();
    gps_status = readParamGps();
    gps_strLat = readParamGps();
    lat = gps_strLat.toFloat() / 100.0;
    gps_ns = readParamGps();
    gps_strLon = readParamGps();
    lon = gps_strLon.toFloat() / 100.0;
    gps_ew = readParamGps();
    gpsSerial.readStringUntil('\n');
  }
  if (gps_status == "A"){
    //updateLCD();
    return true;
  }
  else {
//    if (sizeof(nmea)>0) debug("*_");
    return false;
  }
}
 
void readGPS(){
  int tmp_minutes, tmp_seconds;
  //double tmpLat=0.0, tmpLon=0.0;
  if (getGps()) {
    //debug("GPS fix...\n");
    tmp_minutes = gps_time.substring(2, 4).toInt();
    tmp_seconds = gps_time.substring(4, 6).toInt();
    debug("GPS time: ");
    debug(gps_time);    
    
    debug(" tmp_minutes: ");
    debug((String)tmp_minutes);
    debug(" tmp_seconds: ");
    debug((String)tmp_seconds);
    
    debug("\n");
    if ( ( tx_even && (tmp_minutes % 2 == 1) && (tmp_seconds == 55) ) || ( !tx_even && (tmp_minutes % 2 == 0) && (tmp_seconds == 55) ) ){
      intTxFrequency = intBaseFrequency + random(200);
      debug("New frequency: " + (String)intTxFrequency);
      generateWSPRbuffer(lat, lon);
      debug("\n");
    }
    if ((tx_even && (tmp_minutes % 2 == 0) && (tmp_seconds == 0) ) || ( !tx_even && (tmp_minutes % 2 == 1) && (tmp_seconds == 0))) do_WSPR();
  } else debug("*_");
}
 
void loop(){
  readGPS();
  delay(500);
}

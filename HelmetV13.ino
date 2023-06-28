#include <SPI.h>
#include "FS.h"
#include <SD.h>
#include <sd_defines.h>
#include <sd_diskio.h>
#include <stdlib.h>
#include <string.h>

/*******************************************************************************
 * 
 * File Name:   EdgeLit16_WiFi.ino
 * 
 * Description: Previously: EdgeLit16_WiFi.ino This program now takes input from a micro
 * SD card and displays it on two lobes of LED matrices 

 *                         Copyright (C) 2021 Christopher Biddle
 *                         
 *              This program is free software: you can redistribute it and/or
 *              modify it under the terms of the GNU General Public License as
 *              published by the Free Software Foundation, either version 3 of
 *              the License, or (at your option) any later version. This
 *              program is distributed in the hope that it will be useful, but
 *              WITHOUT ANY WARRANTY; without even the implied warranty of
 *              MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *              GNU General Public License for more details.
 *
 *              You should have received a copy of the GNU General Public
 *              License along with this program.  If not, see
 *              <https://www.gnu.org/licenses/>.
 *              
 *******************************************************************************
 *                              MODIFICATION LOG
 *                              
 * Date                Modified By                       Description
 * ---------- ------------------------- ----------------------------------------
 * 12/09/2021 Chris Biddle              Initial release
 * 07/28/2022 Chris Biddle              Adapted for NodeMCU.
 * 06/26/2023 Cameron Olsen             Adapted for LED Helmet project on ESP32
 ******************************************************************************/

// Import required libraries
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"
#define FASTLED_ESP8266_RAW_PIN_ORDER
#include <FastLED.h>
// Definitions
#define DATA_PIN_L           13
#define CLOCK_PIN_L          12
#define DATA_PIN_R           32
#define CLOCK_PIN_R          14
#define NUM_LEDS             400
#define LED_TYPE            APA102
#define COLOR_ORDER         BGR
#define DEFAULT_FRAME_RATE  40
#define DEFAULT_BRIGHTNESS  2    // 255 is brightest

// Command byte values from I2C master
// Pattern modes must be less than 10
#define CYCLE                        1
#define IMAGE1                       2
#define IMAGE2                       3
#define MULTI_PANEL                  4
#define ANI                          5


CRGB leds_L[NUM_LEDS];
CRGB leds_R[NUM_LEDS];

//int d = 100;

int Mapping[400] = {
0,1,2,3,4,25,26,27,28,29,50,51,52,53,54,75,76,77,78,79,
5,6,7,8,9,30,31,32,33,34,55,56,57,58,59,80,81,82,83,84,
10,11,12,13,14,35,36,37,38,39,60,61,62,63,64,85,86,87,88,89,
15,16,17,18,19,40,41,42,43,44,65,66,67,68,69,90,91,92,93,94,
20,21,22,23,24,45,46,47,48,49,70,71,72,73,74,95,96,97,98,99,
100,101,102,103,104,125,126,127,128,129,150,151,152,153,154,175,176,177,178,179,
105,106,107,108,109,130,131,132,133,134,155,156,157,158,159,180,181,182,183,184,
110,111,112,113,114,135,136,137,138,139,160,161,162,163,164,185,186,187,188,189,
115,116,117,118,119,140,141,142,143,144,165,166,167,168,169,190,191,192,193,194,
120,121,122,123,124,145,146,147,148,149,170,171,172,173,174,195,196,197,198,199,
200,201,202,203,204,225,226,227,228,229,250,251,252,253,254,275,276,277,278,279,
205,206,207,208,209,230,231,232,233,234,255,256,257,258,259,280,281,282,283,284,
210,211,212,213,214,235,236,237,238,239,260,261,262,263,264,285,286,287,288,289,
215,216,217,218,219,240,241,242,243,244,265,266,267,268,269,290,291,292,293,294,
220,221,222,223,224,245,246,247,248,249,270,271,272,273,274,295,296,297,298,299,
300,301,302,303,304,325,326,327,328,329,350,351,352,353,354,375,376,377,378,379,
305,306,307,308,309,330,331,332,333,334,355,356,357,358,359,380,381,382,383,384,
310,311,312,313,314,335,336,337,338,339,360,361,362,363,364,385,386,387,388,389,
315,316,317,318,319,340,341,342,343,344,365,366,367,368,369,390,391,392,393,394,
320,321,322,323,324,345,346,347,348,349,370,371,372,373,374,395,396,397,398,399
};




// Raw image arrays store data from files after function call
CRGB image_1L_raw[400];
CRGB image_1R_raw[400];
CRGB image_2L_raw[400];
CRGB image_2R_raw[400];
// Normal image arrays for after pixel mapping
CRGB image_1L[400];
CRGB image_1R[400];
CRGB image_2L[400];
CRGB image_2R[400];


int rowIndex = 0;
int ledsPerRow = 5;
int numPanels =100;
int framesPerSecond = DEFAULT_FRAME_RATE;
int patternMode = IMAGE1;
byte standbyOff = true;
int currentBrightness = DEFAULT_BRIGHTNESS;
byte currentGradientPulseIteration = 0;


// Set your access point network credentials
const char* ssid = "Helmet";
const char* password = "parisfit";

// In most cases, the IP address will be 192.168.4.1

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

String mainPage = String( "<html>"
                          "<head>"
                          "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
                          "</head>"
                          "<body style=\"font-family:arial,helvetica,sans-serif\">"
                          "<h1 style=\"text-align:center\">LED Motorcycle Helmet Controller</h1>"
                          "<h2 style=\"text-align:center\">Image Selection</h2>"
                          "<div style=\"text-align:center;padding-bottom:10px\"><a href=\"http://192.168.4.1/Image1\">Image 1</a></div>"
                          "<div style=\"text-align:center;padding-bottom:10px\"><a href=\"http://192.168.4.1/Image2\">Image 2</a></div>"
                          //"<div style=\"text-align:center;padding-bottom:10px\"><a href=\"http://192.168.4.1/Animation\">Animation</a></div>"
                          //"<div style=\"text-align:center;padding-bottom:10px\"><a href=\"http://192.168.4.1/Test\">Test Cycle</a></div>"
                          "<h2 style=\"text-align:center\">Brightness</h2>"
                          "<div style=\"text-align:center;padding-bottom:10px\"><a href=\"http://192.168.4.1/br_low\">Low</a></div>"
                          "<div style=\"text-align:center;padding-bottom:10px\"><a href=\"http://192.168.4.1/br_medium\">Medium</a></div>"
                          "<div style=\"text-align:center\"><a href=\"http://192.168.4.1/br_high\">High</a></div>"
                          "<h2 style=\"text-align:center\">Run Mode</h2>"
                          "<div style=\"text-align:center;padding-bottom:10px\"><a href=\"http://192.168.4.1/sby_off\">Enable</a></div>"
                          "<div style=\"text-align:center\"><a href=\"http://192.168.4.1/sby_on\">Disable</a></div>"
                          "</body>"
                          "</html>"
                        );




void setup(){
  Serial.begin( 115200 );

  //SD Setup
  SD.begin(33);

   if(!SD.begin(33)){
    Serial.println("Failed to mount card");
    return;
   }
 ImageRead("/Image_1L.txt",image_1L_raw);
 ImageRead("/Image_1R.txt",image_1R_raw);
 ImageRead("/Image_2L.txt",image_2L_raw);
 ImageRead("/Image_2R.txt",image_2R_raw);

 //Webhost Setup
 //Serial.begin( 115200 );
   
  delay( 3000 ); // power-up safety delay

  // Setting the ESP as an access point
  Serial.println( "Setting AP (Access Point)…" );
  
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP( ssid, password );

  IPAddress IP = WiFi.softAPIP();
  Serial.print( "AP IP address: " );
  Serial.println( IP );

  server.on( "/", HTTP_GET, []( AsyncWebServerRequest *request )
  {
    patternMode = MULTI_PANEL;
    request->send_P( 200, "text/html", mainPage.c_str());
  });

  server.on( "/Image1", HTTP_GET, []( AsyncWebServerRequest *request )
  {
    patternMode = IMAGE1;
    request->send_P( 200, "text/html", mainPage.c_str());
  });

  server.on( "/Image2", HTTP_GET, []( AsyncWebServerRequest *request )
  {
    patternMode = IMAGE2;
    request->send_P( 200, "text/html", mainPage.c_str());
  });

    server.on( "/Animation", HTTP_GET, []( AsyncWebServerRequest *request )
  {
    patternMode = ANI;
    request->send_P( 200, "text/html", mainPage.c_str());
  });

  server.on( "/Test", HTTP_GET, []( AsyncWebServerRequest *request )
  {
    patternMode = MULTI_PANEL;
    request->send_P( 200, "text/html", mainPage.c_str());
  });

  server.on( "/br_low", HTTP_GET, []( AsyncWebServerRequest *request )
  {
    FastLED.setBrightness( 2 );
    currentBrightness = 2;
    request->send_P( 200, "text/html", mainPage.c_str());
  });

  server.on( "/br_medium", HTTP_GET, []( AsyncWebServerRequest *request )
  {
    FastLED.setBrightness( 6 );
    currentBrightness = 6;
    request->send_P( 200, "text/html", mainPage.c_str());
  });

  server.on( "/br_high", HTTP_GET, []( AsyncWebServerRequest *request )
  {
    FastLED.setBrightness( 12 );
    currentBrightness = 12;
    request->send_P( 200, "text/html", mainPage.c_str());
  });

  server.on( "/sby_off", HTTP_GET, []( AsyncWebServerRequest *request )
  {
    standbyOff = true;
    request->send_P( 200, "text/html", mainPage.c_str());
  });

  server.on( "/sby_on", HTTP_GET, []( AsyncWebServerRequest *request )
  {
    standbyOff = false;
    request->send_P( 200, "text/html", mainPage.c_str());
  });

  // Start server
  server.begin();

  FastLED.addLeds<LED_TYPE, DATA_PIN_L, CLOCK_PIN_L, COLOR_ORDER>(leds_L, NUM_LEDS).setCorrection( 0xFFFFFF );
  FastLED.addLeds<LED_TYPE, DATA_PIN_R, CLOCK_PIN_R, COLOR_ORDER>(leds_R, NUM_LEDS).setCorrection( 0xFFFFFF );

  FastLED.setBrightness( DEFAULT_BRIGHTNESS );

  delay( 1000 );


 //Reordering matrix with mapping
  for (int i=0; i<400;i++){
    image_1L[Mapping[i]]=image_1L_raw[i];
    image_1R[Mapping[i]]=image_1R_raw[i];
    image_2L[Mapping[i]]=image_2L_raw[i];
    image_2R[Mapping[i]]=image_2R_raw[i];
  }
}


void loop()
{

  //Standby
  if ( !standbyOff )
  {
    FastLED.clear();
    FastLED.show();
    delay( 500 );
  }

  //Image 1
  else if ( standbyOff && patternMode == IMAGE1 )
  {
    
    //FastLED.clear();
    
    for(int i=0; i<NUM_LEDS;i++){
      
    
            leds_L[i] = image_1L[i]; 
            leds_R[i] = image_1R[i]; 
    }

    FastLED.show(); 
        
  }

  //Image 2
  else if ( standbyOff && patternMode == IMAGE2 )
  {
    
    FastLED.clear();

      for(int i=0; i<NUM_LEDS;i++){
      
            leds_L[i] = image_2L[i]; 
            leds_R[i] = image_2R[i]; 
     }
    FastLED.show(); 
        
   
  }

  //Animation
  else if ( standbyOff && patternMode == ANI )
  {

    
    FastLED.clear();

     
  }


  //Test Cycle
  else if ( standbyOff && patternMode == MULTI_PANEL )
  {
    if ( standbyOff && patternMode == MULTI_PANEL )
    {
      cycleFillPanel( CRGB::Red, ledsPerRow, numPanels, framesPerSecond, true );
      cycleFillPanel( CRGB::Black, ledsPerRow, numPanels, framesPerSecond, true );
    }

    if ( standbyOff && patternMode == MULTI_PANEL )
    {
      cycleFillPanel( CRGB::Yellow, ledsPerRow, numPanels, framesPerSecond, true );
      cycleFillPanel( CRGB::Black, ledsPerRow, numPanels, framesPerSecond, true );
    }

    if ( standbyOff && patternMode == MULTI_PANEL )
    {
      cycleFillPanel( CRGB::Green, ledsPerRow, numPanels, framesPerSecond, true );
      cycleFillPanel( CRGB::Black, ledsPerRow, numPanels, framesPerSecond, true );
    }
    
    if ( standbyOff && patternMode == MULTI_PANEL )
    {
      cycleFillPanel( CRGB::Blue, ledsPerRow, numPanels, framesPerSecond, true );
      cycleFillPanel( CRGB::Black, ledsPerRow, numPanels, framesPerSecond, true );
    }

    if ( standbyOff && patternMode == MULTI_PANEL )
    {
      cycleFillPanel( CRGB::Purple, ledsPerRow, numPanels, framesPerSecond, true );
      cycleFillPanel( CRGB::Black, ledsPerRow, numPanels, framesPerSecond, true );
    }
  }

 
}



void cycleFillPanel( CRGB pColor,
                     int  pLedsPerRow,
                     int  pNumPanels,
                     int  pFramesPerSecond,
                     boolean pForward
                   )
{
  for ( int inx = 0; inx < pNumPanels; inx++ )
  {
    if ( pForward )
    {
      lightUpRow( pColor, pNumPanels - 1 - inx, pLedsPerRow, pNumPanels );
      delay( 1000 / pFramesPerSecond );
    }
    else
    {
      lightUpRow( pColor, inx, pLedsPerRow, pNumPanels );
      delay( 1000 / pFramesPerSecond );
    }
  }
}



void lightUpRow( CRGB pColor,
                 int  pRowIndex,
                 int  pLedsPerRow,
                 int  pNumPanels
               )
{
  int ledIndex = ( pNumPanels - 1 - pRowIndex ) * pLedsPerRow;
  
  for ( int inx = ledIndex; inx < ledIndex + ledsPerRow; inx++ )
  {
    leds_L[inx] = pColor;
    leds_R[inx] = pColor;
  }

  FastLED.show();
}

void clearRow( int pRowIndex,
               int pLedsPerRow,
               int pNumPanels
             )
{
  int ledIndex = ( pNumPanels - 1 - pRowIndex ) * pLedsPerRow;
  
  for ( int inx = ledIndex; inx < ledIndex + ledsPerRow; inx++ )
  {
    leds_L[inx] = CRGB::Black;
    leds_R[inx] = CRGB::Black;
  }

  FastLED.show();
}



  char filehsv[8];
  unsigned int fileint[400];


//Function to read a file into an CRGB
void ImageRead(const char* name, CRGB (&result)[400]){  


  File file1 = SD.open(name, FILE_READ);
  if (!file1) {
  Serial.println("Opening file to read failed");
  return;
  }

  // Resetting the arrays
  memset(filehsv, 0, sizeof(filehsv));
  memset(fileint, 0, sizeof(fileint));

  // Parsing Data
  for(int n=0; n < 399; n++){
    // Skips 0x
    file1.read();
    file1.read();

    // Hex Code
    for(int m=0; m<8; m++){
      filehsv[m] = file1.read();
    }

    // comma space
    file1.read();
    file1.read();

    // Skips 2 at the end of each row
    if(n%20==0 && n!=0){
     file1.read();
     file1.read();  
    }

    // Converts string into int
    fileint[n] = std::stoul(filehsv, nullptr, 16);

    // Sets matrix Equal to ints
    result[n] = fileint[n];
    
  }

  file1.close();

}




















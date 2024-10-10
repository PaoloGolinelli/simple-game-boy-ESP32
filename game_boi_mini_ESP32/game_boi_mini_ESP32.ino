// -------------------------------------------------------------------------------------------------------
#define   MULTIPLAY       true    // set to true to allow for multiplayer games, otherwise set to false
#define   BT_CLASSIC      false   // set to true to use BT classic, false to use Bluetooth Low Energy 
#define   ESP32_MOD       false   // set to true if you want to use a ESP32 wroom, false to use a s3 mini 
// -------------------------------------------------------------------------------------------------------

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <cmath>
#include <String.h>

#if (MULTIPLAY) // ------------------------- MULTIPLAYER
  #if (BT_CLASSIC)
    #include "BluetoothSerial.h"
    #include "esp_bt_device.h"
    #include <esp_mac.h>
  #else
    #include <BLEDevice.h>
    #include <BLEServer.h>
  #endif
#endif // ----------------------------------------------

// Macro functions
#define SIGN(x) ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))
#define ROUND_TO_1_DECIMALS(x) (int(x*10.0) / 10.0)


// Game parameters
#define   SCREEN_WIDTH    128   // OLED display width, in pixels
#define   SCREEN_HEIGHT   64    // OLED display height, in pixels
#define   SNAKE_WIDTH     6     // Snake display width, in pixels
#define   SNAKE_HEIGHT    6     // Snake display height, in pixels
#define   POOL_FORCE_COEF 5.6   // Multiplier of force to calc pool initial velocity 
const int t_menu =        700;  // if pressed for this amount of ms returns to menu
const int t_sample_menu = 200;  // ms each sample is made in the menu selection
const int t_bounce =      40;   // time in ms to account for switch bouncing
#define   ANALOG_RES      4095  // Resolution of ADC
#define   d_to_r          3.14/180.0


// Declaration for an SSD1306 display connected to I2C (default pins are SDA: GPIO21, SCL: GPIO22)
#define          OLED_RESET    -1    // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display (SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// define a struct to handle coordinates (double x,y, and direction)
struct crd {  // coordinate
  double x;
  double y;
  int    d;   // snakes heading direction for each coordinate
};

#if (MULTIPLAY) // ------------------------- MULTIPLAYER
  #define NRGAMES 5   // number of games
  const char *options[NRGAMES] = { 
    " Snake ", 
    " Pool ", 
    " Hit the target ",
    " Pong multiplayer ",
    " Calibration "};
#else // --------------------------------- SINGLE PLAYER
  #define NRGAMES 4   // number of games
  const char *options[NRGAMES] = { 
    " Snake ", 
    " Pool ", 
    " Hit the target ",
    " Calibration "};
#endif // ----------------------------------------------

const char *difficult[3] = {
  " Easy ",
  " Medium ",
  " Hard " };


#if (ESP32_MOD) // Input Pin Wroom
  const int     AI1           = 35;   // IO26 - analog signal x joystick
  const int     AI2           = 26;   // IO18 - analog signal y joystick
  const int     switchPin     = 19;   // IO19
  const int     SCLpin        = 22;   // IO21
  const int     SDApin        = 21;   // IO21
#else           // Input Pin S3 mini
  const int     AI1           = 8;    // GP8 - analog signal x joystick
  const int     AI2           = 9;    // GP9 - analog signal y joystick
  const int     switchPin     = 7;    // GP7
  const int     SCLpin        = 2;    // GP1
  const int     SDApin        = 1;    // GP2
#endif

// global variables
int           gameRate;               // time in ms between moves
unsigned long t, t_press, t_release;

// snake variables
bool  alive           = true;       
int   length;                   // current snake length
crd   position[200]   = {0};    // array containing snake's position
crd   food;
int   nr_width        = SCREEN_WIDTH/SNAKE_WIDTH;
int   nr_height       = SCREEN_HEIGHT/SNAKE_HEIGHT;

// pool variables
crd          balls[4];          // initial position to be set at beginning of game
crd          b_vel[4];          // define the carthesian velocity of the balls
crd          stick[3];          // defines the position of the stick
bool         bhole[4];          // true if they end up in a hole
double       stick_angle, stick_F;
const double hole_R   = 7;
const double ball_R   = 4;
const double stick_L  = 18;
const double dt       = 0.03;   // integration time in seconds
const double fr_coeff = 0.4;    // friction coefficient

// hit the target variables
int           d_range = 8;
double        dist, dist_tot;
unsigned long t_tot;
double        score;

#if (MULTIPLAY) // ----------- MULTIPLAYER -------------------------
  // BlueTooth variable
  #if (BT_CLASSIC)
    BluetoothSerial SerialBT; 
    uint8_t slaveMACAddress[6] = {0x32, 0xAE, 0xA4, 0x07, 0x0D, 0x66}; // slave MAC address
    uint8_t oldMACAddress[6];
  #else
    #define SERVICE_UUID        "37859a32-9f35-4408-b4e8-ad61f9bc1f31"
    #define CHARACTERISTIC_UUID "e21f0560-f548-4c6c-a189-81b2264e5485"

    //static BLEUUID serviceUUID(SERVICE_UUID);
    //static BLEUUID charUUID(CHARACTERISTIC_UUID);
    static BLEUUID serviceUUID("37859a32-9f35-4408-b4e8-ad61f9bc1f31");     // DEBUG
    static BLEUUID charUUID("e21f0560-f548-4c6c-a189-81b2264e5485");

    // server
    BLEServer         *pServer          = NULL;
    BLECharacteristic *pCharacteristic  = NULL;
    bool    deviceConnected     = false;
    bool    oldDeviceConnected  = false;    
    // client
    static  BLERemoteCharacteristic *pRemoteCharacteristic;
    static  BLEAdvertisedDevice     *myDevice;
    static  boolean connected   = false;
    // common
    bool    notified;
    double  received_value;
  #endif

  // pong variables
  #define     XV        70      // ball's horizontal velocity
  #define     MAX_XV    110     // ball's max horizontal velocity
  double      max_t_v = 0.05;   // max pixels a tile can move per millisecond
  const int   tile_H  = 10;     // nr of pixels defining the tile height
  const int   tile_W  = 3;      // nr of pixels defining the tile width
  const int   bord_W  = 3;      // border width
  const int   v_bound = 10;     // pixel above field (for displaying score)
  const int   bR      = 2;      // ball radius
  crd         ball_xy;          // ball coordinates
  crd         ball_v;           // ball velocities
  double      t_y1, t_y2;       // y coordinates of the two tiles
  crd         pr_ball_xy;       // previous ball coordinates
  double      pr_y1, pr_y2;     // previous y coordinates of the two tiles
  double      Dt_y1;            // receiver tile difference with previous y coord
  bool        scored;           // turned to true when it needs to update the score
  bool        is_master;        // set to T when gaimboi is master in BT communication     
  bool        play_receive;     // set to T when the current device is receiving the ball
  int         score_1, score_2; // score of the two players

#endif // ----------------------------------------------------------


// joystick variables
double  xa = 238, xd = 4.30, xe = -10.89;  // parabola parameters
double  ya = 192, yd = 5.33, ye = -18.75;  // initial calibration
double  Xval, Yval, Xlin, Ylin, magn, theta;
int     dir, precDir;
bool    sw_press = true;

// menu variables
bool    gotomenu, mnpress, sw_rel;
int     selected = 0;
int     entered  = -1;

// Task handles for parallel programming
TaskHandle_t handle_snake_joy_read;

// functions
void starting_animation ();
void menu               ();                 // displays menu to choose the game
void readJoystick       ();                 // read joystick values
  void calculate_dir    ();                 // calculate direction indicated by joystick
  void calibrate        ();                 // calibration procedure defines parabola parameters to linearize joystick readings
void ICACHE_RAM_ATTR sw_activity (void);    // deals joystick switch pressing and releasing

void snake_game         (); 
  void move             (int);              // function moves snake accordingly to the direction
  void eat              (bool);             // if heading position equals food position it increment snakes length and place new food; bolean defines first call
  void printFrame       ();                 // print frame on display

void pool_game          ();
  void pool_aim         ();
  void pool_balls_anim  ();
  void pool_printfield  (bool = false, bool = false);
  void collision        (int, int);
  void hole_ball        (int);

void target_game        ();

#if (MULTIPLAY) // -------------- MULTIPLAYER GAMES ----------
void pong_game          ();
  bool initialize_BT    ();                 // starts BT in master/slave mode and pair the devices
#if (!BT_CLASSIC)
  bool connectToServer  ();                 // used by slave to connect to server
#endif
  void pong_printfield  ();                 // prints the two tiles, the ball and the score 
#endif // ----------------------------------------------------


void setup() {
  Serial.begin(115200);
  Serial.printf("\nInitiate 201\n");

  pinMode(switchPin, INPUT_PULLUP);

  Wire.begin(SDApin, SCLpin); // define the pins for I2C (SDA, SCL)

  attachInterrupt(digitalPinToInterrupt(switchPin), sw_activity, CHANGE);
  t_release = millis();
  t_press   = millis();

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;) delay(1); // Don't proceed, loop forever
  }

  // starting animation "GAIM BOY"
  starting_animation();

  delay(500);
  display.clearDisplay();

  display.setTextColor(1); display.setTextSize(1);
  display.setCursor(5, 20); display.print("At any moment press");
  display.setCursor(5, 29); display.print("the switch for ~1 s");
  display.setCursor(5, 38); display.print(" to return to menu");
  display.display();

  delay(100);

  // display "press button":
  while (sw_press)
    delay(1);

  // Milliseconds at which switch is pressed defines random seed
  randomSeed(millis()%1000);

  // menù here going through the games
  menu();
}


void loop() {

  if (gotomenu) {
    menu();
  }
  if (selected == 0) 
    snake_game();
  else if (selected == 1) {
    pool_game();
  }
  else if (selected == 2) {
    target_game();
  }
#if (MULTIPLAY) // ----------- MULTIPLAYER ---------
  else if (selected == 3) {
    pong_game();
  }
  else if (selected == 4) {
    calibrate();
  }
#else // -------------------- SINGLE PLAYER --------
  else if (selected == 3) {
    calibrate();
  }
#endif // ------------------------------------------

  delay(100);
}


void starting_animation() {
  double  tsnake = 120;
  double  angle;
  int     sign;
  crd     aimer;
  aimer.x = 160;
  aimer.y = 40;

  display.setTextColor(1);
  display.setTextSize(3);

  // balls initial position and velocities
  balls[0].x = 23;    balls[0].y = 41;
  b_vel[0].x = 0;     b_vel[0].y = 0;
  balls[1].x = -6;    balls[1].y = 36;
  b_vel[1].x = 35.5;  b_vel[1].y = 10;

  t_tot = millis();
  while (millis() - t_tot <= 3000) {
    display.clearDisplay();
    display.setCursor(28, 0);   display.println(F("GAIM"));
    display.setCursor(37, 43);  display.println(F("B I"));

    // calculate new aimer coords
    aimer.x = 64.0 + 2.0*pow(3.0-((millis()-t_tot)/1000.0), 2);
    aimer.y = 54.0 - 2.2*pow(3.0-((millis()-t_tot)/1000.0), 3);

    display.drawCircle(aimer.x, aimer.y, d_range, 1);
    display.drawCircle(aimer.x, aimer.y, 2, 1);
    display.drawFastHLine(aimer.x-(d_range+1), aimer.y, 2*(d_range+1)+1, 1);
    display.drawFastVLine(aimer.x, aimer.y-(d_range+1), 2*(d_range+1)+1, 1);

    for (int i = 0; i < 2; i++) {
      balls[i].x += b_vel[i].x * dt;
      balls[i].y += b_vel[i].y * dt;

      // dinamic rolling friction:
      angle = atan(b_vel[i].y/b_vel[i].x);
      sign = b_vel[i].x >= 0 ? 1 : -1;
  
      if (fabs(b_vel[i].x) > 0.11)
        b_vel[i].x -= sign*0.2*cos(angle);
      else
        b_vel[i].x = 0;   // static friction: too slow -> come to a stop

      if (fabs(b_vel[i].y) > 0.11)
        b_vel[i].y -= sign*0.2*sin(angle);
      else
        b_vel[i].y = 0;   // static friction 

      display.fillCircle(balls[i].x, balls[i].y, ball_R, 1);
    }

    if (sqrt(pow(balls[0].x - balls[1].x, 2) + pow(balls[0].y - balls[1].y, 2)) <= 2*ball_R)
      collision(0, 1);

    display.display();
  }

  int sn_d = 16; // snake dimension
  display.setTextColor(0);
  display.setTextSize(2);

  // snake's initial positions
  length = 6;
  for (int s = length-1; s >=0 ; s--) {
    position[s].x = -sn_d*s - sn_d/2; 
    position[s].y = SCREEN_HEIGHT/2+2;
  }

  for (int n = 0; n < 2*(length)+3; n++) {
    t = millis();
    display.display();

    for (int l = 0; l < length; l++)
      position[l].x += sn_d/2;

    display.fillRect(0, SCREEN_HEIGHT/2-10, SCREEN_WIDTH, 20, 0);

    // display snake's head
    display.fillRoundRect(position[0].x-sn_d/2-5, position[0].y-sn_d/2, sn_d+5, sn_d, sn_d/4+1, 1); // below mouth

    if (n < 2*length-4 || n >= 2*length-1)
      display.fillTriangle(position[0].x-3, position[0].y, position[0].x+sn_d/2, position[0].y+1, position[0].x+sn_d/2, position[0].y-1, 0); // mouth
    else
      display.fillTriangle(position[0].x-1, position[0].y, position[0].x+sn_d/2, position[0].y+5, position[0].x+sn_d/2, position[0].y-5, 0); // mouth

    for (int l = 1; l < length-1; l++)
      display.fillRect(position[l].x - sn_d/2, position[l].y - sn_d/2, sn_d, sn_d, 1);

    display.fillCircle(position[0].x-8, position[0].y-8, 4, 1);                                 // above eye
    display.fillCircle(position[0].x-8, position[0].y-8, 2, 0);                                 // eye

    // display tail
    display.fillRect(position[length-1].x-sn_d, position[length-1].y-sn_d/2, sn_d+8, sn_d, 1);

    display.fillCircle(position[length-1].x+3, position[length-1].y+sn_d/2+1, 7, 0);
    display.fillCircle(position[length-1].x-6, position[length-1].y-sn_d/2-1, 7, 0);
    display.fillTriangle(position[length-1].x-sn_d, position[length-1].y-sn_d/2, position[length-1].x-sn_d, position[length-1].y+sn_d/2, position[length-1].x-sn_d+9, position[length-1].y+sn_d/2, 0);

    display.fillCircle(balls[0].x, balls[0].y, ball_R, 1);  // display deleted ball
    display.fillCircle(balls[1].x, balls[1].y, ball_R, 1);  // display deleted ball

    display.setCursor(-65+n*sn_d/2, SCREEN_HEIGHT/2-5); display.print(F("MINI"));

    while (millis() - t < tsnake)
      delay(1);
  }
}


void menu () {
  gotomenu = false;
  selected = 0;
  
  while(!sw_press)
    delay(1);

  mnpress = false;
  while (true) {
    readJoystick();
    calculate_dir();

    if (dir == 1) {
      selected--;
    }
    else if (dir == 3) {
      selected++;
    }

    selected = (selected + NRGAMES) % NRGAMES;

    if (mnpress) {
      break;
    }
    else {
      display.clearDisplay();
      display.setTextSize(1);             
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.println(F("Games"));
      display.println("");

      for(int i=0; i < NRGAMES; i++) {
        if (i == selected) {
          display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
          display.println(options[i]);
        }
        else if (i != selected) {
          display.setTextColor(SSD1306_WHITE);
          display.println(options[i]);
        }
      }
      display.display();
      delay(t_sample_menu);
    }
  }
}

/*---------------------
  |       SNAKE       |
  --------------------- */
// function called on secondary core to run in parallel with snake main loop
// it reads the joistick inputs so that the other core can  
void snake_joy_read(void *parameter) {  
  while (true) {
    vTaskDelay(1);  // Short delay to yield control
  
    // Read the joystick and calculate the direction
    readJoystick();

    calculate_dir();

    if (!alive) {
      break;
    }
  }

  vTaskDelete(NULL);
}

void snake_game() {
  while(!sw_press)
    delay(1);
  
  selected  = 0;
  mnpress   = false;
  while (true) {
    readJoystick();
    calculate_dir();

    if (dir == 1) {
      selected--;
    }
    else if (dir == 3) {
      selected++;
    }

    selected = (selected + 3) % 3;

    if (mnpress) {
      break;
    }
    else {
      display.clearDisplay();
      display.setTextSize(1);             
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.println(F("Select difficulty"));
      display.println("");

      for(int i=0; i < 3; i++) {
        if (i == selected) {
          display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
          display.println(difficult[i]);
        }
        else if (i != selected) {
          display.setTextColor(SSD1306_WHITE);
          display.println(difficult[i]);
        }
      }
      display.display();
      delay(t_sample_menu);
    }
  }

  // select difficulty
  if (selected == 0) 
    gameRate = 420;
  else if (selected == 1)
    gameRate = 320;
  else
    gameRate = 220;

  // defines initial position
  position[0].x = nr_width/2 + 1;
  position[0].y = nr_height/2;
  position[0].d = 2;
  position[1].x = nr_width/2;
  position[1].y = nr_height/2;
  position[1].d = 2;
  position[2].x = nr_width/2 - 1;
  position[2].y = nr_height/2;
  position[2].d = 2;
  length  = 2;  // incremented to 3 by eat(true) call
  precDir = 2;
  eat(true);    // defines first food position
  alive = true;

  // call to handle_snake_joy_read task on secondary core
  xTaskCreatePinnedToCore(
    snake_joy_read,           // Function to implement the task
    "snake_joy_read",         // Name of the task
    2048,                     // Stack size in words
    NULL,                     // Task input parameter
    1,                        // Priority of the task
    &handle_snake_joy_read,   // Task handle
    0);                       // Core where the task should run


  while (alive) {
    t = millis();
  
    if (dir == 0)
      dir = precDir;

    // controls new direction isn't the opposite of of current heading
    if ((dir+2)%4 == precDir%4) {
      dir = precDir;
    }

    move(dir);
    precDir = dir;
    
    printFrame();
    
    eat(false);  // false because it's not the first call
        
    while (millis() - t < gameRate)
      delay(1);
  }

  display.clearDisplay();
  display.setTextSize(2);     
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(SCREEN_WIDTH/2-50, SCREEN_HEIGHT/2-20);
  display.println("GAME OVER");
  display.println("");
  display.setCursor(SCREEN_WIDTH/2-52, SCREEN_HEIGHT/2+2);
  display.print("Score: ");
  display.println(length-3);
  display.display();

  gotomenu = true;

  while(sw_press)
    delay(1);
}

void move(int drct) {
/*  Dir | heading
     1  | up
     2  | right
     3  | dowm
     4  | left    */

  int newX = position[0].x;
  int newY = position[0].y;

  if (drct == 1) {
    if (position[0].y == 0)
      alive = false;

    newY--;
  }
  else if (drct == 2) {
    if (position[0].x == nr_width-1)
      alive = false;

    newX++;
  }
  else if (drct == 3) {
    if (position[0].y == nr_height-1)
      alive = false;

    newY++;
  }
  else if (drct == 4) {
    if (position[0].x == 0)
      alive = false;

    newX--;
  }

  // moves the coordinates down the snake one at a time
  for (int t = length-1; t >= 0; t--) {
    position[t+1].x = position[t].x;
    position[t+1].y = position[t].y;
    position[t+1].d = position[t].d;
  }

  // verify the head doesn't collide with snake
  for (int k = 0; k < length; k++) {
    if (newX == position[k].x && newY == position[k].y) {
      alive = false;
    }
  }

  // assign new heading position
  position[0].x = newX;
  position[0].y = newY;
  position[0].d = drct;
}

void eat(bool first) {
  if (first || (position[0].x == food.x && position[0].y == food.y)) {
    length++;

    bool notPresent = false;
    while (!notPresent) {
      notPresent = true;

      food.x = random(nr_width);
      food.y = random(nr_height);

      for (int l = 0; l < length; l++) {
        if (food.x == position[l].x && food.y == position[l].y) {
          notPresent = false;
          break;
        }
      }
    }
  }
}

void printFrame() {
  int imin, imax, jmin, jmax;

  display.clearDisplay();
  
  display.drawFastHLine(0, 0, SCREEN_WIDTH, 1);
  display.drawFastHLine(0, SCREEN_HEIGHT-1, SCREEN_WIDTH, 1);
  display.drawFastVLine(0, 0, SCREEN_HEIGHT, 1);
  display.drawFastVLine(SCREEN_WIDTH-1, 0, SCREEN_HEIGHT, 1);

  for (int s = 0; s < length; s++) {

    if (position[s].d == 2 || position[s-1].d == 4)
      imin = 0;
    else
      imin = 1;

    if (position[s].d == 4 || position[s-1].d == 2)
      imax = SNAKE_WIDTH;
    else
      imax = SNAKE_WIDTH-1;

    if (position[s].d == 3 || position[s-1].d == 1)
      jmin = 0;
    else
      jmin = 1;

    if (position[s].d == 1 || position[s-1].d == 3)
      jmax = SNAKE_HEIGHT;
    else
      jmax = SNAKE_HEIGHT-1;
    
    display.fillRect(position[s].x*SNAKE_WIDTH + imin, position[s].y*SNAKE_HEIGHT + jmin + 1, imax - imin, jmax - jmin, 1);
  }

  if (position[0].d == 2 || position[0].d == 4) {
    display.fillCircle(position[0].x*SNAKE_WIDTH + SNAKE_WIDTH/2, position[0].y*SNAKE_HEIGHT+1 + 1, 1, 1);
    display.fillCircle(position[0].x*SNAKE_WIDTH + SNAKE_WIDTH/2, (position[0].y+1)*SNAKE_HEIGHT-1 + 1, 1, 1);
  }
  else if (position[0].d == 1 || position[0].d == 3) {
    display.fillCircle(position[0].x*SNAKE_WIDTH+1, position[0].y*SNAKE_HEIGHT + SNAKE_HEIGHT/2 + 1, 1, 1);
    display.fillCircle((position[0].x+1)*SNAKE_WIDTH-1, position[0].y*SNAKE_HEIGHT + SNAKE_HEIGHT/2 + 1, 1, 1);
  }

  display.fillCircle(food.x*SNAKE_WIDTH + SNAKE_WIDTH/2, food.y*SNAKE_HEIGHT + SNAKE_HEIGHT/2 + 1, SNAKE_WIDTH/2, 1);

  display.display();
}


/*---------------------
  |       POOL        |
  --------------------- */
void pool_game() {

  while (!sw_press)
    delay(1);

  // draw pool table
  pool_printfield();
  display.display();

  // print game name
  display.setTextSize(2);     
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(SCREEN_WIDTH/2-21, SCREEN_HEIGHT/2-6);
  display.println("POOL");
  display.display();

  delay(500);
  
  // delay some time 
  display.setTextSize(1);     
  display.setCursor(SCREEN_WIDTH/2-41, SCREEN_HEIGHT/2+10);
  display.println(F("click to start"));
  display.display();
  alive = true;

  while (sw_press)
    delay(1);

  // initialize coordinates
  balls[0].x = 35; balls[0].y = SCREEN_HEIGHT/2;
  balls[1].x = 90; balls[1].y = SCREEN_HEIGHT/2-(ball_R+1);
  balls[2].x = 90; balls[2].y = SCREEN_HEIGHT/2+(ball_R+1);
  balls[3].x = 90-sqrt(3)*(ball_R+1); balls[3].y = SCREEN_HEIGHT/2;

  for (int l = 0; l < 4; l++)
    bhole[l] = false;

  pool_printfield(true);

  delay(200);

  while (alive) {
    pool_aim ();

    b_vel[0].x =  POOL_FORCE_COEF * stick_F * cos(stick_angle*d_to_r);
    b_vel[0].y = -POOL_FORCE_COEF * stick_F * sin(stick_angle*d_to_r);

    pool_printfield();
    pool_balls_anim();
    
    if (bhole[0] == true) {
      alive = false;
      gotomenu = true;

      display.setTextSize(2);     
      display.setTextColor(SSD1306_WHITE);
      display.fillRect(SCREEN_WIDTH/2-54, SCREEN_HEIGHT/2-6, 110, 18, 0);
      display.setCursor(SCREEN_WIDTH/2-53, SCREEN_HEIGHT/2-5);
      display.println("GAME OVER");
      display.display();
      
      while(sw_press)
        delay(1);
    }
    else if (bhole[1] == true && bhole[2] == true && bhole[3] == true) { 
      alive = false;
      gotomenu = true;

      display.setTextSize(2);     
      display.setTextColor(SSD1306_WHITE);
      display.fillRect(SCREEN_WIDTH/2-41, SCREEN_HEIGHT/2-6, 70, 18, 0);
      display.setCursor(SCREEN_WIDTH/2-40, SCREEN_HEIGHT/2-5);
      display.println("YOU WIN");
      display.display();

      while(sw_press)
        delay(1);
    }
  }
}

void pool_aim() {
  stick_angle = 0;
  stick_F = 5;

  delay(100);

  while (sw_press) {
    readJoystick();

    if (magn > 0.1) {
      int sign = Ylin > 0 ? 1 : -1;
      stick_angle += 6*sign*(pow(Ylin,2));
    }

    stick[0].x = balls[0].x - (ball_R+2) * cos(stick_angle*d_to_r);
    stick[0].y = balls[0].y + (ball_R+2) * sin(stick_angle*d_to_r);
    stick[1].x = stick[0].x - stick_L * cos((stick_angle+4)*d_to_r);
    stick[1].y = stick[0].y + stick_L * sin((stick_angle+4)*d_to_r);
    stick[2].x = stick[0].x - stick_L * cos((stick_angle-4)*d_to_r);
    stick[2].y = stick[0].y + stick_L * sin((stick_angle-4)*d_to_r);

    pool_printfield(true, true);
  }

  while(!sw_press)
    delay(1);

  // started pressing: 
  while (sw_press && alive) { // check
    readJoystick();

    if (abs(Xlin) > 0.1)
      stick_F -= 0.5*Xlin;

    if (stick_F < 0)
      stick_F = 0;
    else if (stick_F > 10)
      stick_F = 10;

    display.fillRect(SCREEN_WIDTH/2-20, SCREEN_HEIGHT-8, 40,        5, 0);
    display.drawRect(SCREEN_WIDTH/2-20, SCREEN_HEIGHT-8, 40,        5, 1);
    display.fillRect(SCREEN_WIDTH/2-20, SCREEN_HEIGHT-8, 4*stick_F, 5, 1);
    display.display();
  }

  stick_F += 2;
}

void pool_balls_anim() {
  double angle;
  int sign;

  // set velocities to 0
  b_vel[1].x = 0; b_vel[1].y = 0;
  b_vel[2].x = 0; b_vel[2].y = 0;
  b_vel[3].x = 0; b_vel[3].y = 0;

  while (alive) {
    // simulation proceedure: integration of accelerations and velocities to define new position
    t = millis();
    display.fillRect(hole_R+1, hole_R+1, SCREEN_WIDTH-2*hole_R-1, SCREEN_HEIGHT-2*hole_R-1, 0);
  
    for (int j = 0; j < 4; j++) {
      balls[j].x += b_vel[j].x * dt;
      balls[j].y += b_vel[j].y * dt;

      // dinamic rolling friction:
      angle = atan(b_vel[j].y/b_vel[j].x);  // costly
      sign = b_vel[j].x >= 0 ? 1 : -1;

      if (fabs(b_vel[j].x) > fr_coeff/2)
        b_vel[j].x -= sign*fr_coeff*cos(angle);
      else
        b_vel[j].x = 0;   // static friction: too slow -> come to a stop
  
      if (fabs(b_vel[j].y) > fr_coeff/2)
        b_vel[j].y -= sign*fr_coeff*sin(angle);
      else
        b_vel[j].y = 0;   // static friction

      // check if the balls hit the wall or ended up in a hole  
      if (balls[j].x < ball_R+hole_R) {
        if (balls[j].y < ball_R+hole_R+2 || balls[j].y > SCREEN_HEIGHT-ball_R-hole_R-2) {
          if (balls[j].x < hole_R+1)
            hole_ball(j);
        }
        else {
          b_vel[j].x *= -1;
          balls[j].x = 2 * (ball_R+hole_R) - balls[j].x;
        }
      }
      else if (balls[j].x > SCREEN_WIDTH-ball_R-hole_R) {
        if (balls[j].y < ball_R+hole_R+2 || balls[j].y > SCREEN_HEIGHT-ball_R-hole_R-2) {
          if (balls[j].x < SCREEN_WIDTH-hole_R-1)
            hole_ball(j);
        }
        else {
          b_vel[j].x *= -1;
          balls[j].x = 2 * (SCREEN_WIDTH-ball_R-hole_R) - balls[j].x;
        }
      }
      if (balls[j].y < ball_R+hole_R) {
        if (balls[j].x < SCREEN_WIDTH/2+hole_R && balls[j].x > SCREEN_WIDTH/2-hole_R || balls[j].x < ball_R+hole_R+2 || balls[j].x > SCREEN_WIDTH-ball_R-hole_R-2) {
          if (balls[j].y < hole_R+1)
            hole_ball(j);
        }
        else {
          b_vel[j].y *= -1;
          balls[j].y = 2 * (ball_R+hole_R) - balls[j].y;
        }
      }
      else if (balls[j].y > SCREEN_HEIGHT-ball_R-hole_R) {
        if (balls[j].x < SCREEN_WIDTH/2+hole_R && balls[j].x > SCREEN_WIDTH/2-hole_R || balls[j].x < ball_R+hole_R+2 || balls[j].x > SCREEN_WIDTH-ball_R-hole_R-2) {
          if (balls[j].y > SCREEN_HEIGHT-hole_R-1)
            hole_ball(j);       
        }
        else {
          b_vel[j].y *= -1;
          balls[j].y = 2 * (SCREEN_HEIGHT-ball_R-hole_R) - balls[j].y;
        }
      }

      if (bhole[j] == true) 
        continue;  // skips the ball if it ended in a hole

      // check collision with each other
      for (int i = j+1; i < 4; i++) {
        if (sqrt(pow(balls[i].x - balls[j].x, 2) + pow(balls[i].y - balls[j].y, 2)) <= 2*ball_R) {
          collision(i, j);
        }
      }

      // display the ball
      display.fillCircle(balls[j].x, balls[j].y, ball_R, 1);
    }
    display.fillCircle(balls[0].x, balls[0].y, 2, 0);
    display.display();
  
    if (b_vel[3].x == 0 & b_vel[3].y == 0 & b_vel[2].x == 0 & b_vel[2].y == 0 & b_vel[1].x == 0 & b_vel[1].y == 0 & b_vel[0].x == 0 & b_vel[0].y == 0)
      break;

    // check if integration time passed, otherwise wait
    while (millis() - t < 1000*dt)
      delay(1);
  }
}

void hole_ball(int nr_bal) { // called if a ball ended inside a hole
  bhole[nr_bal]   = true;
  balls[nr_bal].x = NAN; balls[nr_bal].y = NAN;
  b_vel[nr_bal].x = 0;   b_vel[nr_bal].y = 0;
}

void collision(int b1, int b2) { 
  double beta = atan((balls[b1].y - balls[b2].y)/(balls[b1].x - balls[b2].x)); // angle between ball's centers

  if (balls[b1].x - balls[b2].x < 0) 
    beta += 3.1415;

  double vr1 = b_vel[b1].x * cos(beta) + b_vel[b1].y * sin(beta);
  double vt1 = b_vel[b1].x * sin(beta) - b_vel[b1].y * cos(beta);

  double vr2 = b_vel[b2].x * cos(beta) + b_vel[b2].y * sin(beta);
  double vt2 = b_vel[b2].x * sin(beta) - b_vel[b2].y * cos(beta);

  if (vr2 > vr1) {
    b_vel[b1].x = vr2*cos(beta) + vt1*sin(beta);
    b_vel[b1].y = vr2*sin(beta) - vt1*cos(beta);

    b_vel[b2].x = vr1*cos(beta) + vt2*sin(beta);
    b_vel[b2].y = vr1*sin(beta) - vt2*cos(beta);
  }
}

void pool_printfield(bool pr_ball, bool pr_stick) {
  display.clearDisplay();

  display.drawLine(2*hole_R, hole_R, SCREEN_WIDTH/2-hole_R, hole_R, 1);   display.drawLine(SCREEN_WIDTH/2+hole_R, hole_R, SCREEN_WIDTH-2*hole_R, hole_R, 1);
  display.drawLine(hole_R, 2*hole_R, hole_R, SCREEN_HEIGHT-2*hole_R, 1);  display.drawLine(SCREEN_WIDTH-hole_R, 2*hole_R, SCREEN_WIDTH-hole_R, SCREEN_HEIGHT-2*hole_R, 1);
  display.drawLine(2*hole_R, SCREEN_HEIGHT-hole_R, SCREEN_WIDTH/2-hole_R, SCREEN_HEIGHT-hole_R, 1);  
  display.drawLine(SCREEN_WIDTH/2+hole_R, SCREEN_HEIGHT-hole_R, SCREEN_WIDTH-2*hole_R, SCREEN_HEIGHT-hole_R, 1);
  display.drawCircle(hole_R, hole_R, hole_R, 1); display.drawCircle(SCREEN_WIDTH/2, hole_R, hole_R, 1); display.drawCircle(SCREEN_WIDTH-hole_R, hole_R, hole_R, 1); 
  display.drawCircle(hole_R, SCREEN_HEIGHT-hole_R, hole_R, 1); display.drawCircle(SCREEN_WIDTH/2, SCREEN_HEIGHT-hole_R, hole_R, 1); display.drawCircle(SCREEN_WIDTH-hole_R, SCREEN_HEIGHT-hole_R, hole_R, 1); 
  display.fillRect(hole_R+1, hole_R+1, SCREEN_WIDTH-2*hole_R-1, SCREEN_HEIGHT-2*hole_R-1, 0);

  if (pr_ball) {
    for (int v = 0; v < 4; v++)
      if (bhole[v] == false)
        display.fillCircle(balls[v].x, balls[v].y, ball_R, 1);

    display.fillCircle(balls[0].x, balls[0].y, 2, 0);
  }

  if (pr_stick)
    display.fillTriangle(stick[0].x, stick[0].y, stick[1].x, stick[1].y, stick[2].x, stick[2].y, 1);

  display.display();
}

  
/*---------------------
  |   Hit the target  |
  --------------------- */
void target_game() {

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(1);
  display.setCursor(20,5); display.print(F("Hit the target")); 
  display.setCursor(8,14); display.print(F("as fast as you can"));
  display.display();

  delay(1000);

  display.setCursor(21,30); display.println(F("click to start"));
  display.display();

  while(sw_press)
    delay(1);

  food.x = random(d_range, SCREEN_WIDTH - d_range);
  food.y = random(d_range, SCREEN_HEIGHT - d_range);
  
  position[0].x = SCREEN_WIDTH/2;
  position[0].y = SCREEN_HEIGHT/2;

  t_tot     = millis();
  t         = millis();
  dist_tot  = 0;
  alive     = true;

  for (int i = 0; i < 20 && alive; i++) {
    while (alive) {
      display.clearDisplay();

      // display the target
      display.drawCircle(food.x, food.y, d_range, 1);
      display.drawCircle(food.x, food.y, 2, 1);
      display.drawFastHLine(food.x-(d_range+1), food.y, 2*(d_range+1)+1, 1);
      display.drawFastVLine(food.x, food.y-(d_range+1), 2*(d_range+1)+1, 1);

      // display the pointing dot
      display.fillCircle(position[0].x, position[0].y, 2, 1);

      display.display();

      // if button is pressed check distance is within radius
      if (!sw_press && sw_rel) {
        sw_rel = false;
        // compute distance (added to accuracy) 
        dist = sqrt(pow(position[0].x - food.x, 2) + pow(position[0].y - food.y, 2));
        Serial.print("clicked at dist: "); Serial.println(dist);

        // save total distance for accuracy measure
        if (dist > 2)
          dist_tot += dist;

        if (dist <= d_range) {
          // define new coordinate
          food.x = random(d_range, SCREEN_WIDTH  - d_range);
          food.y = random(d_range, SCREEN_HEIGHT - d_range);
          break;
        }
      }

      // read joystick
      readJoystick(); 

      // this would be using polar cordinates but it's not very precise
      // position[0].x += magn * cos(theta);
      // position[0].y += magn * sin(theta);

      // move to new position integrating the 2 distances
      if      (Xlin > 0.15 && Xlin <= 0.5)
        position[0].x -= 1;
      else if (Xlin > 0.5 && Xlin <= 0.9) 
        position[0].x -= 2;
      else if (Xlin > 0.9)
        position[0].x -= 3;

      if      (Xlin < -0.15 && Xlin >= -0.5)
        position[0].x += 1;
      else if (Xlin < -0.5 && Xlin >= -0.9) 
        position[0].x += 2;
      else if (Xlin < -0.9)
        position[0].x += 3;
      
      if      (Ylin > 0.15 && Ylin <= 0.5)
        position[0].y += 1;
      else if (Ylin > 0.5 && Ylin <= 0.9) 
        position[0].y += 2;
      else if (Ylin > 0.9)
        position[0].y += 3;

      if      (Ylin < -0.15 && Ylin >= -0.5)
        position[0].y -= 1;
      else if (Ylin < -0.5 && Ylin >= -0.9) 
        position[0].y -= 2;
      else if (Ylin < -0.9)
        position[0].y -= 3; 

      // stay in boundaries
      if (position[0].x < 0) 
        position[0].x = 0;
      else if (position[0].x > SCREEN_WIDTH)
        position[0].x = SCREEN_WIDTH;
      if (position[0].y < 0) 
        position[0].y = 0;
      else if (position[0].y > SCREEN_HEIGHT)
        position[0].y = SCREEN_HEIGHT;

      // delay to be calibrated for velocity of movement
      delay(10);
    }
  }

  // save total time
  t_tot = (millis() - t_tot)/1000;

  // calculate accuracy as the ratio between maximum in-range distance (as if every hit was at the bound of the target) and actual total distance
  dist = 1-dist_tot/(20.0*d_range);

  if (t_tot < 39)
    score = 100.0*abs(dist) * (40.0-t_tot);
  else
    score = 50.0*abs(dist);

  Serial.println(dist_tot);
  Serial.println(dist);

  // stats
  display.clearDisplay();
  display.setCursor(8, 0);  display.print("Stats");
  display.drawFastHLine(8, 8, SCREEN_WIDTH-15, 1);
  display.setCursor(8, 14); display.print("total time = "); display.print(t_tot);     display.print("s");
  display.setCursor(8, 32); display.print("accuracy = ");   display.print(100*dist);  display.print("%");
  display.setCursor(8, 50); display.print("score = ");      display.print(score);
  display.display();

  delay(1000);

  gotomenu = true;

  while(sw_press)
    delay(1);
}

#if (MULTIPLAY) // --------------- MULTIPLAYER GAMES ---------------

/*---------------------
  |   BLE functions   |
  --------------------- */
  #if (!BT_CLASSIC)
    // BLE server class
    class MyServerCallbacks : public BLEServerCallbacks {
      void onConnect(BLEServer *pServer) {
        deviceConnected = true;
      };

      void onDisconnect(BLEServer *pServer) {
        deviceConnected = false;
      }
    };

    // BLE client class
    class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
      void onResult(BLEAdvertisedDevice advertisedDevice) {
        Serial.print("BLE Advertised Device found: ");
        Serial.println(advertisedDevice.toString().c_str());

        // We have found a device, let us now see if it contains the service we are looking for.
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

          BLEDevice::getScan()->stop();
          myDevice = new BLEAdvertisedDevice(advertisedDevice);
        }  // Found our server
      }  
    };  

    class MyClientCallback : public BLEClientCallbacks {
      void onConnect(BLEClient *pclient) {
        Serial.printf("Connecting to server\n");
      }

      void onDisconnect(BLEClient *pclient) {
        connected = false;
        Serial.println("Disconnecting");
      }
    };

    // Callback class to handle writes from the client
    class MyCallbacks : public BLECharacteristicCallbacks {
      void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue();
        
        received_value = value.toFloat()/10.0;
        notified = true;
      }
    };

    // Callback class to handle writes from the server
    static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
      uint32_t int_in = pData[0];
      for (int i = 1; i < length; i++)
        int_in = int_in | (pData[i] << i*8);

      received_value = (double)int_in / 10.0;
      notified = true;
    }

  bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());

    BLEClient *pClient = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");
    pClient->setMTU(517);  //set client to request maximum MTU from server (default is 23 otherwise)

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    if (pRemoteCharacteristic->canNotify()) {
      pRemoteCharacteristic->registerForNotify(notifyCallback);
    }

    connected = true;
    return true;
  }

#endif


/*---------------------
  |       PONG        |
  --------------------- */
void pong_game() {
  // print game name
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(1);
  display.setCursor(38, 8); display.println("PONG");
  display.display();

  // delay some time 
  delay(500);

  Serial.printf("\n\n--------------------------\n\n");  // DEBUG

  // select between master and slave
  selected  = 1;
  gotomenu  = false;
  mnpress   = false;
  display.setTextSize(1);
  display.setCursor(8, 30); display.println(F("Bluetooth pairing"));
  while (!gotomenu) {
    readJoystick();
    calculate_dir();

    if (dir == 1)
      selected--;
    else if (dir == 3)
      selected++;

    selected = (selected + 2) % 2;

    if (mnpress) {
      if (selected == 1) 
        is_master = true;
      else
        is_master = false;

      break;
    }
    else {
      display.setCursor(0, 40); 

      if (selected == 1) {
        display.setTextColor(SSD1306_WHITE); display.print(" ");
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.println(" Player 1 ");
        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        display.print("  Player 2  ");
      }
      else {
        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        display.println("  Player 1 ");
        display.print(" ");
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.print(" Player 2  ");
      }
    }

    display.display();
    delay(t_sample_menu);
  }
  display.setTextColor(SSD1306_WHITE);

  if (gotomenu) {  // safe exit
    Serial.printf("Returning to menu");
    return;
  }

  // pair bluetooths
  if(initialize_BT()) {       // succesfull pairing
    display.clearDisplay();
    display.setCursor(14, 10); display.println(F("Bluetooth paired"));
    display.setCursor(14, 18); display.println(F("  succesfully"));
    display.display();

    delay(2000);
    
    alive = true;
  } else {                    // could not initiate bluetooth communication
  
#if (BT_CLASSIC)  /* -------------- BT CLASSIC --------------- */
    SerialBT.disconnect();
#else             /* ----------------- BLE ------------------- */
    if (is_master) {
      BLEDevice::getAdvertising()->stop();
    }

    BLEDevice::deinit();
#endif            /* ---------------- END BT ----------------- */

    display.clearDisplay();
    display.setCursor(10, 25); display.println(F(" Failed to connect"));
    display.setCursor(10, 38); display.println(F("press to go to menu"));
    display.display();

    while (sw_press) 
      delay(1);

    alive = false;
    gotomenu = true;
    ESP.restart();
  }

  // Game initialization
  double frRate = 60;          // milliseconds per frame

  score_1 = 0; score_2 = 0;  
  scored = true;

  // initial tile position
  t_y1 = (SCREEN_HEIGHT+v_bound)/2.0;
  t_y2 = (SCREEN_HEIGHT+v_bound)/2.0;
  pr_y1 = t_y1;
  pr_y2 = t_y2;

#if (BT_CLASSIC)  /* -------------- BT CLASSIC --------------- */
  SerialBT.println(t_y1);       // sends tile position
#else             /* ----------------- BLE ------------------- */
 int int_out = (t_y1*10.0);

  if (is_master) {
    pCharacteristic->setValue((uint8_t *)&int_out, 4);
    pCharacteristic->notify();
  } else {
    pRemoteCharacteristic->writeValue(String(int_out));
  }
#endif            /* ---------------- END BT ----------------- */

  // initial ball position
  ball_xy.x = SCREEN_WIDTH/2;
  ball_xy.y = (SCREEN_HEIGHT+v_bound)/2;
  pr_ball_xy.x = ball_xy.x; 
  pr_ball_xy.y = ball_xy.y;

  // initial ball velocity
  ball_v.y = 0.7*random(-XV, XV);   // y v is calculated randomicly
  if (is_master) {
    play_receive = true;        // master starts
    ball_v.x = -XV;             // initial ball velocity
  } else {
    play_receive = false;
    ball_v.x =  XV;             // initial ball velocity
  }

  display.clearDisplay();
  display.drawFastHLine(0, v_bound, SCREEN_WIDTH, 1);
  display.drawFastHLine(0, SCREEN_HEIGHT-1, SCREEN_WIDTH, 1);
  display.drawFastVLine(0, v_bound, SCREEN_HEIGHT-v_bound, 1);
  display.drawFastVLine(SCREEN_WIDTH-1, v_bound, SCREEN_HEIGHT-v_bound, 1);
  pong_printfield();

  // Game loop
  while (alive) {
    // BT communication: only the tile position is communicated. Every device calculates the game by itself      
    // syncronization is essential
#if (BT_CLASSIC)  /* -------------- BT CLASSIC --------------- */
    while (alive) {                         // waits until opponents tile position is received via BT
      delayMicroseconds(100);
      Serial.print("-");
      if (SerialBT.available())
        break;
    }
    Serial.print("BT\n");

    pr_y2 = t_y2;                           // saves previous tile position
    t_y2 = (SerialBT.readStringUntil(' ')).toFloat();
#else             /* ----------------- BLE ------------------ */
    while (alive) {                         // waits until opponents tile position is received via BT
      delayMicroseconds(100);
      Serial.print("-");
      if (notified)
        break;
    }
    Serial.print("BLE\n");

    pr_y2 = t_y2;                           // saves previous tile position
    t_y2 = received_value;
    notified = false;
    Serial.printf("tile sent %.1f tile received %.1f\n", t_y1, t_y2);
#endif            /* ---------------- END BT ----------------- */

    t = millis(); // start the frame rate counter

    // read analog input
    Yval = analogRead(AI2);                 // read the y axis of the joystick
    Ylin = sqrt(Yval/ya - ye) - yd;         // and linearize it

    Dt_y1 = t_y1 - pr_y1;                   // stores in a variable the previous tile difference
    pr_y1 = t_y1;                           // saves previous tile position

    if (fabs(Ylin) > 0.04) {                // noise cancellation
      Ylin = SIGN(Ylin)*pow(fabs(Ylin), 1.0/3.0); // increase sensibility for lower values
      t_y1 += max_t_v*frRate*Ylin;          // increment the tile position
    }

#if (!BT_CLASSIC) // for BLE tile pos is rounded to 2nd decimal place
    t_y1 = ROUND_TO_1_DECIMALS(t_y1);
#endif // -----------------------------------------------------------

    if (t_y1 < v_bound + tile_H/2 + 1)      // check if the tile is not over the border
      t_y1 = v_bound + tile_H/2 + 1;
    else if (t_y1 > SCREEN_HEIGHT - tile_H/2 - 2)
      t_y1 = SCREEN_HEIGHT - tile_H/2 - 2;
    
    // Tile position communication 
#if (BT_CLASSIC)  /* -------------- BT CLASSIC --------------- */
      SerialBT.printf("%f ", t_y1);           // sends tile position giving a lot of time to the message to be sent
#else             /* ----------------- BLE ------------------- */
      int int_out = (t_y1*10.0);

      if (is_master) {
        pCharacteristic->setValue((uint8_t *)&int_out, 4);
        pCharacteristic->notify();
      } else {
        pRemoteCharacteristic->writeValue(String(int_out));
      }
#endif            /* ---------------- END BT  ---------------- */

    pr_ball_xy.x = ball_xy.x;               // saves the current ball position 
    pr_ball_xy.y = ball_xy.y;

    ball_xy.x += ball_v.x*frRate/1000.0;    // calculates the new ball position
    ball_xy.y += ball_v.y*frRate/1000.0;

    // check if it hits the border
    if (ball_xy.y < v_bound + bR + 1) { 
      ball_xy.y += 2.0*(v_bound + bR + 1 - ball_xy.y);
      ball_v.y *= -1;
    }
    else if (ball_xy.y > SCREEN_HEIGHT - bR - 1) {
      ball_xy.y -= 2.0*(ball_xy.y - (SCREEN_HEIGHT - bR - 1));
      ball_v.y *= -1;
    }

    // check if it hit the tile or if it scored
    if (ball_xy.x <= bord_W + tile_W) {                   // I am receiving the ball
      if (fabs(pr_y1-ball_xy.y) < tile_H/2 + 1) {         // bounce
        ball_xy.x += 2.0*(bord_W + tile_W - ball_xy.x) ;  // sets it to exact bouncing position
        ball_v.x *= -1.0;

        // calculate new vertical velocity
        ball_v.y += Dt_y1*180.0/frRate;

        if (fabs(ball_v.y) > MAX_XV)
          ball_v.y = ball_v.y > 0 ? MAX_XV : -MAX_XV; 
      }
      else {  // opponent scored
        scored = true;
        play_receive = false;
        score_2++;
      }
    }
    else if ((SCREEN_WIDTH - ball_xy.x) <= bord_W + tile_W) { // opponent is receiving the ball
      if (fabs(t_y2-ball_xy.y) < tile_H/2 + 1) {              // bounce
        ball_xy.x -= 2.0*(ball_xy.x - (SCREEN_WIDTH - bord_W - tile_W)) ;  // sets it to exact bouncing position
        ball_v.x *= -1.0;

        // calculate new vertical velocity
        ball_v.y += (t_y2-pr_y2)*180.0/frRate;

        if (fabs(ball_v.y) > MAX_XV) {
          ball_v.y = ball_v.y > 0 ? MAX_XV : -MAX_XV; 
        }
      }
      else {  // I scored
        scored = true;
        play_receive = true;
        score_1++;
      }
    }
    
    pong_printfield();  // print field

    if (scored) {
      if (score_1 > 9 || score_2 > 9) { // check if game ended
        alive     = false;
        gotomenu  = true;

        display.setTextSize(2);     
        display.setTextColor(SSD1306_WHITE);
        display.fillRect(SCREEN_WIDTH/2-41, SCREEN_HEIGHT/2-6, 70, 18, 0);

        if (score_1 > 9) {
          display.setCursor(SCREEN_WIDTH/2-40, SCREEN_HEIGHT/2-5); display.println("YOU WIN");
        } else {
          display.setCursor(SCREEN_WIDTH/2-44, SCREEN_HEIGHT/2-5); display.println("YOU LOST");
        }

        display.display();

        while(sw_press)
          delay(1);
      }
      else { // game is still on: starts back towards who scored
        pr_ball_xy.x = ball_xy.x;               // saves the current ball position 
        pr_ball_xy.y = ball_xy.y;
        ball_xy.x = SCREEN_WIDTH/2;
        ball_xy.y = (SCREEN_HEIGHT+v_bound)/2;
        pr_y1 = t_y1;
        pr_y2 = t_y2;
        t_y1 = (SCREEN_HEIGHT+v_bound)/2.0;
        t_y2 = (SCREEN_HEIGHT+v_bound)/2.0;

        delay(1000); // check
        pong_printfield();
        delay(1000);

        if (play_receive) {
          ball_v.x = -XV;             // initial ball velocity
        } else {
          ball_v.x =  XV;
        }
        
        ball_v.y = 0.7*random(-XV, XV);   // y v is calculated randomicly

        scored = false;
      }
    }

    while (millis() - t < frRate) {
      delay(1);
      Serial.print("-"); // DEBUG
    }
    Serial.println("done"); // DEBUG
  }

  // at the moment I have no better alternative to reset the BT classic comm than to restart the devices
  // I should fix this
#if (BT_CLASSIC)    /* -------------- BT CLASSIC --------------- */
  SerialBT.end();
  ESP.restart();
#else               /* ----------------- BLE ------------------- */
  if (is_master)
    BLEDevice::getAdvertising()->stop();

  BLEDevice::deinit();
#endif              /* ---------------- END BT ----------------- */

  return;
}

#if (BT_CLASSIC)  /* -------------- BT CLASSIC --------------- */
  bool initialize_BT() {
    bool success_pairing = false;
    int new_seed;
    display.clearDisplay();
    display.setCursor(12, 10); display.println(F("Bluetooth pairing"));

    if (is_master) {
      // Start Bluetooth in master mode
      SerialBT.begin("ESP32_Master", true);  // true enables master mode
      SerialBT.deleteAllBondedDevices();

      display.setCursor(2, 35); display.println(F("Press when player 2")); 
      display.setCursor(14, 43); display.println(F("is ready to pair")); 
      display.display();

      while (sw_press)
        delay(1);
      
      if (SerialBT.connect(slaveMACAddress)) {  // connect to the server
        Serial.println("Connected to Server!");
        success_pairing = true;

        new_seed = random(1000);
        SerialBT.println(new_seed);             // send new seed to slave, the game can start
        randomSeed(new_seed);

        delay(1200);
      } else {
        Serial.println("Failed to connect to Server.");
        success_pairing = false;
      }
    }
    else {  
      slaveMACAddress[5] -= 2;
      esp_base_mac_addr_set(&slaveMACAddress[0]);
      slaveMACAddress[5] += 2;

      display.setCursor(24, 39); display.println(F("Ready to pair")); 
      display.display();

      SerialBT.begin("ESP32_Slave", false);  // start bluetooth in slave mode
      Serial.println("Bluetooth Server Started. Waiting for clients...");

      alive = true;
      while (alive) {
        if (SerialBT.available()) {
          delay(200);
          new_seed = (SerialBT.readString()).toInt();
          Serial.printf("new seed is %i", new_seed);
          randomSeed(new_seed);
          success_pairing = true;

          break;
        }
        delay(1);
      }
    }

    return success_pairing;
  }
#else   /* -------------- BLE --------------- */
  bool initialize_BT() {
    bool success_pairing = false;
    int new_seed;
    display.clearDisplay();
    display.setCursor(12, 10); display.println(F("Bluetooth pairing"));

    if (is_master) {
      Serial.printf("Initializing BLE server\n");
       // Create the BLE Device
      BLEDevice::init("ESP32");

      // Create the BLE Server
      pServer = BLEDevice::createServer();
      pServer->setCallbacks(new MyServerCallbacks());

      // Create the BLE Service
      BLEService *pService = pServer->createService(SERVICE_UUID);

      // Create a BLE Characteristic
      pCharacteristic = pService->createCharacteristic( 
                          CHARACTERISTIC_UUID,
                          BLECharacteristic::PROPERTY_READ    | 
                          BLECharacteristic::PROPERTY_WRITE   | 
                          BLECharacteristic::PROPERTY_NOTIFY  | 
                          BLECharacteristic::PROPERTY_INDICATE
      );

      // Set the callback to handle writes
      pCharacteristic->setCallbacks(new MyCallbacks());

      // Start the service
      pService->start();

      // Start advertising
      BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
      pAdvertising->addServiceUUID(SERVICE_UUID);
      pAdvertising->setScanResponse(false);
      pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
      BLEDevice::startAdvertising();
      Serial.println("Waiting a client connection to notify...");
      pCharacteristic->notify();

      display.setCursor(8, 35); display.println(F("Waiting for player")); 
      display.setCursor(8, 45); display.println(F("   2 to connect")); 
      display.display();

      notified = false;
      alive = true;

      while (!deviceConnected && alive)
        delay(1);
      success_pairing = deviceConnected;

      if (success_pairing) {
        Serial.printf("Client successfully connected to server\n");

        new_seed = random(1000);

        delay(1000);
      
        int send_seed = new_seed*10;
        pCharacteristic->setValue((uint8_t *)&send_seed, 4);
        pCharacteristic->notify();

        randomSeed(new_seed);
        Serial.printf("new seed = %i\n", new_seed);

        delay(30);
      } 
    }
    else {
      Serial.printf("Initializing BLE client\n");
      BLEDevice::init("");

      BLEScan *pBLEScan = BLEDevice::getScan();
      pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
      pBLEScan->setInterval(1349);
      pBLEScan->setWindow(449);
      pBLEScan->setActiveScan(true);
      pBLEScan->start(5, false);
      
      notified = false;

      display.setCursor(8, 35); display.println(F("Press when player 1")); 
      display.setCursor(14, 45); display.println(F("is ready to pair")); 
      display.display();

      while (sw_press)
        delay(1);

      if (connectToServer()) {
        success_pairing = true;
        Serial.println("We are now connected to the BLE Server.");

        while (!notified) { // waits for sincronization
          Serial.printf("-");   
          delay(1);
        } 
        Serial.printf("\n");   

        new_seed = received_value;
        notified = false;
        randomSeed(new_seed);
        Serial.printf("new seed = %i\n", new_seed);
      } else {
        success_pairing = false;
        Serial.println("We have failed to connect to the server");
      }
    }

    return success_pairing;
  }
#endif /* -------------- END BT --------------- */


void pong_printfield() {
  // draw score if scored
  if (scored) {
    display.setTextSize(1);     
    display.fillRect(50, 1, 32, 8, 0);  // deletes previous score
    if (score_1 >= 10)
      display.setCursor(44, 1); 
    else
      display.setCursor(50, 1); 
    display.print(score_1); display.print(" : "); display.print(score_2);
  }

  // delete previous tiles and ball by drawing with black on the previous positions
  display.fillCircle(pr_ball_xy.x, pr_ball_xy.y, bR, 0);
  display.fillRect(bord_W, pr_y1-tile_H/2, tile_W, tile_H, 0);
  display.fillRect(SCREEN_WIDTH-tile_W-bord_W, pr_y2-tile_H/2, tile_W, tile_H, 0);

  // draw new tiles and ball positions
  display.fillCircle(ball_xy.x, ball_xy.y, bR, 1);
  display.fillRect(bord_W, t_y1-tile_H/2, tile_W, tile_H, 1);
  display.fillRect(SCREEN_WIDTH-tile_W-bord_W, t_y2-tile_H/2, tile_W, tile_H, 1);

  // draw dotted line
  #define DV 4
  for (int l = 0; v_bound+2*l*DV < 64; l++)
    display.drawFastVLine(SCREEN_WIDTH/2, v_bound+2*l*DV, DV, 1);

  display.display();
}

#endif // ----------------------------------------------------------


/*-----------------------
  |  General functions  |
  ----------------------- */
void calibrate() {
  double x1, x2, x3;
  double y1, y2, y3;

  double xb, xc;
  double yb, yc;
  
  // starting frame 
  display.clearDisplay();
  display.setTextColor(1);
  display.setTextSize(2);
  display.setCursor(6,9); display.print(F("Calibrate"));
  display.setCursor(7,23); display.print(F("procedure")); 
  display.display();

  delay(1000);

  display.setTextSize(1);
  display.setCursor(20,48); display.println(F("click to start"));
  display.display();

  while(sw_press)
    delay(1);

  // MIDDLE
  display.clearDisplay();
  display.setCursor(10,19); display.print(F("leave the joystick")); 
  display.drawCircle(SCREEN_WIDTH/2, SCREEN_HEIGHT/2, 4, 1);
  display.setCursor(10,38); display.print(F("  in the middle   ")); 
  display.display();

  while (abs(analogRead(AI1) - 2048) > 500 || abs(analogRead(AI2) - 2048) > 500)
    delay(10);

  delay(1500);
  x2 = analogRead(AI1);
  y2 = analogRead(AI2);

  Serial.print("x2 = "); Serial.print(x2); Serial.print(" y2 = "); Serial.println(y2);

  // UPPER
  display.clearDisplay();
  display.setCursor(15,SCREEN_HEIGHT/2); display.print(F("go to top right")); 
  display.drawLine(SCREEN_WIDTH/2+10, SCREEN_HEIGHT/2-6, SCREEN_WIDTH-10, 10, 1);
  display.drawLine(SCREEN_WIDTH-10, 10, SCREEN_WIDTH-9, 23, 1);
  display.drawLine(SCREEN_WIDTH-10, 10, SCREEN_WIDTH-21, 7, 1);
  display.display();

  while (analogRead(AI1) > 500 || analogRead(AI2) > 500)
    delay(10);

  delay(1000);
  x1 = analogRead(AI1);
  y1 = analogRead(AI2);

  Serial.print("x1 = "); Serial.print(x1); Serial.print(" y1 = "); Serial.println(y1);

  // LOWER
  display.clearDisplay();
  display.setCursor(14,SCREEN_HEIGHT/2); display.print(F("go to bottom left")); 
  display.drawLine(SCREEN_WIDTH/2-15, SCREEN_HEIGHT/2+6, 10, SCREEN_HEIGHT-10, 1);
  display.drawLine(10, SCREEN_HEIGHT-10, 9, SCREEN_HEIGHT-23, 1);
  display.drawLine(10, SCREEN_HEIGHT-10, 21, SCREEN_HEIGHT-7, 1);
  display.display();

  while (analogRead(AI1) < 3600 || analogRead(AI2) < 3600)
    delay(10);

  delay(1000);
  x3 = analogRead(AI1);
  y3 = analogRead(AI2);

  Serial.print("x3 = "); Serial.print(x3); Serial.print(" y3 = "); Serial.println(y3);

  // UPPER
  display.clearDisplay();                               
  display.setCursor(12,SCREEN_HEIGHT/2); display.print(F("calibration completed")); 
  display.display();

  delay(1000);

  display.setTextSize(1);
  display.setCursor(20,48); display.println(F("press to return"));
  display.setCursor(24,54); display.println(F("to menu"));
  display.display();

  // calculate parabola parameters
  xa = (x1+x3)/2 - x2;
  xb = (x3-x1)/2;
  xc = x2;
  xd = xb/(2*xa);
  xe = xc/xa - xd*xd;
  
  ya = (y1+y3)/2 - y2;
  yb = (y3-y1)/2;
  yc = y2;
  yd = yb/(2*ya);
  ye = yc/ya - yd*yd;

  gotomenu = true;
  delay(500);

  while(sw_press)
    delay(1);
}

void readJoystick() {   // read joystick values

  double Xval = analogRead(AI1);
  double Yval = analogRead(AI2);

  Xlin = sqrt(Xval/xa - xe) - xd;
  Ylin = sqrt(Yval/ya - ye) - yd;
  
  //Serial.print(Xval); Serial.print(", "); Serial.println(Yval);
  //Serial.print(Xlin); Serial.print(", "); Serial.println(Ylin);

  magn  = sqrt(Xlin*Xlin + Ylin*Ylin);
  theta = atan(Ylin/Xlin)*180.0 / 3.14;

  if (Xlin < 0)  theta += 180.0;
  if (theta < 0) theta += 360.0;

  //Serial.print("magn: "); Serial.print(magn); Serial.print("; theta: "); Serial.println(theta);
}

void calculate_dir() {
  if (magn > 0.2) {
    if (theta > 45.0 && theta <= 135.0)
      dir = 3;
    else if (theta > 135.0 && theta <= 225.0)
      dir = 2;
    else if (theta > 225.0 && theta <= 315.0)
      dir = 1;
    else if (theta <= 45.0 || theta > 315.0)
      dir = 4;
  } 
  else
    dir = 0;
}


void ICACHE_RAM_ATTR sw_activity (void) {   // deals switch interrupt 
  if (sw_press && millis() - t_release > t_bounce) {
    sw_press  = false;
    mnpress   = true;
    t_press   = millis();
    t_release = millis();
    // Serial.println("Pressed");
  }
  else if (millis() - t_press > t_bounce) { 
    sw_press  = true;
    sw_rel    = true;
    t_release = millis();
    // Serial.print("Released after ");
    // Serial.print(millis() - t_press);
    // Serial.println(" ms");

    if (millis() - t_press > t_menu) {
      // Serial.println("opens menu");
      gotomenu = true;
      alive    = false;
    }

    t_press   = millis();
  }
}
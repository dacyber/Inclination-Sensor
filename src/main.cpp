
#include <Arduino.h> //Important for platformio to include the default Function for Arudino
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h> //by Neal Horman
#include <Adafruit_SSD1306.h> //by Adafruit Industries
#include <MPU6050.h> //by Jeff Rowberg
#include <I2Cdev.h> //by Jeff Rowberg

const unsigned char bikeFront [] PROGMEM = {
    0x00, 0x00, 0x00, 0x30, 0x00, 0x0C, 0x38, 0xFF, 0x1C, 0x0D, 0xFF, 0xB0, 0x07, 0xFF, 0xE0, 0x07,
    0xFF, 0xE0, 0x0F, 0xFF, 0xF0, 0x1F, 0xFF, 0xF8, 0x3F, 0xFF, 0xFC, 0x6F, 0xFF, 0xF6, 0x0E, 0x00,
    0x70, 0x0F, 0x00, 0xF0, 0x0F, 0x81, 0xF0, 0x0F, 0xFF, 0xF0, 0x1F, 0xFF, 0xF8, 0x0F, 0xFF, 0xF0,
    0x0F, 0xFF, 0xF0, 0x07, 0xFF, 0xE0, 0x03, 0xFF, 0xC0, 0x01, 0xFF, 0x80, 0x00, 0xFF, 0x00, 0x00,
    0xFF, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x7E,
    0x00, 0x00, 0x7E, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00,
};

/*
 * Data for display
 */
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
int MAX_INCL_RESET_BUTTON = 8;
int DISPLAY_CHANGE_BUTTON = 12;
int max_incl_button_state = 0;
int display_button_state = 0;
int display_counter = 0;
int16_t maxInclLeft = 0;
int16_t maxInclRight = 0;

/*
 * Data for MPU6050
 */
int16_t axis_X,axis_Y,axis_Z;
const int MPU_addr=0x68;
MPU6050 sensor(MPU_addr);
int minVal=265;
int maxVal=402;
double x,y,z;

/*
 * PIN-Settings for LED´s
 * PIN-Settings for Brightness
 */
int BRIGHTNESS_BUTTON = 2;
int RIGHT_RED_LED = 3;
int RIGHT_YELLOW_LED = 5;
int GREEN_LED = 6;
int LEFT_YELLOW_LED = 9;
int LEFT_RED_LED = 10;
byte brightness = 150;
int brightness_Counter = 4;
int brightness_button_state = LOW;

/*
 * Initialize the OLED-Display
 */
void initializeDisplay(){
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
}

/*
 * Initialize the MPU6050 Sensor
 */
void initializeMPU(){
    Serial.println("Initializing…");
    sensor.initialize();
    Serial.println("Testing device connections…");
    Serial.println(sensor.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

/*
 * Shuts off all LED´s
 */
void shutOffLED(){
    analogWrite(RIGHT_RED_LED,0);
    analogWrite(RIGHT_YELLOW_LED,0);
    analogWrite(GREEN_LED,0);
    analogWrite(LEFT_YELLOW_LED,0);
    analogWrite(LEFT_RED_LED,0);
}

/*
 * Only a initializing Circuit of the LED´s
 */
void initCircuitLED(){
    analogWrite(RIGHT_RED_LED,brightness);
    delay(50);
    analogWrite(RIGHT_RED_LED,0);
    delay(50);
    analogWrite(RIGHT_YELLOW_LED,brightness);
    delay(50);
    analogWrite(RIGHT_YELLOW_LED,0);
    delay(50);
    analogWrite(GREEN_LED,brightness);
    delay(50);
    analogWrite(GREEN_LED,0);
    delay(50);
    analogWrite(LEFT_YELLOW_LED,brightness);
    delay(50);
    analogWrite(LEFT_YELLOW_LED,0);
    delay(50);
    analogWrite(LEFT_RED_LED,brightness);
    delay(50);
    analogWrite(LEFT_RED_LED,0);
    delay(50);
}

/*
 * Function that shows Boot-Logo
 */
void showBootLogo(){
    display.clearDisplay();
	display.setTextColor(WHITE);
	display.setCursor(1,1);
	display.println("Incl -");
	display.setCursor(display.width()-40,1);
	display.println("Sensor");
	display.drawBitmap((display.width()/2)-12, 1, bikeFront, 24, 29, WHITE);
	display.setCursor(display.width()-35, display.height()-10);
	display.println("v0.1");
	display.display();
    delay(2500);
    display.clearDisplay();
}

/*
 * Initialize the PinMode for the Buttons
 */
void initTaster(){
    pinMode(BRIGHTNESS_BUTTON, INPUT);
    pinMode(MAX_INCL_RESET_BUTTON, INPUT);
    pinMode(DISPLAY_CHANGE_BUTTON, INPUT);
}

void setup(){
    Wire.begin();
    Serial.begin(115200);
    initializeMPU();
    initializeDisplay();
    showBootLogo();
    initCircuitLED();
    initTaster();
}

/*
 * Function that read Acc-Data from MPU6050 and calculate the x,y,z - Axis Angle
 */
void calculateSensorData(){
    axis_X = sensor.getAccelerationX();
    axis_Y = sensor.getAccelerationY();
    axis_Z = sensor.getAccelerationZ();

    int xAng = map(axis_X,minVal,maxVal,-90,90);
    int yAng = map(axis_Y,minVal,maxVal,-90,90);
    int zAng = map(axis_Z,minVal,maxVal,-90,90);


    x = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
    y = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
    z = RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
}

/*
 * Function to draw a Line with a specified angle.
 * Also includes the calculation from degree to radians.
 * It also considers the different resolution in x- and y-Direction
 *
 * @param angle
 */
void printAngleLine(float angle){ // angle in degree. Negative Values also allowed
    int16_t length_Line = 20;
    int16_t origin_X = display.width()/2;
    int16_t origin_Y = display.height()-1;
    int16_t calc_X = 0;
    int16_t calc_Y = 0;

    calc_X = ((sin(angle*(PI/180))*length_Line)*2);
    calc_Y = (cos(angle*(PI/180))*length_Line);

    display.drawLine(origin_X, origin_Y, origin_X + calc_X, origin_Y-calc_Y, WHITE);
}

/*
 * Function to show the current Inclination
 * @param angle
 */
void printCurrentInclination(int16_t angle){
    display.setTextSize(0);
    display.setTextColor(WHITE);
    display.setCursor((display.width()/2)-5, 0);
    display.println(angle);
}

/*
 * Function to shows the inclination to the left side
 * @param angle
 */
void printLeftInclination(int16_t angle){
    display.setTextSize(0);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println(angle);
}

/*
 * Function to shows the inclination to the right side
 * @param angle inclination
 */
void printRightInclination(int16_t angle){
    display.setTextSize(0);
    display.setTextColor(WHITE);
    display.setCursor(display.width()-20,0);
    display.println(angle);
}

/*
 * Function only shows a separation Line between the digit-part and the visual-part
 */
void printSeparationLine(){
    display.drawLine(0, 8, display.width(), 8, WHITE);
}

/*
 * Prints the current brightness at the point of change
 * @param str brightness Value to print
 */
void printBrightnessOnDisplay(int16_t str){
    display.clearDisplay();
    display.setTextSize(0);
    display.setTextColor(WHITE);
    display.setCursor(1,display.height()/2);
    display.print("Helligkeit : ");
    display.println(str);
    display.display();
    delay(350);
    display.clearDisplay();
}

/*
 * Changes the Brightness to spezified Values
 */
void changeBrightness(){
    switch(brightness_Counter){
        case(0): brightness = 0; printBrightnessOnDisplay(brightness); brightness_Counter++;break;
        case(1): brightness = 50; printBrightnessOnDisplay(brightness); brightness_Counter++;break;
        case(2): brightness = 100; printBrightnessOnDisplay(brightness); brightness_Counter++;break;
        case(3): brightness = 150; printBrightnessOnDisplay(brightness); brightness_Counter++;break;
        case(4): brightness = 200; printBrightnessOnDisplay(brightness); brightness_Counter++;break;
        case(5): brightness = 255; printBrightnessOnDisplay(brightness); brightness_Counter = 0;break;
    }
}

/*
 * Due to the given angle. This Function lights UP the specified LED
 * 0 to 20 = GREEN
 * 340 to 360 = Green
 * 21 to 30 = Yellow right
 * 31 to 179 = Red right
 * 330 to 339 = Yellow left
 * 180 to 329 = Red left
 *
 * @param angle current angle of Sensor
 */
void light_Up_LED(int16_t angle){
    if(angle>=0 && angle <=20){ //green
        analogWrite(RIGHT_RED_LED, 0);
        analogWrite(RIGHT_YELLOW_LED, 0);
        analogWrite(GREEN_LED, brightness);
        analogWrite(LEFT_YELLOW_LED, 0);
        analogWrite(LEFT_RED_LED, 0);
    }else
    if(angle>20 && angle < 30){ //yellow right
        analogWrite(RIGHT_RED_LED, 0);
        analogWrite(RIGHT_YELLOW_LED, brightness);
        analogWrite(GREEN_LED, 0);
        analogWrite(LEFT_YELLOW_LED, 0);
        analogWrite(LEFT_RED_LED, 0);
    }else
    if(angle >=30 && angle < 180){ //red right
        analogWrite(RIGHT_RED_LED, brightness);
        analogWrite(RIGHT_YELLOW_LED, 0);
        analogWrite(GREEN_LED, 0);
        analogWrite(LEFT_YELLOW_LED, 0);
        analogWrite(LEFT_RED_LED, 0);
    }else
    if(angle <=360 && angle >= 340){ //green
        analogWrite(RIGHT_RED_LED, 0);
        analogWrite(RIGHT_YELLOW_LED, 0);
        analogWrite(GREEN_LED, brightness);
        analogWrite(LEFT_YELLOW_LED, 0);
        analogWrite(LEFT_RED_LED, 0);
    }else
    if(angle < 340 && angle >330){ // yellow left
        analogWrite(RIGHT_RED_LED, 0);
        analogWrite(RIGHT_YELLOW_LED, 0);
        analogWrite(GREEN_LED, 0);
        analogWrite(LEFT_YELLOW_LED, brightness);
        analogWrite(LEFT_RED_LED, 0);
    }else
    if(angle < 330 && angle >=180){ // red left
        analogWrite(RIGHT_RED_LED, 0);
        analogWrite(RIGHT_YELLOW_LED, 0);
        analogWrite(GREEN_LED, 0);
        analogWrite(LEFT_YELLOW_LED, 0);
        analogWrite(LEFT_RED_LED, brightness);
    }
}

/*
 * Shows the first Display-Layout. Currently the Main Layout.
 */
void showDisplay1(){
    display.clearDisplay();
    printRightInclination(maxInclRight);
    printLeftInclination(maxInclLeft);
    printSeparationLine();
    int16_t valueToPrint = 0;
    light_Up_LED(x);
    if(x < 90 && x >= 0){ //Darstellung zwischen 0 und 90 Grad
        valueToPrint = x;
        if(x>maxInclRight){
            maxInclRight = x;
        }
        printCurrentInclination(valueToPrint);
        printAngleLine(valueToPrint);
    }else
    if(x >=270 && x < 360){ //Darstellung zwischen 270 und 360 Grad als 0 bis -90 Grad
        valueToPrint = x-360 ;
        if(-valueToPrint > maxInclLeft){
            maxInclLeft = -valueToPrint;
        }
        printCurrentInclination(-valueToPrint);
        printAngleLine(valueToPrint);
    }else
    if(x == 360){ // 360 Grad = 0 Grad
        printCurrentInclination(0);
        printAngleLine(0);
    }
    display.display();
    Serial.println(x);
    delay(250);
}

/*
 * Shows the Second Display Layout
 * The current Output is only transitionally
 */
void showDisplay2(){
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Inclination-Sensor");
    display.println("v0.1");
    display.println("Code written by");
    display.println("Simon Decker");
    display.display();
    delay(250);
}

/*
 * Due to the Value of DisplayChange Button, this function changes between the
 * different Display Layouts
 */
void switchDisplay(){
    switch(display_counter){
        case(0):showDisplay1();break;
        case(1):showDisplay2();break;
    }
}

/*
 * Read Brightness Button State
 */
void readBrightnessSwitchState(){
    brightness_button_state = digitalRead(BRIGHTNESS_BUTTON);
}

/*
 * Read MaxInclReset Button State
 */
void readMaxInclSwitchState(){
    max_incl_button_state = digitalRead(MAX_INCL_RESET_BUTTON);
}

/*
 * Read DisplayChange Button State
 */
void readDisplaySwitchState(){
    display_button_state = digitalRead(DISPLAY_CHANGE_BUTTON);
}

/*
 * Function that reset the max. Inclination Values to 0
 */
void resetMaxIncl(){
    maxInclLeft = 0;
    maxInclRight = 0;
}

void loop() {
    calculateSensorData();
    readBrightnessSwitchState();
    readMaxInclSwitchState();
    readDisplaySwitchState();

    if(brightness_button_state == HIGH){
        changeBrightness();
    }
    if(max_incl_button_state == HIGH){
        resetMaxIncl();
    }
    if(display_button_state == HIGH){
        display_counter++;
        if(display_counter > 1){
            display_counter = 0;
        }
        if(display_counter != 0){
            shutOffLED();
        }
    }
    switchDisplay();
}

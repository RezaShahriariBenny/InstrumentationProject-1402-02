#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

void setup() {
// set up the LCD's number of columns and rows:
lcd.begin(16, 2);
// Print a message to the LCD.
lcd.setCursor(1,0);
lcd.print("Ldr out= ");
lcd.setCursor(1,1);
lcd.print("The Electronics");
}

void loop ()
{
int ldr= analogRead(A0);

lcd.setCursor(10,0);
lcd.print(ldr);
}

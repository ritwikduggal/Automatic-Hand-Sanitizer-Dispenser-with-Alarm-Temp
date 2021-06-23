#include <Wire.h>
#include <LiquidCrystal.h>
#include <Adafruit_MLX90614.h>
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2, Contrast=150;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
#define echoPin 7
#define trigPin 8
#define pump 13
#define buzzerPin 10
int long duration;
int distance;
void setup() 
{
  analogWrite(6, Contrast);
  lcd.begin(16, 2);
  Serial.begin(9600);
  Serial.println("Adafruit MLX90614 test");  
  mlx.begin();  
  pinMode(echoPin,INPUT);
  pinMode(trigPin,OUTPUT);
  pinMode(pump, OUTPUT);
}
void loop() 
{
  digitalWrite(trigPin, LOW);
  delay(2); 
  digitalWrite(trigPin, HIGH);
  delay(10); 
  digitalWrite(trigPin, LOW);
  duration=pulseIn(echoPin, HIGH);
  distance=(duration*0.034/2);
  lcd.setCursor(0,0);
  lcd.print("Ambient="); lcd.print(mlx.readAmbientTempC()); lcd.print("*C");
  if(distance<=3)
  {
     delay(1000);
     lcd.setCursor(0, 1);
     lcd.print("Object="); lcd.print(mlx.readObjectTempF()); lcd.print("*F");
     if(mlx.readObjectTempF()>=98)
    {
      analogWrite(buzzerPin, 20); 
      digitalWrite(pump, HIGH);
      delay(1500);
      
      digitalWrite(pump, LOW);
      delay(1500);
    }
    else if(mlx.readObjectTempF()>=30 && mlx.readObjectTempF()<98)
    {
      digitalWrite(pump, HIGH);
      delay(1500);
      digitalWrite(pump, LOW); 
      delay(1500);
    }
  }
  delay(500);
}

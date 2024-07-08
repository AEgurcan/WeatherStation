// Pin Definitions
const int tempSensorPin = A0; // Temperature sensor (e.g., LM35)
const int photoResistorPin = A3; // Photoresistor for light detection (day/night)
const int forceSensorPin = A2;    // Force sensor for wind speed
const int ultrasonicTrigPin = 2;  // Ultrasonic sensor - Trigger pin
const int ultrasonicEchoPin = 3;  // Ultrasonic sensor - Echo pin
const int humidityPotPin = A1;    // Potentiometer for humidity


void setup() {
  Serial.begin(9600);

  // Ultrasonic Sensor Setup
  pinMode(ultrasonicTrigPin, OUTPUT);
  pinMode(ultrasonicEchoPin, INPUT);
}

void loop() {
  // Read temperature from the temperature sensor
  int adcValue = analogRead(tempSensorPin);
  float voltage = adcValue * (5.0 / 1024.0);
  float temperature = (voltage / 0.01) - 50; // Convert voltage to temperature in Celsius

  // Read wind speed from the force sensor
  int forceReading = analogRead(forceSensorPin);
  float windSpeed = map(forceReading, 0, 1023, 0, 50); // Mapping to km/h, simulating wind speed from 0 to 50 km/h

  // Read light levels for day/night simulation
  int lightReading = analogRead(photoResistorPin);
  float dayNightFactor = lightReading < 512 ? -3 : 0;

  // Read altitude from the ultrasonic sensor and adjust base pressure
  long duration, distance;
  digitalWrite(ultrasonicTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTrigPin, LOW);
  duration = pulseIn(ultrasonicEchoPin, HIGH);
  distance = duration * 0.034 / 2; // Speed of sound wave in air is 343 m/s, i.e., 0.034 cm/us

  // Calculate pressure based on altitude (Hypsometric formula simplified)
  float pressure = 1013.25 * pow((1 - (0.0065 * distance / (temperature + 273.15))), 5.257);

  // Adjust pressure for day/night
  pressure += dayNightFactor;

  // Read humidity from the potentiometer
  int humidityReading = analogRead(humidityPotPin);
  float humidity = map(humidityReading, 0, 1023, 0, 100); // Mapping to percentage

  // Output the readings
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println("%");

  Serial.print("Altitude: ");
  Serial.print(distance);
  Serial.println(" cm");

  Serial.print("Wind Speed: ");
  Serial.print(windSpeed);
  Serial.println(" km/h");

  Serial.println(); // Blank line for readability

  delay(2000); // Delay before taking the readings again
}

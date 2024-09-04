# Serial Komunikasi

Agar dapat menjalankan komunikasi antara microcontroller dengan PC, maka kita memerlukan sebuah library Serial yang ada pada Python maupun bahasa lain.  

```Python Code```
```py
import serial
import time

# Configure the serial port and baud rate
ser = serial.Serial('COM3', 9600, timeout=1)  # Replace 'COM3' with your Arduino's serial port

def send_data(data):
    ser.write(data.encode())  # Send data to Arduino
    time.sleep(1)  # Wait for Arduino to process the data
    response = ser.readline().decode('utf-8').strip()  # Read the response from Arduino
    return response

if __name__ == "__main__":
    while True:
        data_to_send = input("Enter data to send to Arduino: ")
        response = send_data(data_to_send + '\n')  # Append newline character
        print("Arduino response:", response)
```

```Arduino Code```
```ino
void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud rate
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); // Read the incoming data
    Serial.print("Received: ");
    Serial.println(data); // Send back the received data
  }
}
```
Dalam menggunakan serial komunikasi, antara microcontroller dengan PC harus memiliki baudrate yang sama. Pada contoh tersebut, digunakan baudrate sebesar 9600.  
Nah, itu tadi adalah contoh komunikasi ke Arduino. Komunikasi serial nantinya juga digunakan untuk komunikasi antara PC dan STM32.   

## Pengiriman Data

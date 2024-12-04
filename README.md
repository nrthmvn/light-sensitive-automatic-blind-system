# Light-Sensitive Automatic Blind System
  
A light-sensitive automatic blind system, programmed in C and based on the ATmega328P. Made as a mini-project for the course B31DD: Embedded Systems. An Arduino Uno R3 board was used to implement the system.

---

![Circuit](<./images/circuit.jpg>)

The system implemented on a breadboard

---

  
The system has three modes of operation:  
  
**Mode 0: Data**  
In data mode the light level is regularly transmitted via USART through the USB interface on the Arduino board to a serial monitor.  
  
**Mode 1: Automatic**  
In automatic mode the servo motor adjust to the ambient light level. Closing the blinds in bright light and opening them in low light.  
  
**Mode 2: Manual**  
In manual mode the blind is controlled by the potentiometer by the user.

---

  
The left button is the power button which toggles switches between on and off. The right button switches modes, the system initializes to mode 0. The button increments the mode and wraps around from 2 back to 0.

---

![TinkerCAD](<./images/tinkercad.png>)

A TinkerCAD model can be found [here](<https://www.tinkercad.com/things/5VIbyYFtjsR-light-sensitive-automatic-blind-controller?sharecode=qlA4HkZk9HUoG2sH1UysCUogZdNxzPJbQt4GY22zy7I>)  



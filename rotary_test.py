#!/usr/bin/env python
#
import pigpio
import time

class decoder:

   """Class to decode mechanical rotary encoder pulses."""

   def __init__(self, pi, gpioA, gpioB, callback):

      """
      Instantiate the class with the pi and gpios connected to
      rotary encoder contacts A and B.  The common contact
      should be connected to ground.  The callback is
      called when the rotary encoder is turned.  It takes
      one parameter which is +1 for clockwise and -1 for
      counterclockwise.
      """

      self.pi = pi
      self.gpioA = gpioA
      self.gpioB = gpioB
      self.callback = callback

      self.levA = 0
      self.levB = 0
      
      self.newState = 0
      self.lastState = 0
      #self.lastGpio = None
      
      self.count = 0

      self.pi.set_mode(gpioA, pigpio.INPUT)
      self.pi.set_mode(gpioB, pigpio.INPUT)

      self.pi.set_pull_up_down(gpioA, pigpio.PUD_UP)
      self.pi.set_pull_up_down(gpioB, pigpio.PUD_UP)

      self.cbA = self.pi.callback(gpioA, pigpio.EITHER_EDGE, self._pulse)
      self.cbB = self.pi.callback(gpioB, pigpio.EITHER_EDGE, self._pulse)

   def _pulse(self, gpio, level, tick):

      """
      Decode the rotary encoder pulse.

                   +---------+         +---------+      0
                   |         |         |         |
         A         |         |         |         |
                   |         |         |         |
         +---------+         +---------+         +----- 1

             +---------+         +---------+            0
             |         |         |         |
         B   |         |         |         |
             |         |         |         |
         ----+         +---------+         +---------+  1
      """

      if gpio == self.gpioA:
         self.levA = level
      else:
         self.levB = level;
      self.newState = (self.levA ^ self.levB) | self.levB << 1
      delta = (self.newState - self.lastState) % 4
      if delta == 1:
         self.count += 1
      elif delta == 3:
         self.count -= 1

      self.lastState = self.newState

   def cancel(self):

      """
      Cancel the rotary encoder decoder.
      """
      self.cbA.cancel()
      self.cbB.cancel()

def calibrate(pi, motor_r, motor_l):
   min_r = -1
   min_l = -1
   count = 0
   for i in range(100,255,5):
      msg = "PWM: "+str(i)
      print(msg)
      # 
      pi.set_PWM_dutycycle(motor_r[0], 0)   # Motor derecho
      pi.set_PWM_dutycycle(motor_r[1], i)
      pi.set_PWM_dutycycle(motor_l[0], i)   # Motor Izquierdo
      pi.set_PWM_dutycycle(motor_l[1], 0)
      decoder1 = rotary_encoder.decoder(pi, encoder_r[0] , encoder_r[1], callback)
      decoder2 = rotary_encoder.decoder(pi, encoder_l[0] , encoder_l[1], callback)
      time.sleep(2)
      if abs(decoder1.count) > 2000 and min_r == -1: 
         min_r = i
      elif abs(decoder1.count) < 2000 :
         min_r = -1
         count = -1
      if (decoder2.count) >= 2000 and min_l == -1:
         min_l = i
      elif abs(decoder2.count) < 2000 :
         min_l = -1
         count = -1
      count += 1
      
      # Apagar motores
      pi.set_PWM_dutycycle(motor_r[0],0)
      pi.set_PWM_dutycycle(motor_r[1],0)
      pi.set_PWM_dutycycle(motor_l[0],0)
      pi.set_PWM_dutycycle(motor_l[1],0)
      if count >= 3:
         break
      time.sleep(1)      
      decoder1.cancel()
      decoder2.cancel()

   return (min_l, min_r)
   #aux = (74.83 * 48)

#def findBetas()

def callback(way):
   global pos
   pos += way
   print("pos={}".format(pos))

if __name__ == "__main__":
   import time
   import pigpio
   import rotary_encoder
   
   motor_r = (19,26)    # Motor Derecho.
   motor_l = (16,20)    # Motor Izquierdo.
   encoder_r = (12,13)  # Encoder Derecho.
   encoder_l = (5,6)    # Encoder Izquierdo.

   pi = pigpio.pi()
   pi.set_mode(motor_r[0], pigpio.OUTPUT)
   pi.set_mode(motor_r[1], pigpio.OUTPUT)
   pi.set_mode(motor_l[0], pigpio.OUTPUT)
   pi.set_mode(motor_l[1], pigpio.OUTPUT)

   decoder1 = rotary_encoder.decoder(pi, encoder_r[0] , encoder_r[1], callback)
   decoder2 = rotary_encoder.decoder(pi, encoder_l[0] , encoder_l[1], callback)

   pi.set_PWM_dutycycle(motor_r[0], 0)   # Motor derecho
   pi.set_PWM_dutycycle(motor_r[1], 255)
   pi.set_PWM_dutycycle(motor_l[0], 255)   # Motor Izquierdo
   pi.set_PWM_dutycycle(motor_l[1], 0)

   time.sleep(4)

   print("Decoder1 = " + str(decoder1.count) + " Decoder2 = " + str(decoder2.count))
   pi.set_PWM_dutycycle(motor_r[0], 0)   # Motor derecho
   pi.set_PWM_dutycycle(motor_r[1], 0)
   pi.set_PWM_dutycycle(motor_l[0], 0)   # Motor Izquierdo
   pi.set_PWM_dutycycle(motor_l[1], 0)
'''
   # Variables
   pos = 0
   motor_r = (19,26)    # Motor Derecho.
   motor_l = (16,20)    # Motor Izquierdo.
   encoder_r = (12,13)  # Encoder Derecho.
   encoder_l = (5,6)    # Encoder Izquierdo.



   #PWM: 130 // 150 // 175
   pi = pigpio.pi()
   pi.set_mode(motor_r[0], pigpio.OUTPUT)
   pi.set_mode(motor_r[1], pigpio.OUTPUT)
   pi.set_mode(motor_l[0], pigpio.OUTPUT)
   pi.set_mode(motor_l[1], pigpio.OUTPUT)
   c = calibrate(pi, motor_r, motor_l)
   print("Calibration result:", c)
'''

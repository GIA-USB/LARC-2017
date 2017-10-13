#!/usr/bin/env python

import pigpio

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
      #print(delta)  

      self.lastState = self.newState
      '''  	
      if gpio != self.lastGpio: # debounce
         self.lastGpio = gpio

         if   gpio == self.gpioA and level == 1:
            if self.levB == 1:
               self.callback(1)
         elif gpio == self.gpioB and level == 1:
            if self.levA == 1:
               self.callback(-1)
      '''
   def cancel(self):

      """
      Cancel the rotary encoder decoder.
      """

      self.cbA.cancel()
      self.cbB.cancel()

if __name__ == "__main__":
   import time
   import pigpio
   import rotary_encoder

   pos = 0
   motor_r = (19,26)
   motor_l = (16,20)
   encoder_r = (12,13)
   encoder_l = (5,6)

   def callback(way):

      global pos

      pos += way

      print("pos={}".format(pos))

   pi = pigpio.pi()
   pi.set_mode(motor_r[0], pigpio.OUTPUT)
   pi.set_mode(motor_r[1], pigpio.OUTPUT)
   pi.set_mode(motor_l[0], pigpio.OUTPUT)
   pi.set_mode(motor_l[1], pigpio.OUTPUT)
   pi.set_PWM_dutycycle(motor_r[0], 0)
   pi.set_PWM_dutycycle(motor_r[1], 115)
   pi.set_PWM_dutycycle(motor_l[0], 115)
   pi.set_PWM_dutycycle(motor_l[1], 0)
   decoder1 = rotary_encoder.decoder(pi, encoder_r[0] , encoder_r[1], callback)
   decoder2 = rotary_encoder.decoder(pi, encoder_l[0] , encoder_l[1], callback)
   time.sleep(10)
   '''
   pi.set_PWM_dutycycle(5,0)
   pi.set_PWM_dutycycle(6,255)
   
   time.sleep(1)
   '''
   pi.set_PWM_dutycycle(motor_r[0],0)
   pi.set_PWM_dutycycle(motor_r[1],0)
   pi.set_PWM_dutycycle(motor_l[0],0)
   pi.set_PWM_dutycycle(motor_l[1],0)
   print("Der: " , decoder1.count)
   print("Izq: " , decoder2.count)
   aux = (74.83 * 48)
   decoder1.cancel()
   decoder2.cancel()
   pi.stop()

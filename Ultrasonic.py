import time
import pigpio 

class Ultrasonic():
	"""
	Class to read distance using a sonar ranger.

	Instantiate with the Pi, trigger GPIO, and echo GPIO.

	Trigger a reading with trigger().

	Wait long enough for the maximum echo time and get the
	reading in centimetres with read().   A reading of 999.9
	indicates no echo.

	When finished call cancel() to tidy up.
	"""
	
	def __init__(self, pi, trig, echo):
		self.pi = pi
		self.trig = trig
		self.distance = 999.9
		self.oneTick = None
		self.SOS = 340.29
		self.echo = echo
		
		if trig is not None:
			pi.set_mode(trig, pigpio.OUTPUT)
		pi.set_mode(echo, pigpio.INPUT)
		self._cb = pi.callback(echo, pigpio.EITHER_EDGE, self._cbf)

	def _cbf(self, gpio, level, tick):
		if level == 1:
			self.oneTick = tick
		else:
			if self.oneTick is not None:
				pingMicros = pigpio.tickDiff(self.oneTick, tick)
				self.distance = (pingMicros * self.SOS) / 2e4
				self.oneTick = None

	def trigger(self):
		self.distance = 999.9
		self.oneTick = None
		if self.trig is not None:
			self.pi.gpio_trigger(self.trig, 15) # 15 micros trigger pulse

	def read(self):
		return self.distance

	def cancel(self):
		self._cb.cancel()


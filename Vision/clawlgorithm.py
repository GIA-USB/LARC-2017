# Ubicar garra al principio del riel (A la derecha)

# Ubicar garra de frente.

# Comenzar a mover la garra y agarrar el primero que veo.

# Moverse hacia el frente una distancia D

# Cerrar garras

# Retroceder
from picamera.array import PiRGBArray
from picamera import PiCamera
import pigpio
import time
from findGlass import findGlassHole
from fixedMoves import *

global pi
pi = pigpio.pi()

# PINES
garra1   = 4
garra2   = 5
rotation = 6
riel 	 = 7

# PWM
const_abierto = 500		# Apertura de la garra.
const_cerrado = 1500
const_frente1 = 500		# Rotacion de la garra.
const_frente2 = 2500
const_derecha = 500		# Velocidad y sentido del riel.
const_izquier = 2500

# Tiempos
delay_findGlass = 6		# Numero de tiempo estimado para pasear todo el riel.

# CAMARA
camera = PiCamera()
camera.awb_mode = 'off'
# Start off with ridiculously low gains
rg, bg = (1.90, 1.60)
camera.awb_gains = (rg, bg)

# Movimiento Robot
pwm = 150

# Set de los motores.
pi.set_mode(rotation, pigpio.OUTPUT)
pi.set_mode(garra1, pigpio.OUTPUT)
pi.set_mode(garra2, pigpio.OUTPUT)

def moveServoInPin(pin, pwm):
	pi.set_servo_pulsewidth(pin,pwm)

def take_glass(garra, frente):
	# Start
	stop()
	moveServoInPin(riel, const_derecha)		# Comenzar movimiento.
	moveServoInPin(rotation, const_frente1)	# Orientar la garra al frente.

	# Busqueda
	moveServoInPin(riel, const_izquier) 			# Comenzar a moverse a la izquierda
	find = findGlassHole(camera, delay_findGlass)	# DETECTAR HUECO + stopServoInPin(riel)
	moveServoInPin(riel, 0)							# Detener el riel
	
	# Captura
	moveServoInPin(garra1, const_abierto)			# Abrir garra.
	# Validar con distancia del ultrasonido
	forward(pwm)									# Posicionarse en el vaso.

	########
	moveServoInPin(garra1, const_cerrado)			# Cerrar garra.
	#### Validar con distancia del ultrasonido 
	backward(pwm)									# retroceder
	stop()											# Detener protocolo.
	
take_glass(garra1,const_frente1)

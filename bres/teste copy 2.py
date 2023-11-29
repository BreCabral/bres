import RPi.GPIO as gpio
from RpiMotorLib import RpiMotorLib

# Configuração de IOs:
led = 17
potencia = 27
m1_step = 23
m1_dir = 24
m2_step = 20
m2_dir = 21
m3_step = 19
m3_dir = 16
m4_step = 5
m4_dir = 6
gpio.setmode(gpio.BCM)
gpio.setup(led, gpio.OUT)
gpio.setup(potencia, gpio.OUT)

def main(args=None):
    gpio.output(led, gpio.LOW)
    gpio.output(potencia, gpio.LOW)
    MotorBase = RpiMotorLib.A4988Nema(m1_dir, m1_step, (led,led,led), "DRV8825")
    MotorBase.motor_go(False,"1/4",200,0.005,True,0.05)
    MotorOmbro = RpiMotorLib.A4988Nema(m2_dir, m2_step, (led,led,led), "DRV8825")
    MotorOmbro.motor_go(False,"1/4",200,0.005,True,0.05)
    MotorBraco = RpiMotorLib.A4988Nema(m3_dir, m3_step, (led,led,led), "DRV8825")
    MotorBraco.motor_go(False,"1/4",200,0.005,True,0.05)
    MotorAntebraco = RpiMotorLib.A4988Nema(m4_dir, m4_step, (led,led,led), "DRV8825")
    MotorAntebraco.motor_go(False,"1/4",200,0.005,True,0.05)
    gpio.cleanup()
    

if __name__ == '__main__':
    main()
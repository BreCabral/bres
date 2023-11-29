import RPi.GPIO as gpio
import RpiMotorLib

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
gpio.setup(m1_step, gpio.OUT)
gpio.setup(m1_dir, gpio.OUT)
gpio.setup(m2_step, gpio.OUT)
gpio.setup(m2_dir, gpio.OUT)
gpio.setup(m3_step, gpio.OUT)
gpio.setup(m3_dir, gpio.OUT)
gpio.setup(m4_step, gpio.OUT)
gpio.setup(m4_dir, gpio.OUT)

class MotorHandle():

    def __init__(self, motor=0, qtd_passos=200):
        self.angulo_por_passo = 360/qtd_passos
        conectarGPIO(motor)

    def conectarGPIO(self, motor):
        if (motor == 0)
            return
        if (motor == 1)
            self.step = m1_step
            self.dir = m1_dir

    def rotacao(self, angulo):
        if (angulo == 0)
            return "sem movimento"
        else if (angulo > 0)
            direcao = 1
        else if (angulo < 0)
            direcao = 0
        qtd_passos_mover = abs(angulo)/self.angulo_por_passo
        for passo in qtd_passos_mover:
            gpio.output(self.step, gpio.HIGH)


def main(args=None):
    MotorBase = MotorHandle(1)
    MotorBase.rotacao(90)
    gpio.cleanup()
    mymotortest = RpiMotorLib.A4988Nema(direction, step, (21,21,21), "DRV8825")
    

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
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

gpio.output(potencia, gpio.HIGH)


class MotorHandle():

    def __init__(self, motor=1, angulo_por_passo=0.45, deslocamento_por_passo=0.01):
        self.angulo_por_passo = angulo_por_passo
        self.deslocamento_por_passo = deslocamento_por_passo
        self.step = m1_step
        self.dir = m1_dir
        self.conectarGPIO(motor)
        

    def conectarGPIO(self, motor):
        if (motor == 1):
            self.step = m1_step
            self.dir = m1_dir
        elif (motor == 2):
            self.step = m2_step
            self.dir = m2_dir
        elif (motor == 3):
            self.step = m3_step
            self.dir = m3_dir
        elif (motor == 4):
            self.step = m4_step
            self.dir = m4_dir
        else:
            raise IndexError('ERRO: Escolha um motor entre 1 a 4')
        self.motor_conectado = RpiMotorLib.A4988Nema(self.dir, self.step, (led,led,led), "DRV8825")

    def rotacao(self, angulo):
        if (abs(angulo) == 0):
            return 0
        elif (angulo > 0):
            direcao = True
            mul_dir = 1
        elif (angulo < 0):
            direcao = False
            mul_dir = -1
        qtd_passos_mover = int(abs(angulo)/self.angulo_por_passo)
        self.motor_conectado.motor_go(direcao,"1/4",qtd_passos_mover,0.005,False,0.05)
        return (qtd_passos_mover*self.angulo_por_passo*mul_dir)
    
    def translacao(self, deslocamento):
        if (deslocamento == 0):
            return 0
        elif (deslocamento > 0):
            direcao = True
        elif (deslocamento < 0):
            direcao = False
        qtd_passos_mover = int(abs(deslocamento)/self.deslocamento_por_passo)
        self.motor_conectado.motor_go(direcao,"1/4",qtd_passos_mover,0.005,True,0.05)
        return (qtd_passos_mover*self.deslocamento_por_passo)


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.motor1 = MotorHandle(motor=1, angulo_por_passo=(0.45/4))
        self.motor2 = MotorHandle(motor=2, deslocamento_por_passo=(0.01))
        self.motor3 = MotorHandle(motor=3, angulo_por_passo=(0.45/4))
        self.motor4 = MotorHandle(motor=4, angulo_por_passo=(0.45/4))

        self.m1_pos_goal = 0
        self.m2_pos_goal = 0
        self.m3_pos_goal = 0
        self.m4_pos_goal = 0

        self.m1_pos_atual = 0
        self.m2_pos_atual = 0
        self.m3_pos_atual = 0
        self.m4_pos_atual = 0

    def listener_callback(self, msg):

        self.m1_pos_goal = self.rad_to_deg(-1*msg.position[0])
        self.m2_pos_goal = self.m_to_mm(msg.position[1])
        self.m3_pos_goal = self.rad_to_deg(msg.position[2])
        self.m4_pos_goal = self.rad_to_deg(msg.position[3])

        diferenca_m1 = (self.m1_pos_goal-self.m1_pos_atual)
        diferenca_m2 = (self.m2_pos_goal-self.m2_pos_atual)
        diferenca_m3 = (self.m3_pos_goal-self.m3_pos_atual)
        diferenca_m4 = (self.m4_pos_goal-self.m4_pos_atual)

        self.m1_pos_atual += self.motor1.rotacao(diferenca_m1)
        self.m2_pos_atual += self.motor2.translacao(diferenca_m2)
        self.m3_pos_atual += self.motor3.rotacao(diferenca_m3)
        self.m4_pos_atual += self.motor4.rotacao(diferenca_m4)

    def rad_to_deg(self, rad):
        deg = (rad*180)/3.14159
        return deg
    
    def m_to_mm(self, m):
        mm = m*1000
        return mm


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
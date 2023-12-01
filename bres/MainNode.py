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

    def __init__(self, motor=1, angulo_por_passo=0.45):
        self.angulo_por_passo = angulo_por_passo
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
        if (angulo == 0):
            return print("sem movimentacao")
        elif (angulo > 0):
            direcao = True
        elif (angulo < 0):
            direcao = False
        qtd_passos_mover = int(abs(angulo)/self.angulo_por_passo)
        self.motor_conectado.motor_go(direcao,"1/4",200,0.005,True,0.05)
        return (qtd_passos_mover*self.angulo_por_passo)


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.m1_pos_goal = 0
        self.m2_pos_goal = 0
        self.m3_pos_goal = 0
        self.m4_pos_goal = 0

    def listener_callback(self, msg):
        self.m1_pos_goal = msg.position[0]
        self.m2_pos_goal = msg.position[1]
        self.m3_pos_goal = msg.position[2]
        self.m4_pos_goal = msg.position[3]


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
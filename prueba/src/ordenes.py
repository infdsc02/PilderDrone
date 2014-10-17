import roslib
roslib.load_manifest('prueba')
import sys
import rospy
from drone_controller import BasicDroneController
from std_msgs.msg import String


class Ordenes:

    def __init__(self):
        self.controller = BasicDroneController()
        self.girando = False
        self.despLatIzq = False
        self.despLatDer = False
        self.subiendoDer = False
        self.subiendoIzq = False
        self.bajandoDer = False
        self.bajandoIzq = False
        self.subiendo = False
        self.bajando = False

    def getStatus(self):
        return self.controller.status

    """
    def emergencia(self):
        self.controller.SendEmergency()

    def despega(self):
        self.controller.SendTakeoff()

    def aterriza(self):
        self.controller.SendLand()
    """


    def yawIzq(self):
        self.controller.SetCommand(0, 0, 0.25, 0)

    def yawDer(self):
        self.controller.SetCommand(0, 0, -0.25, 0)

    def paraYaw(self):
        self.controller.setYaw(0)

    def paraCentrar(self):
        self.controller.setRoll(0)
        self.controller.setZvelocity(0)

    def paraPitch(self):
        self.controller.setPitch(0)

    def estaGirando(self):
        return self.girando

    def gira(self):
        self.controller.setYaw(1)
        self.girando = True

    def paraGiro(self):
        self.controller.setYaw(0)
        self.girando = False

    def gira90Izq(self):
        rotZ = self.controller.getRotZ()
        rotZ_dest = rotZ + 90
        self.controller.writeLog('rotZ_ini ' + str(rotZ))
        self.controller.writeLog('rotZ_dest ' + str(rotZ_dest))
        self.girando = True
        while self.girando:
                rotZ = self.controller.getRotZ()
                if (rotZ < rotZ_dest):
                    if self.controller.getYawVelocity() <= 0:
                        self.controller.SetCommand(0, 0, 1, 0)

                else:
                    self.girando = False
                    self.controller.writeLog('YA LLEGAMOS')
                    self.controller.SetCommand(0, 0, 0, 0)

    def gira90Der(self):
        rotZ = self.controller.getRotZ()
        rotZ_dest = rotZ - 90
        self.controller.writeLog('rotZ_ini ' + str(rotZ))
        self.controller.writeLog('rotZ_dest ' + str(rotZ_dest))
        self.girando = True
        while self.girando:
                rotZ = self.controller.getRotZ()
                if (rotZ > rotZ_dest):
                    if self.controller.getYawVelocity() >= 0:
                        self.controller.SetCommand(0, 0, -1, 0)

                else:
                    self.girando = False
                    self.controller.writeLog('YA LLEGAMOS')
                    self.controller.SetCommand(0, 0, 0, 0)


    def gira180(self):
        rotZ = self.controller.getRotZ()
        rotZ_dest = rotZ + 160
        self.controller.writeLog('rotZ_ini ' + str(rotZ))
        self.controller.writeLog('rotZ_dest ' + str(rotZ_dest))
        self.girando = True
        while self.girando:
                rotZ = self.controller.getRotZ()
                if (rotZ < rotZ_dest):
                    if self.controller.getYawVelocity() <= 0:
                        self.controller.SetCommand(0, 0, 1, 0)
                else:
                    self.girando = False
                    self.controller.writeLog('YA LLEGAMOS')
                    self.controller.SetCommand(0, 0, 0, 0)

    def corregirYaw(self, angulo):
        rotZ = self.controller.getRotZ()
        rotZ_dest = rotZ + angulo
        self.controller.writeLog('rotZ_ini ' + str(rotZ))
        self.controller.writeLog('rotZ_dest ' + str(rotZ_dest))
        self.girando = True
        while self.girando:
            rotZ = self.controller.getRotZ()
            if angulo < 0:
                if (rotZ > rotZ_dest):
                    self.controller.writeLog('rotZ ' + str(rotZ))
                    #if self.controller.getYawVelocity() >= 0:
                        #self.controller.SetCommand(0, 0, -10, 0)
                else:
                    self.girando = False
                    self.controller.writeLog('YA LLEGAMOS')
                    #self.controller.SetCommand(0, 0, 0, 0)
            else:
                if (rotZ < rotZ_dest):
                    self.controller.writeLog('rotZ ' + str(rotZ))
                    #if self.controller.getYawVelocity() <= 0:
                        #self.controller.SetCommand(0, 0, 10, 0)
                else:
                    self.girando = False
                    self.controller.writeLog('YA LLEGAMOS')
                    #self.controller.SetCommand(0, 0, 0, 0)

    def despLateralDer(self):
        self.controller.SetCommand(-0.15, 0, 0, 0)
        self.despLateral = True

    def despLateralIzq(self):
        self.controller.SetCommand(0.15, 0, 0, 0)
        self.despLateral = True

    def paraDespLateral(self):
        self.controller.SetCommand(0, 0, 0, 0)
        self.despLateral = False


    """
    def sube(self, alt):
        alt_ini = self.controller.getNavdata().altd
        alt_fin = alt_ini + alt
        subiendo = True
        while subiendo:
            alt_act = self.controller.getNavdata().altd
            if alt_act < alt_fin:
                if self.controller.getLinearZ() == 0:
                    self.controller.SetCommand(0, 0, 0, 1)
            else:
                subiendo = False
                self.controller.writeLog('YA ESTAMOS ARRIBA')
                self.controller.SetCommand(0, 0, 0, 0)

    def baja(self, alt):
        alt_ini = self.controller.getNavdata().altd
        alt_fin = alt_ini - alt
        bajando = True
        while bajando:
            alt_act = self.controller.getNavdata().altd
            if alt_act > alt_fin:
                if self.controller.getLinearZ() == 0:
                    self.controller.SetCommand(0, 0, 0, -1)
            else:
                bajando = False
                self.controller.writeLog('YA ESTAMOS ABAJO')
                self.controller.SetCommand(0, 0, 0, 0)
    """
    def sube(self):
        self.controller.SetCommand(0, 0, 0, 0.15)
        self.subiendo = True

    def baja(self):
        self.controller.SetCommand(0, 0, 0, -0.15)
        self.bajando = True

    def subeIzq(self):
        self.controller.SetCommand(0.15, 0, 0, 0.15)
        self.subiendoIzq = True

    def subeDer(self):
        self.controller.SetCommand(-0.15, 0, 0, 0.15)
        self.subiendoDer = True

    def bajaIzq(self):
        self.controller.SetCommand(0.15, 0, 0, -0.15)
        self.bajandoIzq = True

    def bajaDer(self):
        self.controller.SetCommand(-0.15, 0, 0, -0.15)
        self.bajandoDer = True

    def avanza(self, vel):
        self.controller.SetCommand(0, vel, 0, 0)

    def retrocede(self, vel):
        self.controller.SetCommand(0, (-1)*vel, 0, 0)

    def para(self):
        self.controller.SetCommand(0, 0, 0, 0)
        self.girando = False
        self.despLatIzq = False
        self.despLatDer = False
        self.subiendoDer = False
        self.subiendoIzq = False
        self.bajandoDer = False
        self.bajandoIzq = False
        self.subiendo = False
        self.bajando = False





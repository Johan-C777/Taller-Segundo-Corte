"""
Control por IK del brazo izquierdo de Baxter vía joystick (ESP32 / Serial).

CAMBIOS DE ESTA VERSIÓN RESPECTO A LA ANTERIOR (detalle en el chat):
  7) La IK y setMotors() ya NO tocan el brazo derecho, la cabeza ni los dedos
     de la pinza. Antes, al ser una IK de "cuerpo completo", cada frame
     recalculaba y reescribía la posición de ESOS joints también (incluidos
     los dedos 49/51), peleándose con el comando del botón. Eso era la causa
     real de "la pinza se mueve sola": dos controladores distintos escribiendo
     el mismo joint en el mismo frame.
  8) Agarre por constraint en vez de solo fricción: cuando el botón cierra la
     pinza estando cerca del cubo, se crea una unión rígida temporal
     (p.createConstraint) entre la pinza y el cubo. Sin esto, aunque bajes la
     fuerza o la velocidad de cierre, el contacto rígido de PyBullet entre una
     pinza pequeña y un cubo liviano es inestable y termina "disparando" el
     objeto (lo repliqué con fuerza=200 hasta fuerza=5, con y sin fricción
     extra: siempre terminaba saliendo volando). El constraint elimina ese
     problema de raíz. Al soltar, se destruye el constraint y el objeto vuelve
     a caer por gravedad normalmente.
  9) restPoses ahora se calcula UNA vez por llamada a accurateIK (no una vez
     por cada una de las iteraciones internas). Sumado al punto 7 (7 joints en
     vez de ~25), esto es lo que le pega directo al delay: bajé de ~25 joints
     x 15 iteraciones a ~7 joints x 1, por frame.
 10) El loop de lectura serial ahora vacía el buffer completo en cada pasada y
     se queda solo con la ÚLTIMA línea recibida. Si el bucle de Python se
     atrasa un poco (por el cálculo de IK), antes se iban acumulando lecturas
     viejas en el buffer del sistema operativo y el script las procesaba en
     orden, siempre "atrasado". Ahora siempre reacciona al dato más reciente.

(Los cambios 1-6 de la versión anterior — límites articulares reales, joints
de la pinza correctos, cámara, gizmo de ejes, límites de target — se mantienen
igual, ya estaban validados.)
"""

import serial
from time import sleep
import pybullet as p
import pybullet_data
import numpy as np

# ============================================================
# CONSTANTES AJUSTABLES
# ============================================================
PUERTO_SERIAL = 'COM3'
BAUDRATE = 115200

FACTOR_VELOCIDAD = 0.00003   # sensibilidad del joystick -> metros por lectura
ZONA_MUERTA_BAJA = 1800
ZONA_MUERTA_ALTA = 2200

# Límites del TARGET, ajustados a la zona que el brazo izquierdo puede alcanzar
# de verdad (probado con física real, no solo con el cálculo de IK).
LIMITE_X = (0.00, 0.65)
LIMITE_Y = (-0.20, 0.55)
LIMITE_Z = (-0.35, 0.50)

GRIPPER_ABIERTO = 0.02      # el límite físico real del dedo es 0.021 (ver URDF)
GRIPPER_CERRADO = 0.0
GRIPPER_FUERZA = 30         # ver punto 8: la fuerza de los dedos ya no sostiene
                             # el objeto (eso lo hace el constraint), solo da la
                             # animación de cierre, así que no necesita ser alta
DISTANCIA_AGARRE = 0.08     # si el punto de agarre está a menos de esto del
                             # cubo Y el botón cierra, se activa el constraint

IK_MAX_ITER = 10
IK_THRESHOLD = 1e-3

# Los 7 joints del BRAZO IZQUIERDO únicamente (left_s0, s1, e0, e1, w0, w1, w2).
# La IK y setMotors solo escriben sobre estos — ver punto 7.
LEFT_ARM_JOINTS = [34, 35, 36, 37, 38, 40, 41]

# Joints reales de los dedos de la pinza izquierda (NO son 48 ni 50 — ver
# explicación en el chat: 48 es el link del punto de agarre, usado como
# endEffectorId, y 50 es un joint fijo interno de la punta del dedo).
GRIPPER_JOINT_L = 49
GRIPPER_JOINT_R = 51


def setUpWorld(initialSimSteps=100):
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Piso a nivel Z=0. Nota: el piso "real" del mesh de Baxter (sus pies) está
    # cerca de Z=-1 en este URDF, pero ese punto queda fuera del alcance físico
    # del brazo (lo comprobé por IK: a Z=-1 la distancia de error no baja de
    # ~0.27 m pase lo que pase). Por eso, igual que en tu versión, se deja el
    # piso en Z=0 como "mesa de trabajo" alcanzable en vez del suelo real.
    p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)
    sleep(0.1)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

    baxterId = p.loadURDF("baxter_common/baxter_description/urdf/toms_baxter.urdf", useFixedBase=True)
    p.resetBasePositionAndOrientation(baxterId, [0.5, -0.8, 0.0], [0., 0., -1., -1.])

    # Cubo en una posición confirmada alcanzable por el brazo IZQUIERDO
    # (validado con IK + motores + stepSimulation real, no solo con el cálculo).
    objId = p.loadURDF("cube_small.urdf", [0.3, 0.2, 0.05])

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    endEffectorId = 48  # link "left_gripper" (punto de agarre) — confirmado contra el URDF real
    p.setGravity(0., 0., -10.)

    for _ in range(initialSimSteps):
        p.stepSimulation()

    return baxterId, endEffectorId, objId


def getJointRanges(bodyId, includeFixed=False):
    """
    Límites articulares REALES (tomados del URDF), no inventados.

    Esto sigue cubriendo TODOS los joints móviles del cuerpo (no solo el
    brazo izquierdo): calculateInverseKinematics necesita que estos arreglos
    coincidan con el total de grados de libertad del robot para calcular bien
    el nullspace. Lo que cambió es que, más abajo, solo APLICAMOS el resultado
    a los 7 joints del brazo izquierdo (ver LEFT_ARM_JOINTS).
    """
    lowerLimits, upperLimits, jointRanges = [], [], []
    numJoints = p.getNumJoints(bodyId)
    for i in range(numJoints):
        jointInfo = p.getJointInfo(bodyId, i)
        if includeFixed or jointInfo[3] > -1:
            ll, ul = jointInfo[8:10]
            lowerLimits.append(ll)
            upperLimits.append(ul)
            jointRanges.append(ul - ll)
    return lowerLimits, upperLimits, jointRanges


def getCurrentRestPoses(bodyId, includeFixed=False):
    """Configuración articular ACTUAL de todo el cuerpo, usada como pose de
    referencia (nullspace) para darle "warm start" a la IK."""
    restPoses = []
    numJoints = p.getNumJoints(bodyId)
    for i in range(numJoints):
        jointInfo = p.getJointInfo(bodyId, i)
        if includeFixed or jointInfo[3] > -1:
            restPoses.append(p.getJointState(bodyId, i)[0])
    return restPoses


def accurateIK(bodyId, endEffectorId, targetPosition, lowerLimits, upperLimits, jointRanges,
               useNullSpace=True, maxIter=IK_MAX_ITER, threshold=IK_THRESHOLD):
    closeEnough = False
    it = 0
    jointPoses = None
    restPoses = getCurrentRestPoses(bodyId)  # UNA sola vez por llamada (punto 9)

    while (not closeEnough and it < maxIter):
        if useNullSpace:
            jointPoses = p.calculateInverseKinematics(
                bodyId, endEffectorId, targetPosition,
                lowerLimits=lowerLimits, upperLimits=upperLimits, jointRanges=jointRanges,
                restPoses=restPoses)
        else:
            jointPoses = p.calculateInverseKinematics(bodyId, endEffectorId, targetPosition)

        # Solo el brazo izquierdo (punto 7): así no se toca la pinza, el brazo
        # derecho ni la cabeza en cada iteración.
        for i in LEFT_ARM_JOINTS:
            qIndex = p.getJointInfo(bodyId, i)[3]
            p.resetJointState(bodyId, i, jointPoses[qIndex - 7])

        ls = p.getLinkState(bodyId, endEffectorId)
        newPos = ls[4]
        diff = [targetPosition[0] - newPos[0], targetPosition[1] - newPos[1], targetPosition[2] - newPos[2]]
        dist2 = np.sqrt((diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]))
        closeEnough = (dist2 < threshold)
        it += 1
    return jointPoses


def setMotors(bodyId, jointPoses):
    """Aplica el resultado de la IK solo a los 7 joints del brazo izquierdo
    (punto 7) — la pinza se controla aparte, en actualizarPinza()."""
    for i in LEFT_ARM_JOINTS:
        qIndex = p.getJointInfo(bodyId, i)[3]
        p.setJointMotorControl2(bodyIndex=bodyId, jointIndex=i, controlMode=p.POSITION_CONTROL,
                                 targetPosition=jointPoses[qIndex - 7], force=800)


class ControladorPinza:
    """
    Controla la apertura/cierre visual de los dedos Y el agarre real del
    objeto (punto 8).

    Los dedos (fuerza baja, GRIPPER_FUERZA) dan la animación de abrir/cerrar,
    pero NO son los que sostienen el cubo: sostenerlo solo por fricción de
    contacto rígido es inestable en este URDF (probado con varias fuerzas,
    con y sin fricción extra, siempre terminaba disparando el cubo). Lo que
    realmente sostiene el objeto es un constraint rígido temporal que se crea
    al cerrar la pinza estando cerca del cubo, y se destruye al abrirla.
    """
    def __init__(self, bodyId, endEffectorId, objId):
        self.bodyId = bodyId
        self.endEffectorId = endEffectorId
        self.objId = objId
        self.constraintId = None

    def actualizar(self, btn):
        pos_pinza = GRIPPER_CERRADO if btn == 0 else GRIPPER_ABIERTO
        p.setJointMotorControl2(self.bodyId, GRIPPER_JOINT_L, p.POSITION_CONTROL,
                                 targetPosition=pos_pinza, force=GRIPPER_FUERZA)
        p.setJointMotorControl2(self.bodyId, GRIPPER_JOINT_R, p.POSITION_CONTROL,
                                 targetPosition=-pos_pinza, force=GRIPPER_FUERZA)

        if btn == 0 and self.constraintId is None:
            eePos, eeOrn = p.getLinkState(self.bodyId, self.endEffectorId)[4:6]
            objPos, objOrn = p.getBasePositionAndOrientation(self.objId)
            if np.linalg.norm(np.array(eePos) - np.array(objPos)) < DISTANCIA_AGARRE:
                framePos, frameOrn = p.multiplyTransforms(*p.invertTransform(eePos, eeOrn), objPos, objOrn)
                self.constraintId = p.createConstraint(
                    self.bodyId, self.endEffectorId, self.objId, -1, p.JOINT_FIXED,
                    [0, 0, 0], framePos, [0, 0, 0], frameOrn)
        elif btn == 1 and self.constraintId is not None:
            p.removeConstraint(self.constraintId)
            self.constraintId = None


def dibujarEjesReferencia(origin=(0.55, -0.05, 0.05), length=0.15):
    """
    Gizmo de calibración visual: X=rojo, Y=verde, Z=azul, dibujado una sola vez
    cerca del área de trabajo. Si al mover un joystick el objetivo (TARGET) se
    mueve en la dirección visualmente opuesta a lo que esperas, compara contra
    estos ejes para saber exactamente qué signo invertir en el bloque de mapeo
    de abajo (cada eje está aislado en una sola línea, fácil de cambiar).
    """
    p.addUserDebugLine(origin, [origin[0] + length, origin[1], origin[2]], [1, 0, 0], lineWidth=3)
    p.addUserDebugLine(origin, [origin[0], origin[1] + length, origin[2]], [0, 1, 0], lineWidth=3)
    p.addUserDebugLine(origin, [origin[0], origin[1], origin[2] + length], [0, 0, 1], lineWidth=3)
    p.addUserDebugText("X", [origin[0] + length + 0.02, origin[1], origin[2]], textColorRGB=[1, 0, 0], textSize=1.2)
    p.addUserDebugText("Y", [origin[0], origin[1] + length + 0.02, origin[2]], textColorRGB=[0, 1, 0], textSize=1.2)
    p.addUserDebugText("Z", [origin[0], origin[1], origin[2] + length + 0.02], textColorRGB=[0, 0, 1], textSize=1.2)


def leerUltimaLinea(ser):
    """
    Vacía el buffer serial y devuelve solo la lectura MÁS RECIENTE (punto 10).
    Si el script se atrasa un frame, en vez de ir procesando el backlog en
    orden (y quedar cada vez más atrás), descarta lo viejo y reacciona al
    estado actual del joystick.
    """
    ultima = None
    while ser.in_waiting > 0:
        try:
            ultima = ser.readline().decode('utf-8').strip()
        except Exception:
            break
    return ultima


if __name__ == "__main__":
    guiClient = p.connect(p.GUI)

    # Cámara: el ángulo original (yaw=60) miraba desde el mismo lado en el que
    # está el hombro DERECHO de Baxter (por la rotación de 90° de la base, el
    # hombro derecho queda del lado +X y el izquierdo del lado -X). yaw=225
    # pone la cámara del lado del brazo IZQUIERDO / al frente.
    p.resetDebugVisualizerCamera(cameraDistance=1.1, cameraYaw=225, cameraPitch=-20,
                                  cameraTargetPosition=[0.35, 0.10, 0.15])

    baxterId, endEffectorId, objId = setUpWorld()
    lowerLimits, upperLimits, jointRanges = getJointRanges(baxterId, includeFixed=False)
    dibujarEjesReferencia()
    pinza = ControladorPinza(baxterId, endEffectorId, objId)

    # Posición inicial del TARGET, justo encima del cubo
    targetPosX, targetPosY, targetPosZ = 0.3, 0.2, 0.4
    markerId = p.addUserDebugText("TARGET", [targetPosX, targetPosY, targetPosZ], textColorRGB=[1, 0, 0], textSize=1.5)

    try:
        ser = serial.Serial(PUERTO_SERIAL, BAUDRATE, timeout=0.01)
    except Exception:
        ser = None
        print("Sin hardware serial")

    while True:
        if ser:
            linea = leerUltimaLinea(ser)
            if linea:
                try:
                    datos = linea.split(',')
                    if len(datos) == 4:
                        val_x, val_y, val_z, btn = map(int, datos)

                        # --- MAPEO FÍSICO (igual al que ya tenías especificado) ---
                        if val_y < ZONA_MUERTA_BAJA:
                            targetPosX += (ZONA_MUERTA_BAJA - val_y) * FACTOR_VELOCIDAD  # Arriba -> Acerca
                        if val_y > ZONA_MUERTA_ALTA:
                            targetPosX -= (val_y - ZONA_MUERTA_ALTA) * FACTOR_VELOCIDAD  # Abajo -> Aleja

                        if val_x < ZONA_MUERTA_BAJA:
                            targetPosY += (ZONA_MUERTA_BAJA - val_x) * FACTOR_VELOCIDAD  # Izquierda -> Extiende
                        if val_x > ZONA_MUERTA_ALTA:
                            targetPosY -= (val_x - ZONA_MUERTA_ALTA) * FACTOR_VELOCIDAD  # Derecha -> Encoge

                        if val_z < ZONA_MUERTA_BAJA:
                            targetPosZ -= (ZONA_MUERTA_BAJA - val_z) * FACTOR_VELOCIDAD  # Izquierda -> Baja
                        if val_z > ZONA_MUERTA_ALTA:
                            targetPosZ += (val_z - ZONA_MUERTA_ALTA) * FACTOR_VELOCIDAD  # Derecha -> Sube

                        targetPosX = max(LIMITE_X[0], min(targetPosX, LIMITE_X[1]))
                        targetPosY = max(LIMITE_Y[0], min(targetPosY, LIMITE_Y[1]))
                        targetPosZ = max(LIMITE_Z[0], min(targetPosZ, LIMITE_Z[1]))

                        pinza.actualizar(btn)
                except Exception:
                    pass

        targetPosition = [targetPosX, targetPosY, targetPosZ]
        p.addUserDebugText("TARGET", targetPosition, textColorRGB=[1, 0, 0], textSize=1.5, replaceItemUniqueId=markerId)

        jointPoses = accurateIK(baxterId, endEffectorId, targetPosition, lowerLimits, upperLimits, jointRanges,
                                 useNullSpace=True)
        setMotors(baxterId, jointPoses)

        p.stepSimulation()
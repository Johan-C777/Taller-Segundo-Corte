import serial
import math
import numpy as np
import pybullet as p
import pybullet_data
import cv2

# ============================================================
# CONFIGURACIÓN Y CONSTANTES
# ============================================================
PUERTO_SERIAL = 'COM3'
BAUDRATE = 115200
VAL_MAX = 4095
# El reposo real de cada joystick NO es exactamente 2000 (en el ESP32 suele
# quedar entre ~1800 y ~2400 según el módulo). Por eso el centro se calibra
# solo al arrancar y la zona muerta se mide alrededor de ese centro.
ZONA_MUERTA = 250                 # ± cuentas alrededor del centro calibrado
MUESTRAS_CALIBRACION = 40         # ~0.8 s de tramas del ESP32 (50 Hz)
DISPERSION_MAX_CAL = 300          # si un eje se mueve más que esto, se repite la calibración

VELOCIDAD_AVANCE_MAX = 1.0
VELOCIDAD_GIRO_MAX = 1.5

# Optimización Visual
CAM_ANCHO, CAM_ALTO = 240, 160
CAM_FOV, CAM_NEAR, CAM_FAR = 70, 0.05, 20.0
CAM_CADA_N_FRAMES = 5
VENTANAS = ("Synthetic Camera RGB data", "Synthetic Camera Depth data", "Synthetic Camera Segmentation Mask")

FUERZA_MARCHA = 800
FUERZA_CUELLO = 200

# Índices (Atlas)
TORSO_YAW = 0                       # back_bkz
L_SHOULDER_SWING, R_SHOULDER_SWING = 3, 11
NECK_PITCH = 10                     # neck_ry (link "head")
L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH = 20, 21, 22
R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH = 26, 27, 28
L_FOOT, R_FOOT = 23, 29

LIMITE_CUELLO = (-0.60, 1.14)
LIMITE_TORSO = (-0.66, 0.66)

# Osciladores de Caminata
AMPLITUD_CADERA, AMPLITUD_RODILLA, AMPLITUD_TOBILLO = 0.45, 0.60, 0.20
RODILLA_BASE = 0.15
AMPLITUD_BRAZO = 0.50
OMEGA_MARCHA_MAX = 2 * math.pi * 1.5
TAU_AMPLITUD = 0.15

# Variables de Salto
T_AGACHE, T_VUELO, T_ATERRIZAJE = 0.15, 0.40, 0.15
ALTURA_SALTO, FLEXION_SALTO = 0.30, 0.65

# Suelo / gravedad
GRAVEDAD = 9.8
ALTURA_PASO_MAX = 0.6     # escalón máximo que sube caminando (la caja mide 0.5 m sobre el piso)
TAU_SUBIDA = 0.08         # suavizado al subir un escalón
BASE_Z_INICIAL = -0.5


def iniciarMundo():
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

    atlas = p.loadURDF("atlas/atlas_v4_with_multisense.urdf", [-2, 3, BASE_Z_INICIAL], useFixedBase=True)
    for i in range(p.getNumJoints(atlas)):
        p.setJointMotorControl2(atlas, i, p.POSITION_CONTROL, 0, force=FUERZA_MARCHA)

    # Atlas se mueve cinemáticamente: se le quitan las colisiones. Así los rayos
    # al piso lo atraviesan (no se "detecta a sí mismo") y sus pies no pelean
    # contra el suelo.
    for link in range(-1, p.getNumJoints(atlas)):
        p.setCollisionFilterGroupMask(atlas, link, 0, 0)

    objs = p.loadSDF("botlab/botlab.sdf", globalScaling=2.0)
    for o in objs:
        pos, orn = p.getBasePositionAndOrientation(o)
        y2x = p.getQuaternionFromEuler([math.pi / 2, 0, math.pi / 2])
        p.resetBasePositionAndOrientation(o, *p.multiplyTransforms([0, 0, 0], y2x, pos, orn))

    p.loadURDF("boston_box.urdf", [-2, 3, -2], useFixedBase=True)
    p.resetDebugVisualizerCamera(1.8, 140, -15, [-2, 3, -0.5])
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.setGravity(0, 0, -10)
    return atlas


def limitesArticulares(atlas):
    return {j: p.getJointInfo(atlas, j)[8:10] for j in range(p.getNumJoints(atlas))}


class LectorSerial:
    """Lee sin bloquear y devuelve solo la última trama COMPLETA. Los pedazos
    de línea se guardan hasta que llegue su '\\n' (una trama cortada como
    '50,2100,...' en vez de '1950,2100,...' daría un tirón falso)."""

    def __init__(self, ser):
        self.ser = ser
        self.buffer = b""
        self.primera = True          # la primera línea tras abrir el puerto puede venir cortada

    def leerUltimaLinea(self):
        n = self.ser.in_waiting
        if n <= 0:
            return None
        self.buffer += self.ser.read(n)
        *lineas, self.buffer = self.buffer.split(b"\n")
        if lineas and self.primera:
            lineas, self.primera = lineas[1:], False
        return lineas[-1].decode('utf-8', errors='ignore').strip() if lineas else None


def parsearTrama(linea):
    """(x1, y1, x2, y2, btn) validados, o None si la trama no sirve."""
    try:
        datos = [int(v) for v in linea.split(',')]
    except ValueError:
        return None
    if len(datos) != 5 or not all(0 <= v <= VAL_MAX for v in datos[:4]) or datos[4] not in (0, 1):
        return None
    return datos


class Calibrador:
    """Toma el centro real de reposo de cada eje durante el primer ~0.8 s."""

    def __init__(self):
        self.reiniciar()

    def reiniciar(self):
        self.muestras = []
        self.centros = None
        print("Calibrando joysticks... no los toques")

    def agregar(self, ejes):
        self.muestras.append(ejes)
        if len(self.muestras) < MUESTRAS_CALIBRACION:
            return
        m = np.array(self.muestras)
        centros = np.median(m, axis=0)
        quietos = (m.max(axis=0) - m.min(axis=0)) <= DISPERSION_MAX_CAL
        en_rango = (centros > ZONA_MUERTA + 100) & (centros < VAL_MAX - ZONA_MUERTA - 100)
        if quietos.all() and en_rango.all():
            self.centros = centros
            print("Centros de reposo (x1, y1, x2, y2):", centros.astype(int).tolist())
        else:
            self.muestras = []
            print("Se movió un joystick durante la calibración, repitiendo... (suéltalos)")


def eje(val, centro):
    """-1..1 con zona muerta alrededor del centro calibrado.
    Positivo = stick hacia valores bajos (por debajo del centro)."""
    if val < centro - ZONA_MUERTA:
        return min(1.0, (centro - ZONA_MUERTA - val) / (centro - ZONA_MUERTA))
    if val > centro + ZONA_MUERTA:
        return -min(1.0, (val - centro - ZONA_MUERTA) / (VAL_MAX - centro - ZONA_MUERTA))
    return 0.0


def detectarPiso(atlas, z_desde):
    """Un rayo vertical bajo cada pie (p.rayTestBatch). Devuelve la Z del piso
    más alto encontrado (el pie que está apoyado), o None si no hay nada."""
    pies = [p.getLinkState(atlas, L_FOOT)[4], p.getLinkState(atlas, R_FOOT)[4]]
    desde = [[x, y, z_desde] for x, y, _ in pies]
    hasta = [[x, y, z_desde - 5.0] for x, y, _ in pies]
    alturas = [r[3][2] for r in p.rayTestBatch(desde, hasta) if r[0] >= 0 and r[0] != atlas]
    return max(alturas) if alturas else None


def plantaRelativa(atlas):
    """Altura de la planta más baja respecto a la base (depende solo de la pose
    de las piernas). Con esto la base baja al flexionar rodillas y el pie de
    apoyo queda pegado al piso."""
    baseZ = p.getBasePositionAndOrientation(atlas)[0][2]
    return min(p.getAABB(atlas, L_FOOT)[0][2], p.getAABB(atlas, R_FOOT)[0][2]) - baseZ


class Marcha:
    def __init__(self):
        self.fase = 0.0
        self.amplitud = 0.0

    def actualizar(self, avance_vel, giro_vel, dt):
        intensidad = min(1.0, abs(avance_vel) / VELOCIDAD_AVANCE_MAX + abs(giro_vel) / VELOCIDAD_GIRO_MAX)
        direccion = -1.0 if avance_vel < 0 else 1.0          # retroceder = marcha en reversa
        self.fase += direccion * intensidad * OMEGA_MARCHA_MAX * dt
        self.amplitud += (intensidad - self.amplitud) * min(1.0, dt / TAU_AMPLITUD)

    def poses(self):
        a = self.amplitud
        f_L = self.fase
        f_R = self.fase + math.pi

        hip_L = AMPLITUD_CADERA * a * math.sin(f_L)
        hip_R = AMPLITUD_CADERA * a * math.sin(f_R)
        # La rodilla se dobla mientras la pierna viaja hacia adelante (fase de vuelo),
        # máximo a mitad del paso; la pierna de apoyo va casi recta.
        knee_L = RODILLA_BASE + AMPLITUD_RODILLA * a * max(0.0, -math.cos(f_L))
        knee_R = RODILLA_BASE + AMPLITUD_RODILLA * a * max(0.0, -math.cos(f_R))
        # Tobillo: compensa cadera + rodilla (pie paralelo al piso) + impulso de punta.
        ankle_L = -(hip_L + knee_L) + AMPLITUD_TOBILLO * a * max(0.0, math.sin(f_L))
        ankle_R = -(hip_R + knee_R) + AMPLITUD_TOBILLO * a * max(0.0, math.sin(f_R))

        return {
            L_HIP_PITCH: hip_L, R_HIP_PITCH: hip_R,
            L_KNEE: knee_L, R_KNEE: knee_R,
            L_ANKLE_PITCH: ankle_L, R_ANKLE_PITCH: ankle_R,
            # shz está espejado entre brazos: mismo seno en ambos = braceo alternado,
            # brazo contrario a la pierna que avanza.
            L_SHOULDER_SWING: AMPLITUD_BRAZO * a * math.sin(f_R),
            R_SHOULDER_SWING: AMPLITUD_BRAZO * a * math.sin(f_R),
        }


class Salto:
    """Máquina de estados no bloqueante (agache -> vuelo parabólico -> aterrizaje).
    Usa el mismo reloj de simulación (dt fijo) que la marcha."""

    def __init__(self):
        self.activo, self.t = False, 0.0

    def disparar(self):
        if not self.activo:
            self.activo, self.t = True, 0.0

    def actualizar(self, dt):
        if not self.activo:
            return 0.0, None
        self.t += dt
        t = self.t
        if t < T_AGACHE:
            return 0.0, FLEXION_SALTO * (t / T_AGACHE)
        t -= T_AGACHE
        if t < T_VUELO:
            frac = t / T_VUELO
            return ALTURA_SALTO * 4 * frac * (1 - frac), FLEXION_SALTO * (1 - frac)
        t -= T_VUELO
        if t < T_ATERRIZAJE:
            return 0.0, FLEXION_SALTO * 0.5 * (1 - t / T_ATERRIZAJE)
        self.activo = False
        return 0.0, None


def capturarCamaras(bodyId):
    headPos, headOrn = p.getLinkState(bodyId, NECK_PITCH)[4:6]
    rot = np.array(p.getMatrixFromQuaternion(headOrn)).reshape(3, 3)
    eye = np.array(headPos)
    target = eye + (rot @ np.array([1, 0, 0])) * 3.0
    up = rot @ np.array([0, 0, 1])

    vm = p.computeViewMatrix(eye.tolist(), target.tolist(), up.tolist())
    pm = p.computeProjectionMatrixFOV(CAM_FOV, CAM_ANCHO / CAM_ALTO, CAM_NEAR, CAM_FAR)
    _, _, rgbB, depthB, segB = p.getCameraImage(CAM_ANCHO, CAM_ALTO, viewMatrix=vm, projectionMatrix=pm)

    bgr = cv2.cvtColor(np.reshape(rgbB, (CAM_ALTO, CAM_ANCHO, 4))[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2BGR)

    depthR = (CAM_FAR * CAM_NEAR) / (CAM_FAR - (CAM_FAR - CAM_NEAR) * np.reshape(depthB, (CAM_ALTO, CAM_ANCHO)))
    depthC = cv2.applyColorMap(((np.clip(depthR, CAM_NEAR, 6.0) - CAM_NEAR) / (6.0 - CAM_NEAR) * 255).astype(np.uint8),
                               cv2.COLORMAP_BONE)

    segB = np.reshape(segB, (CAM_ALTO, CAM_ANCHO)).astype(np.int32)
    segC = np.zeros((CAM_ALTO, CAM_ANCHO, 3), dtype=np.uint8)
    for objId in np.unique(segB):
        if objId < 0:
            continue
        h = (int(objId) * 2654435761) & 0xFFFFFF
        segC[segB == objId] = (h & 0xFF, (h >> 8) & 0xFF, (h >> 16) & 0xFF)

    return bgr, depthC, segC


def ventanaCerrada():
    for nombre in VENTANAS:
        try:
            if cv2.getWindowProperty(nombre, cv2.WND_PROP_VISIBLE) < 1:
                return True
        except cv2.error:
            return True
    return False


if __name__ == "__main__":
    atlas = iniciarMundo()
    limites = limitesArticulares(atlas)

    baseX, baseY, yaw = -2.0, 3.0, 0.0
    cuelloPitch, torsoYaw = 0.0, 0.0
    avance_vel, giro_vel = 0.0, 0.0

    # Estado vertical: altura del apoyo de los pies y su velocidad de caída
    z_apoyo = detectarPiso(atlas, BASE_Z_INICIAL)
    if z_apoyo is None:
        z_apoyo = BASE_Z_INICIAL + plantaRelativa(atlas)
    vz = 0.0

    marcha, salto = Marcha(), Salto()

    try:
        ser = serial.Serial(PUERTO_SERIAL, BAUDRATE, timeout=0.01)
        lector = LectorSerial(ser)
    except Exception:
        ser = lector = None
        print("Sin hardware serial")
    calibrador = Calibrador()

    for nombre in VENTANAS:
        cv2.namedWindow(nombre, cv2.WINDOW_NORMAL)

    frame = 0
    try:
        while p.isConnected():
            # ---------------- Serial (no bloqueante, solo la última línea) ----------------
            if ser:
                try:
                    linea = lector.leerUltimaLinea()
                except (serial.SerialException, OSError):
                    print("ESP32 desconectado")
                    ser, linea = None, None
                    avance_vel = giro_vel = 0.0
                trama = parsearTrama(linea) if linea else None
                if trama:
                    val_x1, val_y1, val_x2, val_y2, btn = trama

                    if calibrador.centros is None:
                        # Mientras calibra, el robot se queda quieto
                        calibrador.agregar((val_x1, val_y1, val_x2, val_y2))
                        avance_vel = giro_vel = torsoYaw = cuelloPitch = 0.0
                    else:
                        cx1, cy1, cx2, cy2 = calibrador.centros
                        avance_vel = eje(val_y1, cy1) * VELOCIDAD_AVANCE_MAX
                        giro_vel = eje(val_x1, cx1) * VELOCIDAD_GIRO_MAX
                        torsoYaw = eje(val_x2, cx2) * LIMITE_TORSO[1]
                        e = eje(val_y2, cy2)
                        cuelloPitch = e * (LIMITE_CUELLO[1] if e > 0 else -LIMITE_CUELLO[0])

                        if btn == 0:
                            salto.disparar()

            dt = 1.0 / 240.0

            baseX += math.cos(yaw) * avance_vel * dt
            baseY += math.sin(yaw) * avance_vel * dt
            yaw += giro_vel * dt

            # ---------------- Piso + gravedad ----------------
            piso_z = detectarPiso(atlas, z_apoyo + ALTURA_PASO_MAX)
            if piso_z is None:
                piso_z = z_apoyo
            if z_apoyo > piso_z + 1e-4:                 # sin apoyo debajo: cae
                vz -= GRAVEDAD * dt
                z_apoyo = max(piso_z, z_apoyo + vz * dt)
                if z_apoyo == piso_z:
                    vz = 0.0
            else:                                        # apoyado (o subiendo un escalón)
                vz = 0.0
                z_apoyo += (piso_z - z_apoyo) * min(1.0, dt / TAU_SUBIDA)

            marcha.actualizar(avance_vel, giro_vel, dt)
            z_salto, flexion_salto = salto.actualizar(dt)

            # ---------------- Articulaciones ----------------
            objetivos = marcha.poses()
            if flexion_salto is not None:
                # Sentadilla con pie plano: cadera y tobillo compensan la rodilla
                objetivos[L_KNEE] = objetivos[R_KNEE] = RODILLA_BASE + flexion_salto
                objetivos[L_HIP_PITCH] = objetivos[R_HIP_PITCH] = -(RODILLA_BASE + flexion_salto) / 2
                objetivos[L_ANKLE_PITCH] = objetivos[R_ANKLE_PITCH] = -(RODILLA_BASE + flexion_salto) / 2

            objetivos[TORSO_YAW] = torsoYaw
            objetivos[NECK_PITCH] = cuelloPitch

            for jointId, target in objetivos.items():
                ll, ul = limites[jointId]
                p.setJointMotorControl2(atlas, jointId, p.POSITION_CONTROL,
                                        targetPosition=min(max(target, ll), ul),
                                        force=FUERZA_CUELLO if jointId == NECK_PITCH else FUERZA_MARCHA)

            p.stepSimulation()

            # Base DESPUÉS del paso: se usa la pose de piernas recién calculada, así el
            # pie de apoyo queda exactamente sobre el piso (sin retraso de un frame).
            baseZ = z_apoyo - plantaRelativa(atlas) + z_salto
            p.resetBasePositionAndOrientation(atlas, [baseX, baseY, baseZ], p.getQuaternionFromEuler([0, 0, yaw]))

            if frame % CAM_CADA_N_FRAMES == 0:
                bgr, depthC, segC = capturarCamaras(atlas)
                cv2.imshow(VENTANAS[0], bgr)
                cv2.imshow(VENTANAS[1], depthC)
                cv2.imshow(VENTANAS[2], segC)

            frame += 1
            tecla = cv2.waitKey(1) & 0xFF
            if tecla == ord('c'):            # recalibrar el reposo de los joysticks
                calibrador.reiniciar()
            if tecla == ord('q') or (frame > CAM_CADA_N_FRAMES and ventanaCerrada()):
                break

    except p.error:
        pass            # se cerró la ventana de PyBullet a mitad de un frame
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if ser:
            try:
                ser.close()
            except Exception:
                pass
        try:
            if p.isConnected():
                p.disconnect()
        except p.error:
            pass
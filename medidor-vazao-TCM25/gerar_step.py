"""
Modelo 3D (STEP) do medidor de vazão tipo turbina TechMeter
TCM-25-BBEA-10R-KF2 (pedido como TCM25-BBER-10R/KF) - DN25 (1") - Conexões BSP 1 1/4" macho.

Dimensões tiradas de fotos da peça real, usando a rosca BSP 1 1/4" (Ø41,91 mm)
como referência de escala. Precisão estimada: +-1 a 2 mm. Confira com paquímetro
as cotas críticas antes de usar em projeto.

Unidades: mm. Eixo X = sentido do fluxo (entrada em X negativo).
O sensor (pick-up + amplificador) fica em +Z.

Uso:  pip install cadquery && python gerar_step.py
"""
import math
import cadquery as cq

# ---------------- Corpo ----------------
L = 105.0                  # comprimento total (ponta a ponta)
D_FURO = 25.0              # passagem interna (DN25)
R_TUBO = 39.5 / 2          # tubo externo
# Rosca BSP 1 1/4" (ISO 228): Ø maior 41,910 mm, 11 fios/pol, altura do filete 0,640327*p
R_ROSCA = 41.910 / 2
PASSO = 25.4 / 11
H_FIO = 0.640327 * PASSO
L_ROSCA_ENT = 16.0         # rosca de entrada
L_ROSCA_SAI = 19.0         # rosca de saída
L_ALIVIO = 2.0             # canal de alívio após as roscas
# Sextavado (onde entra o sensor) - fica deslocado para o lado da saída
SEXT_AF = 46.0             # entre faces
SEXT_X0, SEXT_X1 = 55.0, 79.0
COLAR_X1 = 84.0            # colar entre sextavado e rosca de saída
X_SENSOR = (SEXT_X0 + SEXT_X1) / 2

# ---------------- Sensor / amplificador (cotas em Z a partir do eixo) ----------------
Z_FACE = SEXT_AF / 2       # face do sextavado onde o sensor é montado
PORCA_AF = 22.0            # contraporca do pick-up
D_ROSCA_SENSOR = 18.0      # rosca do pick-up (M18x1,5)
D_AMP = 25.0               # cilindro do amplificador
FLANGE = 34.0              # placa quadrada
D_CONECTOR = 24.5          # anel do conector aviação
D_CABO = 7.0


def rosca(x0, comprimento, r_maior, passo, h, eixo="X"):
    """Rosca representada por filetes em V revolucionados (anéis no passo real)."""
    r_menor = r_maior - h
    pts = [(x0, 0), (x0, r_menor)]
    x = x0
    while x + passo <= x0 + comprimento + 1e-6:
        pts += [(x + passo / 2, r_maior), (x + passo, r_menor)]
        x += passo
    if x < x0 + comprimento:
        pts.append((x0 + comprimento, r_menor))
    pts.append((x0 + comprimento, 0))
    s = cq.Workplane("XY").polyline(pts).close().revolve(360, (0, 0, 0), (1, 0, 0))
    if eixo == "Z":
        s = s.rotate((0, 0, 0), (0, 1, 0), -90)
    return s


def cil_x(x0, comp, r):
    return cq.Workplane("YZ").workplane(offset=x0).circle(r).extrude(comp)


def cil_z(z0, comp, r, x=X_SENSOR, y=0.0):
    return cq.Workplane("XY").workplane(offset=z0).center(x, y).circle(r).extrude(comp)


# ======================= CORPO =======================
corpo = rosca(0, L_ROSCA_ENT, R_ROSCA, PASSO, H_FIO)
corpo = corpo.union(cil_x(L_ROSCA_ENT, L_ALIVIO, R_ROSCA - H_FIO - 0.2))
corpo = corpo.union(cil_x(L_ROSCA_ENT + L_ALIVIO, SEXT_X0 - L_ROSCA_ENT - L_ALIVIO, R_TUBO))
# sextavado com chanfro de barra (interseção com cilindro chanfrado)
ac = SEXT_AF / math.cos(math.pi / 6)
sext = cq.Workplane("YZ").workplane(offset=SEXT_X0).polygon(6, ac).extrude(SEXT_X1 - SEXT_X0)
sext = sext.intersect(cil_x(SEXT_X0, SEXT_X1 - SEXT_X0, ac / 2).edges().chamfer(2.5))
corpo = corpo.union(sext)
corpo = corpo.union(cil_x(SEXT_X1, COLAR_X1 - SEXT_X1, R_TUBO))
corpo = corpo.union(cil_x(COLAR_X1, L_ALIVIO, R_ROSCA - H_FIO - 0.2))
corpo = corpo.union(rosca(COLAR_X1 + L_ALIVIO, L - COLAR_X1 - L_ALIVIO, R_ROSCA, PASSO, H_FIO))
# ressalto redondo de assento do pick-up
corpo = corpo.union(cil_z(Z_FACE - 2, 4.0, 11.0))
# passagem interna com chanfro nas bocas
corpo = corpo.cut(cil_x(-1, L + 2, D_FURO / 2))
for xb, sgn in ((0, 1), (L, -1)):
    cone = cq.Solid.makeCone(D_FURO / 2 + 1.2, D_FURO / 2, 1.2,
                             cq.Vector(xb, 0, 0), cq.Vector(sgn, 0, 0))
    corpo = corpo.cut(cq.Workplane().add(cone))

# gravação "FLOW -> DN25" na face do sextavado vizinha ao sensor
n = cq.Vector(0, -math.sin(math.radians(60)), math.cos(math.radians(60)))
face = cq.Plane(origin=cq.Vector(X_SENSOR, 0, 0) + n * (SEXT_AF / 2), xDir=(1, 0, 0), normal=n.toTuple())
seta = (cq.Workplane(face).center(0, 0.5)
        .polyline([(-8, -1.6), (3, -1.6), (3, -3.6), (8.5, 0), (3, 3.6), (3, 1.6), (-8, 1.6)]).close()
        .extrude(-0.3))
corpo = corpo.cut(seta)
try:
    for txt, dy in (("FLOW", 7.0), ("DN25", -6.0)):
        corpo = corpo.cut(cq.Workplane(face).center(0, dy).text(txt, 4.5, -0.3, kind="bold"))
except Exception as e:  # fonte indisponível: mantém só a seta
    print("aviso: texto gravado omitido:", e)

# ======================= INTERNOS =======================
def guia(x0, x1, nariz):
    """Retificador de fluxo em cruz (4 aletas) com cubo e ponta cônica."""
    g = cil_x(x0, x1 - x0, 5.0)
    for k in range(4):
        g = g.union(cq.Workplane("YZ").workplane(offset=x0).rect(1.6, D_FURO - 0.2).extrude(x1 - x0)
                    .rotate((0, 0, 0), (1, 0, 0), 45 * (k % 2)))
    xn, sgn = nariz
    g = g.union(cq.Workplane().add(cq.Solid.makeCone(5.0, 1.0, 5.0, cq.Vector(xn, 0, 0), cq.Vector(sgn, 0, 0))))
    return g


def anel_trava(x):
    return cq.Workplane("YZ").workplane(offset=x).circle(D_FURO / 2).circle(D_FURO / 2 - 1.3).extrude(1.0)


guias = guia(40, 52, (52, 1)).union(guia(82, 96, (82, -1)))
guias = guias.union(anel_trava(38.5)).union(anel_trava(96.5))
# cubo do mancal ligando as guias ao rotor
guias = guias.union(cil_x(57, 2, 2.0)).union(cil_x(75, 2, 2.0))

rotor = cil_x(59, 16, 5.0)
for k in range(6):
    pa = (cq.Workplane("YZ").workplane(offset=61).center(0, 8.6).rect(1.3, 7.2)
          .twistExtrude(12, 45).rotate((0, 0, 0), (1, 0, 0), 60 * k))
    rotor = rotor.union(pa)

# ======================= SENSOR / AMPLIFICADOR =======================
def sext_z(z0, h, af, x=X_SENSOR, y=0.0, ch=0.8):
    s = cq.Workplane("XY").workplane(offset=z0).center(x, y).polygon(6, af / math.cos(math.pi / 6)).extrude(h)
    return s.intersect(cil_z(z0, h, af / math.cos(math.pi / 6) / 2, x, y).edges().chamfer(ch))


inox_sensor = sext_z(25, 6, PORCA_AF)                                   # contraporca
inox_sensor = inox_sensor.union(rosca(31, 6, D_ROSCA_SENSOR / 2, 1.5, 0.92, "Z").translate((X_SENSOR, 0, 0)))
inox_sensor = inox_sensor.union(cil_z(37, 3, 10.0))                     # colar
inox_sensor = inox_sensor.union(cq.Workplane().add(
    cq.Solid.makeCone(10.0, D_AMP / 2, 7, cq.Vector(X_SENSOR, 0, 40), cq.Vector(0, 0, 1))))
inox_sensor = inox_sensor.union(cil_z(47, 42, D_AMP / 2))               # corpo do amplificador
placa = (cq.Workplane("XY").workplane(offset=89).center(X_SENSOR, 0).rect(FLANGE, FLANGE).extrude(3)
         .edges("|Z").fillet(1.5))
inox_sensor = inox_sensor.union(placa)
for sx in (-1, 1):
    for sy in (-1, 1):
        xs, ys = X_SENSOR + 13 * sx, 13 * sy
        inox_sensor = inox_sensor.union(cil_z(82.5, 9.5, 2.0, xs, ys))  # prisioneiro M4
        inox_sensor = inox_sensor.union(sext_z(85.8, 3.2, 7.0, xs, ys, 0.4))  # porca M4

etiqueta = cil_z(50, 32, D_AMP / 2 + 0.15).cut(cil_z(49, 34, D_AMP / 2))

vedacao = (cq.Workplane("XY").workplane(offset=92).center(X_SENSOR, 0).rect(33, 33).extrude(3)
           .edges("|Z").fillet(2))

conector = (cq.Workplane("XY").workplane(offset=95).center(X_SENSOR, 0).rect(32, 32).extrude(3)
            .edges("|Z").fillet(3))
for sx in (-1, 1):
    for sy in (-1, 1):
        conector = conector.union(cil_z(98, 2.2, 3.0, X_SENSOR + 12 * sx, 12 * sy).faces(">Z").edges().fillet(1.0))
conector = conector.union(cil_z(98, 6, 10.0))
anel = cil_z(104, 17, D_CONECTOR / 2).edges().chamfer(1.0)
for k in range(16):  # estrias do anel de acoplamento
    a = 2 * math.pi * k / 16
    anel = anel.cut(cq.Workplane("XY").workplane(offset=107).center(X_SENSOR + math.cos(a) * D_CONECTOR / 2,
                                                                   math.sin(a) * D_CONECTOR / 2)
                    .rect(1.6, 1.6).extrude(11).rotate((X_SENSOR, 0, 0), (X_SENSOR, 0, 1), 0))
for zc in (105.5, 119.0):
    anel = anel.cut(cil_z(zc, 1.0, D_CONECTOR / 2 + 1).cut(cil_z(zc - 1, 3, D_CONECTOR / 2 - 0.6)))
conector = conector.union(anel).union(cil_z(121, 11, 8.5))

abracadeira = cil_z(128, 5, 9.5).cut(cil_z(127, 7, 8.5))
abracadeira = abracadeira.union(cq.Workplane("XY").workplane(offset=128).center(X_SENSOR, 0).rect(30, 4).extrude(5)
                                .cut(cil_z(127, 7, 8.5)))
for sx in (-1, 1):
    abracadeira = abracadeira.union(cq.Workplane("XZ").workplane(offset=-5).center(X_SENSOR + 12.5 * sx, 130.5)
                                    .circle(1.5).extrude(-10))

# cabo de 4 vias saindo do conector
xs = X_SENSOR
pts = [(xs, 0, 132), (xs, 0, 150), (xs + 8, 0, 170), (xs + 30, 0, 184), (xs + 75, 0, 190), (xs + 125, 0, 186)]
caminho = cq.Workplane("XY").spline(pts, includeCurrent=False)
cabo = cq.Workplane(cq.Plane(origin=pts[0], xDir=(1, 0, 0), normal=(0, 0, 1))).circle(D_CABO / 2).sweep(caminho)

# fios na ponta do cabo: amarelo (Fout), vermelho, azul e malha
fios = {}
fim = cq.Vector(*pts[-1])
for nome, (dy, dz, comp) in {"fio_amarelo": (-2.2, 1.0, 28), "fio_vermelho": (0.0, 2.2, 32),
                             "fio_azul": (2.2, 0.6, 30), "fio_malha": (0.0, -1.8, 24)}.items():
    p0 = fim + cq.Vector(-2, dy * 0.4, dz * 0.4)
    p2 = fim + cq.Vector(comp, dy * 3, dz * 3 - 4)
    p1 = (p0 + p2) * 0.5 + cq.Vector(0, 0, 3)
    path = cq.Workplane("XY").spline([p0.toTuple(), p1.toTuple(), p2.toTuple()], includeCurrent=False)
    tang = (p1 - p0).normalized()
    fio = cq.Workplane(cq.Plane(origin=p0.toTuple(), xDir=(0, 1, 0), normal=tang.toTuple())).circle(0.8).sweep(path)
    fios[nome] = fio

# ======================= MONTAGEM =======================
DX = -L / 2  # centraliza o corpo na origem
C = {
    "inox": (0.80, 0.81, 0.82), "inox_escuro": (0.55, 0.56, 0.58), "rotor": (0.22, 0.23, 0.25),
    "preto": (0.06, 0.06, 0.07), "borracha": (0.10, 0.10, 0.10), "conector": (0.42, 0.43, 0.45),
    "zinco": (0.78, 0.74, 0.55), "amarelo": (0.95, 0.80, 0.10), "vermelho": (0.80, 0.10, 0.10),
    "azul": (0.10, 0.30, 0.80), "malha": (0.85, 0.80, 0.70),
}
PECAS = [
    ("corpo", corpo, "inox"),
    ("retificadores", guias, "inox_escuro"),
    ("rotor", rotor, "rotor"),
    ("pickup_amplificador", inox_sensor, "inox"),
    ("etiqueta", etiqueta, "preto"),
    ("vedacao", vedacao, "borracha"),
    ("conector_aviacao", conector, "conector"),
    ("abracadeira", abracadeira, "zinco"),
    ("cabo", cabo, "preto"),
    ("fio_amarelo_Fout", fios["fio_amarelo"], "amarelo"),
    ("fio_vermelho", fios["fio_vermelho"], "vermelho"),
    ("fio_azul", fios["fio_azul"], "azul"),
    ("malha_blindagem", fios["fio_malha"], "malha"),
]
PECAS = [(nome, w.translate((DX, 0, 0)), cor) for nome, w, cor in PECAS]

if __name__ == "__main__":
    asm = cq.Assembly(name="TCM-25-BBEA-10R-KF2")
    for nome, w, cor in PECAS:
        asm.add(w, name=nome, color=cq.Color(*C[cor]))
    asm.export("TCM25-BBER-10R_KF.step")
    print("OK: TCM25-BBER-10R_KF.step")

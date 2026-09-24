"""
Modelo 3D (STEP) aproximado do medidor de vazão tipo turbina
TCM25-BBER-10R/KF - Nominal 1" - Conexões BSP 1 1/4" macho.

ATENÇÃO: dimensões estimadas (típicas de turbinas 1" roscadas).
Ajuste os parâmetros abaixo conforme o datasheet/desenho do fabricante.
Unidades: mm. Eixo X = sentido do fluxo.

Uso:  pip install cadquery && python gerar_step.py
"""
import math
import cadquery as cq

# ---------------- Parâmetros ----------------
L_TOTAL = 150.0        # comprimento total face a face
D_CORPO = 52.0         # diâmetro externo do corpo
D_FURO = 25.0          # diâmetro interno (nominal 1")
# Rosca BSP 1 1/4" (ISO 228 / ISO 7): Ø maior 41,910 mm, 11 fios/pol
D_ROSCA = 41.910
PASSO = 25.4 / 11
L_ROSCA = 22.0         # comprimento da rosca em cada extremidade
L_SEXT = 12.0          # sextavado de aperto em cada lado
SEXT_AF = 50.0         # medida entre faces do sextavado
# Sensor (pick-up magnético) e cabeçote
D_BOSS = 30.0
H_BOSS = 12.0          # altura do ressalto acima do corpo
D_PICKUP = 18.0        # rosca do pick-up (ex.: M18x1)
H_PICKUP = 30.0
D_CABECOTE = 80.0      # caixa do indicador/totalizador
H_CABECOTE = 45.0
D_VISOR = 60.0
# Rotor
N_PAS = 6
L_ROTOR = 18.0
D_CUBO = 9.0

r_corpo = D_CORPO / 2
x0 = -L_TOTAL / 2

# ---------------- Corpo ----------------
L_CENTRAL = L_TOTAL - 2 * (L_ROSCA + L_SEXT)
corpo = cq.Workplane("YZ").workplane(offset=-L_CENTRAL / 2).circle(r_corpo).extrude(L_CENTRAL)

for s in (-1, 1):
    # sextavado
    xs = s * L_CENTRAL / 2 if s > 0 else -L_CENTRAL / 2 - L_SEXT
    sext = cq.Workplane("YZ").workplane(offset=xs).polygon(6, SEXT_AF / math.cos(math.pi / 6)).extrude(L_SEXT)
    corpo = corpo.union(sext)
    # rosca (representação: cilindro com sulcos anelares no passo da BSP)
    xr = L_CENTRAL / 2 + L_SEXT if s > 0 else -L_TOTAL / 2
    rosca = cq.Workplane("YZ").workplane(offset=xr).circle(D_ROSCA / 2).extrude(L_ROSCA)
    prof = 0.64 * PASSO * 0.5
    n = int((L_ROSCA - 2) / PASSO)
    for i in range(n):
        xi = xr + 1.5 + i * PASSO
        anel = (cq.Workplane("YZ").workplane(offset=xi)
                .circle(D_ROSCA / 2 + 1).circle(D_ROSCA / 2 - prof).extrude(PASSO / 2))
        rosca = rosca.cut(anel)
    # chanfro de entrada
    rosca = rosca.faces("<X" if s < 0 else ">X").edges().chamfer(1.0)
    corpo = corpo.union(rosca)

# ressalto do sensor
boss = cq.Workplane("XY").workplane(offset=r_corpo - 5).circle(D_BOSS / 2).extrude(H_BOSS + 5)
corpo = corpo.union(boss)

# furo passante
furo = cq.Workplane("YZ").workplane(offset=x0 - 1).circle(D_FURO / 2).extrude(L_TOTAL + 2)
corpo = corpo.cut(furo)

# seta de fluxo gravada
seta = (cq.Workplane("XZ").workplane(offset=-r_corpo - 1)
        .polyline([(-20, -2), (8, -2), (8, -6), (20, 0), (8, 6), (8, 2), (-20, 2)]).close()
        .extrude(1.6))
corpo = corpo.cut(seta)

# ---------------- Internos: retificadores + rotor ----------------
internos = None
for s in (-1, 1):
    xc = s * (L_CENTRAL / 2 - 6)
    cubo = cq.Workplane("YZ").workplane(offset=xc - 6).circle(D_CUBO / 2).extrude(12)
    for k in range(4):
        aleta = (cq.Workplane("YZ").workplane(offset=xc - 6)
                 .center(0, D_FURO / 4).rect(1.5, D_FURO / 2).extrude(12)
                 .rotate((0, 0, 0), (1, 0, 0), 90 * k + 45))
        cubo = cubo.union(aleta)
    internos = cubo if internos is None else internos.union(cubo)

eixo = cq.Workplane("YZ").workplane(offset=-(L_CENTRAL / 2 - 6)).circle(1.5).extrude(L_CENTRAL - 12)
rotor = cq.Workplane("YZ").workplane(offset=-L_ROTOR / 2).circle(D_CUBO / 2).extrude(L_ROTOR)
R_PA = (D_CUBO / 2 + D_FURO / 2 - 0.8) / 2   # raio médio da pá
for k in range(N_PAS):
    pa = (cq.Workplane("YZ").workplane(offset=-L_ROTOR / 2 + 2)
          .center(0, R_PA).rect(1.2, D_FURO / 2 - 0.8 - D_CUBO / 2 + 1).extrude(L_ROTOR - 4)
          .rotate((0, 0, R_PA), (0, 0, R_PA + 1), 30)          # inclinação da pá (~30°)
          .rotate((0, 0, 0), (1, 0, 0), 360 / N_PAS * k))
    rotor = rotor.union(pa)
rotor = rotor.union(eixo)
internos = internos.union(rotor)

# ---------------- Sensor + cabeçote ----------------
z_topo = r_corpo + H_BOSS
pickup = cq.Workplane("XY").workplane(offset=z_topo).circle(D_PICKUP / 2).extrude(H_PICKUP)
porca = cq.Workplane("XY").workplane(offset=z_topo + 4).polygon(6, 27 / math.cos(math.pi / 6)).extrude(8)
pescoco = cq.Workplane("XY").workplane(offset=z_topo + H_PICKUP).circle(12).extrude(10)
z_cab = z_topo + H_PICKUP + 10
cab = (cq.Workplane("XY").workplane(offset=z_cab).circle(D_CABECOTE / 2).extrude(H_CABECOTE)
       .edges(">Z").fillet(4))
# visor rebaixado no topo e prensa-cabo lateral
cab = cab.cut(cq.Workplane("XY").workplane(offset=z_cab + H_CABECOTE - 1)
              .circle(D_VISOR / 2).extrude(2))
prensa = (cq.Workplane("YZ").workplane(offset=D_CABECOTE / 2 - 3)
          .center(0, z_cab + H_CABECOTE / 2).polygon(6, 22 / math.cos(math.pi / 6)).extrude(14))
prensa = prensa.union(cq.Workplane("YZ").workplane(offset=D_CABECOTE / 2 + 11)
                      .center(0, z_cab + H_CABECOTE / 2).circle(8).extrude(10))
sensor = pickup.union(porca).union(pescoco).union(cab).union(prensa)

# ---------------- Montagem ----------------
asm = cq.Assembly(name="TCM25-BBER-10R_KF")
asm.add(corpo, name="corpo", color=cq.Color(0.75, 0.75, 0.78))
asm.add(internos, name="rotor_retificadores", color=cq.Color(0.55, 0.55, 0.6))
asm.add(sensor, name="sensor_cabecote", color=cq.Color(0.15, 0.35, 0.65))
asm.export("TCM25-BBER-10R_KF.step")
print("OK: TCM25-BBER-10R_KF.step")

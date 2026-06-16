#!/usr/bin/env python3
"""
generate_map.py — Génère la carte PGM de l'hôpital pour Nav2
PharmaBot | Ibn Tofaïl University | 2025-2026

Exécuter UNE FOIS depuis le dossier maps/ :
    python3 generate_map.py

Génère : hospital_map.pgm + hospital_map.yaml
"""

import os
import struct

# ── Paramètres carte ──────────────────────────────────────
RESOLUTION   = 0.1       # mètres / pixel (0.1m = 10cm)
ORIGIN_X     = -16.0     # coin bas-gauche de la carte (mètres)
ORIGIN_Y     = -32.0
MAP_W_M      = 32.0      # largeur totale monde (mètres)
MAP_H_M      = 56.0      # hauteur totale monde (mètres)

FREE         = 255        # pixel blanc = libre
OCCUPIED     = 0          # pixel noir  = obstacle/mur
UNKNOWN      = 205        # pixel gris  = inconnu

# ── Dimensions en pixels ──────────────────────────────────
W = int(MAP_W_M / RESOLUTION)   # 320
H = int(MAP_H_M / RESOLUTION)   # 560


def world_to_pixel(wx, wy):
    """Convertit coordonnées monde (m) → pixel (col, row)."""
    col = int((wx - ORIGIN_X) / RESOLUTION)
    row = int((MAP_H_M + ORIGIN_Y - wy) / RESOLUTION)
    return col, row


def fill_rect(grid, wx, wy, ww, wh, val):
    """Remplit un rectangle (coordonnées monde) avec val."""
    c0, r0 = world_to_pixel(wx - ww/2, wy + wh/2)
    c1, r1 = world_to_pixel(wx + ww/2, wy - wh/2)
    for r in range(max(0, r0), min(H, r1+1)):
        for c in range(max(0, c0), min(W, c1+1)):
            grid[r][c] = val


def draw_wall(grid, wx, wy, ww, wh):
    """Dessine un mur (rectangle plein OCCUPIED)."""
    fill_rect(grid, wx, wy, ww, wh, OCCUPIED)


def draw_room_outline(grid, cx, cy, w, h, thickness=0.25):
    """Dessine les 4 murs d'une salle (contour seulement)."""
    t = thickness
    draw_wall(grid, cx,        cy + h/2, w,   t)       # nord
    draw_wall(grid, cx,        cy - h/2, w,   t)       # sud
    draw_wall(grid, cx - w/2,  cy,       t,   h)       # ouest
    draw_wall(grid, cx + w/2,  cy,       t,   h)       # est


def draw_door(grid, wx, wy, ww, wh):
    """Efface un passage (porte) dans un mur."""
    fill_rect(grid, wx, wy, ww, wh, FREE)


# ── Créer la grille (tout FREE) ───────────────────────────
grid = [[FREE] * W for _ in range(H)]

# ── Murs extérieurs hôpital ───────────────────────────────
draw_wall(grid,  0.0,  22.0, 30.0, 0.3)    # nord
draw_wall(grid,  0.0, -31.0, 30.0, 0.3)    # sud
draw_wall(grid, -15.0, -5.0, 0.3, 54.0)    # ouest
draw_wall(grid,  15.0, -5.0, 0.3, 54.0)    # est

# ── Pharmacie  (centre 1, 16 — 8×7m) ────────────────────
draw_wall(grid,  1.0,  12.3, 8.0, 0.25)    # mur sud pharmacie
draw_wall(grid,  5.1,  17.5, 0.25, 10.0)   # mur est
draw_wall(grid, -3.1,  17.5, 0.25, 10.0)   # mur ouest
# Porte sud pharmacie (ouverture 1.5m au centre)
draw_door(grid,  1.0,  12.3, 1.5, 0.4)

# ── Réanimation  (centre 11, 5.5 — 8×6m) ────────────────
draw_wall(grid, 11.0,   2.3, 8.0, 0.25)    # mur sud
draw_wall(grid, 11.0,   8.7, 8.0, 0.25)    # mur nord
draw_wall(grid,  7.1,   4.0, 0.25, 3.2)    # mur ouest partie basse
draw_wall(grid,  7.1,   7.5, 0.25, 2.4)    # mur ouest partie haute
# Porte (ouverture à y=5.5)
draw_door(grid,  7.1,   5.5, 0.4, 1.8)

# ── Urgences  (centre -5, -6.6 — 10×6m) ─────────────────
draw_wall(grid, -5.0,  -3.4, 10.0, 0.25)   # mur nord
draw_wall(grid, -5.0,  -9.8, 10.0, 0.25)   # mur sud
draw_wall(grid,  0.1,  -4.5, 0.25, 2.2)    # mur est partie haute
draw_wall(grid,  0.1,  -8.5, 0.25, 2.4)    # mur est partie basse
# Porte urgences
draw_door(grid,  0.1,  -6.6, 0.4, 1.8)

# ── Consultation  (centre -2, -27 — 8×6m) ───────────────
draw_wall(grid, -2.0, -24.0, 8.0, 0.25)    # mur nord
draw_wall(grid, -2.0, -30.0, 8.0, 0.25)    # mur sud
draw_wall(grid,  2.1, -25.5, 0.25, 3.0)    # mur est haute
draw_wall(grid,  2.1, -28.5, 0.25, 3.0)    # mur est basse
draw_wall(grid, -6.1, -27.0, 0.25, 6.0)    # mur ouest
# Porte consultation
draw_door(grid,  2.1, -27.0, 0.4, 1.8)

# ── Couloir principal (x=0, y=-7, largeur 3.6m) ─────────
draw_wall(grid, -1.8,  -7.0, 0.25, 32.0)   # mur ouest couloir
draw_wall(grid,  1.8,  -7.0, 0.25, 32.0)   # mur est couloir
# Ouvertures couloir (connexion entre zones)
draw_door(grid, -1.8,  12.3, 0.4, 1.5)
draw_door(grid,  1.8,  12.3, 0.4, 1.5)

# ── Obstacles divers ─────────────────────────────────────
draw_wall(grid,  1.0,   3.0, 0.6, 0.5)     # chariot1
draw_wall(grid, -0.5, -12.0, 0.6, 0.5)     # chariot2
draw_wall(grid, -1.2,   8.0, 0.4, 0.4)     # distributeur eau

# ── Écrire fichier PGM ───────────────────────────────────
output_dir = os.path.dirname(os.path.abspath(__file__))
pgm_path  = os.path.join(output_dir, "hospital_map.pgm")
yaml_path = os.path.join(output_dir, "hospital_map.yaml")

with open(pgm_path, "wb") as f:
    header = f"P5\n{W} {H}\n255\n"
    f.write(header.encode())
    for row in grid:
        f.write(bytes(row))

print(f"✅ PGM généré  : {pgm_path}  ({W}×{H} px, {RESOLUTION}m/px)")

# ── Écrire fichier YAML ──────────────────────────────────
yaml_content = f"""image: hospital_map.pgm
resolution: {RESOLUTION}
origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
with open(yaml_path, "w") as f:
    f.write(yaml_content)

print(f"✅ YAML généré : {yaml_path}")
print(f"   Origine  : ({ORIGIN_X}, {ORIGIN_Y})")
print(f"   Taille   : {MAP_W_M}m × {MAP_H_M}m")

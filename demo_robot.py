#!/usr/bin/env python3
"""
PharmaBot — Démonstration Visuelle Complète
Simule une journée hospitalière : EDF + Navigation + Livraisons + KPIs
Fonctionne sur Mac natif ET via Docker (terminal uniquement, pas besoin de Gazebo)
"""
import sys, os, time, random, threading, math

# ─── Couleurs ANSI ───────────────────────────────────────────────────────────
R  = "\033[91m";  G  = "\033[92m";  Y  = "\033[93m"
B  = "\033[94m";  M  = "\033[95m";  C  = "\033[96m"
W  = "\033[97m";  DIM= "\033[2m";   BLD= "\033[1m";  RST= "\033[0m"
BOLD = BLD; RED = R; GREEN = G; YELLOW = Y; BLUE = B; CYAN = C; MAGENTA = M

def clr(): os.system('clear' if os.name == 'posix' else 'cls')
def bar(pct, w=20, fg=G):
    filled = int(w * pct / 100)
    return fg + "█" * filled + DIM + "░" * (w - filled) + RST

# ─── Carte hôpital ASCII ─────────────────────────────────────────────────────
CARTE = [
    "  ┌──────────────────────────────────────────────────┐",
    "  │    🏥  HÔPITAL  IBN  TOFAÏL  — VUE  DU  SOL    │",
    "  ├─────────────┬────────────────┬────────────────────┤",
    "  │  RÉANIMATION│   URGENCES     │    CONSULTATION    │",
    "  │   (11,5.5)  │   (6,13)       │     (-2,-27)       │",
    "  │    🔴 HARD  │   🟡 SOFT      │    🟢 FIRM         │",
    "  │    30s max  │   120s max     │    300s max        │",
    "  ├─────────────┴────────────────┴────────────────────┤",
    "  │                    COULOIR                        │",
    "  ├────────────────────────────────────────────────────┤",
    "  │                PHARMACIE (16,0)                    │",
    "  │                  🏭 DÉPART                        │",
    "  └────────────────────────────────────────────────────┘",
]

# ─── Données ─────────────────────────────────────────────────────────────────
SALLES = {
    "reanimation":  {"nom": "Réanimation",  "x": 11.0, "y":  5.5, "type_rt": "HARD_RT", "deadline": 30,  "emoji": "🔴"},
    "urgences":     {"nom": "Urgences",     "x":  6.0, "y": 13.0, "type_rt": "SOFT_RT", "deadline": 120, "emoji": "🟡"},
    "consultation": {"nom": "Consultation", "x": -2.0, "y":-27.0, "type_rt": "FIRM_RT", "deadline": 300, "emoji": "🟢"},
}
MEDICAMENTS = {
    "reanimation":  ["Adrénaline 1mg", "Morphine 10mg IV", "Noradrénaline"],
    "urgences":     ["Morphine 10mg",  "Paracétamol IV 1g", "Kétamine 50mg"],
    "consultation": ["Amoxicilline 500mg", "Ibuprofène 400mg", "Metformine 500mg"],
}
MEDECINS = {
    "reanimation":  ["Dr. Alaoui",  "Dr. Benjelloun"],
    "urgences":     ["Dr. Driss",   "Dr. Fassi"],
    "consultation": ["Dr. Guessous","Dr. Hilali"],
}

# ─── État global de la démo ───────────────────────────────────────────────────
class Etat:
    def __init__(self):
        self.robot_x     = 16.0
        self.robot_y     = 0.0
        self.nav_state   = "IDLE"
        self.dept_cible  = None
        self.mission_act = None
        self.livraisons  = []
        self.missions_total   = 0
        self.missions_success = 0
        self.overrides        = 0
        self.obstacles        = 0
        self.file_attente     = []
        self.log              = []
        self.lock             = threading.Lock()

    def addlog(self, msg, color=W):
        ts = time.strftime("%H:%M:%S")
        with self.lock:
            self.log.append(f"{DIM}[{ts}]{RST} {color}{msg}{RST}")
            if len(self.log) > 12:
                self.log.pop(0)

etat = Etat()

# ─── Affichage ───────────────────────────────────────────────────────────────
def afficher(t0):
    clr()
    elapsed = time.time() - t0

    # ── Header ──
    print(f"\n{BLD}{CYAN}{'═'*60}{RST}")
    print(f"{BLD}{CYAN}   🤖  PHARMABOT — DÉMONSTRATION EN DIRECT{RST}")
    print(f"{DIM}   Ibn Tofaïl University | Systèmes Temps Réel | ROS2 Humble{RST}")
    print(f"{BLD}{CYAN}{'═'*60}{RST}\n")

    # ── Carte avec position robot ──
    rx = etat.robot_x; ry = etat.robot_y
    dept = etat.dept_cible
    salle_pos = SALLES.get(dept, {}) if dept else {}

    for i, ligne in enumerate(CARTE):
        if i == 8:  # ligne couloir — afficher robot
            # calculer position en %
            prog = ""
            if dept and salle_pos:
                dx = salle_pos["x"] - 16.0
                dy = salle_pos["y"] - 0.0
                dist_tot = math.sqrt(dx*dx + dy*dy)
                drx = rx - 16.0; dry = ry - 0.0
                dist_act = math.sqrt(drx*drx + dry*dry)
                pct = min(dist_act/dist_tot, 1.0) * 100 if dist_tot > 0 else 0
                prog = f"  {bar(pct,15,B)} {pct:.0f}%"
            nav_sym = {
                "IDLE":              f"{DIM}🤖 en attente{RST}",
                "PLANNING":          f"{Y}🤖 planification...{RST}",
                "MOVING":            f"{G}🤖➡️  navigation...{RST}",
                "AVOIDING_OBSTACLE": f"{R}🤖⚠️  évitement!{RST}",
                "REPLANNING":        f"{M}🤖🔄 replan...{RST}",
                "GOAL_REACHED":      f"{G}🤖✅ ARRIVÉ!{RST}",
                "RECOVERING":        f"{Y}🤖🔧 récupération{RST}",
                "FAILED":            f"{R}🤖❌ échec{RST}",
            }.get(etat.nav_state, f"🤖 {etat.nav_state}")
            print(f"{ligne}   {nav_sym}{prog}")
        else:
            print(ligne)

    print()

    # ── Mission active ──
    print(f"{BLD}  ┌── MISSION ACTIVE {'─'*38}┐{RST}")
    if etat.mission_act:
        m = etat.mission_act
        s = SALLES[m["dept"]]
        t_rest = max(0, m["deadline"] - (time.time() - m["t_start"]))
        pct_t  = t_rest / m["deadline"] * 100
        couleur_t = G if pct_t > 50 else Y if pct_t > 20 else R
        print(f"  │  {s['emoji']} {BLD}{m['dept'].upper()}{RST}  —  {CYAN}{m['med']}{RST}")
        print(f"  │  Médecin : {m['doc']}   │  Type : {BLD}{m['type_rt']}{RST}")
        print(f"  │  Temps restant : {couleur_t}{t_rest:.0f}s{RST} / {m['deadline']}s  "
              f"{bar(pct_t, 15, couleur_t)}")
        print(f"  │  État nav : {BLD}{etat.nav_state}{RST}  "
              f"→  Position : ({rx:.1f}, {ry:.1f})")
    else:
        print(f"  │  {DIM}Aucune mission en cours{RST}")
    print(f"{BLD}  └{'─'*49}┘{RST}\n")

    # ── File d'attente EDF ──
    with etat.lock:
        file = list(etat.file_attente)
    if file:
        print(f"{BLD}  File EDF ({len(file)} en attente) :{RST}")
        for i, f in enumerate(file[:3]):
            s = SALLES[f["dept"]]
            print(f"  {i+1}. {s['emoji']} {f['dept']:12s} — {f['type_rt']:8s} — {f['med']}")
        print()

    # ── KPIs ──
    print(f"{BLD}  ┌── KPIs TEMPS RÉEL {'─'*37}┐{RST}")
    total = etat.missions_total
    succ  = etat.missions_success
    rate  = (succ / total * 100) if total > 0 else 100.0
    c_rate = G if rate >= 95 else Y if rate >= 80 else R
    print(f"  │  success_rate      : {c_rate}{BLD}{rate:.1f}%{RST}  (KPI ≥ 95%)   "
          f"{bar(rate,15,c_rate)}")
    print(f"  │  missions          : {succ}/{total} succès")
    print(f"  │  overrides EDF     : {Y}{etat.overrides}{RST} interruptions HARD_RT")
    print(f"  │  obstacles         : {etat.obstacles} évités")
    print(f"{BLD}  └{'─'*49}┘{RST}\n")

    # ── Historique livraisons ──
    with etat.lock:
        livs = list(etat.livraisons[-4:])
    if livs:
        print(f"{BLD}  Dernières livraisons :{RST}")
        for lv in reversed(livs):
            s = SALLES[lv["dept"]]
            print(f"  {G}✅{RST} {s['emoji']} {lv['dept']:12s}  {CYAN}{lv['med']}{RST}  "
                  f"→  {G}{lv['temps']:.1f}s{RST} / {lv['deadline']}s")
        print()

    # ── Log messages ──
    print(f"{BLD}  Log système :{RST}")
    with etat.lock:
        logs = list(etat.log[-8:])
    for l in logs:
        print(f"  {l}")

    # ── Footer ──
    print(f"\n{DIM}  Durée démo : {elapsed:.0f}s  │  Ctrl+C pour quitter{RST}")


# ─── Simulation une mission ───────────────────────────────────────────────────
def simuler_mission(dept, med, doc):
    salle = SALLES[dept]
    t_start = time.time()

    etat.missions_total += 1
    etat.mission_act = {
        "dept": dept, "med": med, "doc": doc,
        "type_rt": salle["type_rt"], "deadline": salle["deadline"],
        "t_start": t_start,
    }
    etat.dept_cible = dept
    etat.addlog(f"📋 REQUÊTE  {salle['emoji']} {dept.upper()} — {med}", CYAN)
    etat.addlog(f"   Médecin: {doc}  |  Type: {salle['type_rt']}  |  Deadline: {salle['deadline']}s", DIM)
    time.sleep(0.3)

    # PLANNING
    etat.nav_state = "PLANNING"
    etat.addlog(f"🗺️  EDF calcule chemin → {salle['nom']} ({salle['x']}, {salle['y']})", Y)
    time.sleep(0.4)

    # MOVING — interpoler position vers destination
    etat.nav_state = "MOVING"
    steps = 20
    x0, y0 = 16.0, 0.0  # départ pharmacie
    xt, yt = salle["x"], salle["y"]
    etat.robot_x, etat.robot_y = x0, y0

    etat.addlog(f"🚀 Navigation MOVING → ({xt}, {yt})", G)

    for step in range(steps):
        # Obstacle aléatoire (10% de chance)
        if random.random() < 0.10 and step in (5, 10, 15):
            etat.nav_state = "AVOIDING_OBSTACLE"
            etat.obstacles += 1
            etat.addlog(f"⚠️  Obstacle détecté ! Évitement en cours...", R)
            time.sleep(0.4)
            etat.nav_state = "REPLANNING"
            etat.addlog(f"🔄 Replanification du chemin...", M)
            time.sleep(0.3)
            etat.nav_state = "MOVING"

        # Stuck aléatoire (3% de chance)
        if random.random() < 0.03:
            etat.nav_state = "RECOVERING"
            etat.addlog(f"🔧 Robot bloqué — récupération...", Y)
            time.sleep(0.5)
            etat.nav_state = "MOVING"

        prog = (step + 1) / steps
        etat.robot_x = x0 + (xt - x0) * prog
        etat.robot_y = y0 + (yt - y0) * prog
        time.sleep(0.12)

    # GOAL_REACHED
    etat.robot_x, etat.robot_y = xt, yt
    etat.nav_state = "GOAL_REACHED"
    temps = time.time() - t_start
    etat.missions_success += 1
    ok = temps <= salle["deadline"]
    with etat.lock:
        etat.livraisons.append({
            "dept": dept, "med": med, "temps": temps,
            "deadline": salle["deadline"], "ok": ok
        })

    if ok:
        etat.addlog(f"✅ LIVRAISON confirmée — {salle['nom']} — {temps:.1f}s /{salle['deadline']}s", G)
    else:
        etat.addlog(f"⚠️  Livraison TARDIVE — {temps:.1f}s > {salle['deadline']}s deadline", Y)

    time.sleep(0.5)

    # Retour pharmacie
    etat.nav_state = "MOVING"
    etat.addlog(f"↩️  Retour pharmacie...", DIM)
    for step in range(10):
        prog = (step + 1) / 10
        etat.robot_x = xt + (16.0 - xt) * prog
        etat.robot_y = yt + (0.0  - yt) * prog
        time.sleep(0.06)

    etat.robot_x, etat.robot_y = 16.0, 0.0
    etat.nav_state  = "IDLE"
    etat.dept_cible = None
    etat.mission_act = None
    time.sleep(0.3)


# ─── Scénario démo ───────────────────────────────────────────────────────────
SCENARIO = [
    # (dept, médicament, médecin)
    ("consultation", "Amoxicilline 500mg",   "Dr. Guessous"),
    ("urgences",     "Paracétamol IV 1g",    "Dr. Driss"),
    ("reanimation",  "Adrénaline 1mg",       "Dr. Alaoui"),     # HARD_RT → override
    ("consultation", "Ibuprofène 400mg",     "Dr. Hilali"),
    ("urgences",     "Morphine 10mg",        "Dr. Fassi"),
    ("reanimation",  "Morphine 10mg IV",     "Dr. Benjelloun"), # 2ème HARD_RT
    ("consultation", "Metformine 500mg",     "Dr. Guessous"),
    ("urgences",     "Kétamine 50mg",        "Dr. Driss"),
]


def thread_affichage(t0, stop_evt):
    while not stop_evt.is_set():
        afficher(t0)
        time.sleep(0.25)


def main():
    t0 = time.time()
    stop_evt = threading.Event()

    # Démarrer le thread d'affichage
    th_aff = threading.Thread(target=thread_affichage, args=(t0, stop_evt), daemon=True)
    th_aff.start()

    etat.addlog("🏥 PharmaBot démarré — Simulation hôpital Ibn Tofaïl", CYAN)
    etat.addlog("   EDF Scheduler actif | Navigation P3 prête | Phase 4 en ligne", DIM)
    time.sleep(1.5)

    try:
        for i, (dept, med, doc) in enumerate(SCENARIO):
            salle = SALLES[dept]

            # Simuler un override HARD_RT si la 3ème mission arrive pendant une SOFT_RT
            if i == 2 and etat.nav_state != "IDLE":
                etat.overrides += 1
                etat.addlog(f"⚡ HARD_RT OVERRIDE! Réanimation interrompt la mission en cours!", R)
                time.sleep(0.5)

            # Ajouter à la file
            with etat.lock:
                etat.file_attente.append({"dept": dept, "med": med, "type_rt": salle["type_rt"]})

            # Trier par EDF (deadline croissante)
            with etat.lock:
                etat.file_attente.sort(key=lambda x: SALLES[x["dept"]]["deadline"])

            time.sleep(0.2)

            # Retirer de la file et naviguer
            with etat.lock:
                if etat.file_attente:
                    etat.file_attente.pop(0)

            simuler_mission(dept, med, doc)

    except KeyboardInterrupt:
        pass
    finally:
        stop_evt.set()
        time.sleep(0.3)
        clr()

    # ── Rapport final ──
    print(f"\n{BLD}{CYAN}{'═'*60}{RST}")
    print(f"{BLD}{CYAN}   🏆  RAPPORT FINAL — JOURNÉE HOSPITALIÈRE{RST}")
    print(f"{BLD}{CYAN}{'═'*60}{RST}\n")

    total  = etat.missions_total
    succ   = etat.missions_success
    rate   = succ / total * 100 if total > 0 else 0

    print(f"  {BLD}Missions totales   :{RST} {total}")
    print(f"  {BLD}Missions réussies  :{RST} {G}{succ}{RST}")
    print(f"  {BLD}Success rate       :{RST} {G}{BLD}{rate:.1f}%{RST}  {'✅' if rate >= 95 else '⚠️ '} (KPI ≥ 95%)")
    print(f"  {BLD}Overrides EDF      :{RST} {Y}{etat.overrides}{RST} interruptions HARD_RT")
    print(f"  {BLD}Obstacles évités   :{RST} {etat.obstacles}")
    print()
    print(f"  {BLD}Détail livraisons :{RST}")
    for lv in etat.livraisons:
        s = SALLES[lv["dept"]]
        status = f"{G}✅ OK{RST}" if lv["ok"] else f"{Y}⚠️  TARDIF{RST}"
        print(f"  {s['emoji']} {lv['dept']:12s}  {lv['temps']:5.1f}s / {lv['deadline']}s  {status}  {CYAN}{lv['med']}{RST}")

    print(f"\n{G}{BLD}  🎉 Démo terminée — PharmaBot Phase 1+2+3+4 opérationnel !{RST}\n")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
send_requete_medecin.py — Simulateur de requêtes médecin (démo jury)
PharmaBot | Ibn Tofaïl University | 2025-2026

Exécuter DANS LA VM dans un 2e terminal après lancement de la simulation :
    python3 send_requete_medecin.py

Menu interactif pour envoyer des requêtes au robot avec différentes priorités.
"""
import json
import random
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

BOLD   = "\033[1m"
RED    = "\033[91m"
ORANGE = "\033[93m"
GREEN  = "\033[92m"
CYAN   = "\033[96m"
BLUE   = "\033[94m"
RESET  = "\033[0m"

MEDICAMENTS = {
    "reanimation": [
        "Adrénaline 1mg IV",
        "Morphine 10mg IV",
        "Noradrénaline 4mg",
        "Atropine 0.5mg",
        "Propofol 200mg",
    ],
    "urgences": [
        "Morphine 10mg",
        "Paracétamol IV 1g",
        "Ibuprofène 400mg",
        "Ondansétron 4mg",
        "Lorazépam 2mg",
    ],
    "consultation": [
        "Amoxicilline 500mg",
        "Paracétamol 1g",
        "Oméprazole 20mg",
        "Metformine 500mg",
        "Lisinopril 10mg",
    ],
}

TYPES_RT = {
    "reanimation":  {"type": "HARD_RT", "deadline": 30,  "symbol": f"{RED}🔴 HARD_RT{RESET}"},
    "urgences":     {"type": "SOFT_RT", "deadline": 120, "symbol": f"{ORANGE}🟡 SOFT_RT{RESET}"},
    "consultation": {"type": "FIRM_RT", "deadline": 300, "symbol": f"{GREEN}🟢 FIRM_RT{RESET}"},
}


class DocteurNode(Node):
    def __init__(self):
        super().__init__("docteur_demo")
        self._pub = self.create_publisher(String, "/pharmabot/requete", 10)
        self._nb  = 0
        self.get_logger().info("Nœud médecin connecté")

    def envoyer(self, dept: str, medicament: str = None):
        self._nb += 1
        if medicament is None:
            medicament = random.choice(MEDICAMENTS[dept])
        rt   = TYPES_RT[dept]
        msg  = String()
        msg.data = json.dumps({
            "requete_id":   self._nb,
            "departement":  dept,
            "medicament":   medicament,
            "type_rt":      rt["type"],
            "deadline_sec": rt["deadline"],
            "timestamp":    time.time(),
        })
        self._pub.publish(msg)
        return medicament


def menu():
    print(f"\n{BOLD}{BLUE}╔═══════════════════════════════════════════════╗{RESET}")
    print(f"{BOLD}{BLUE}║   PharmaBot — Simulateur Requêtes Médecin     ║{RESET}")
    print(f"{BOLD}{BLUE}╚═══════════════════════════════════════════════╝{RESET}\n")
    print(f"  {RED}[1]{RESET} 🔴 Réanimation   — HARD_RT  ≤ 30s  {RED}(URGENT){RESET}")
    print(f"  {ORANGE}[2]{RESET} 🟡 Urgences      — SOFT_RT  ≤ 120s")
    print(f"  {GREEN}[3]{RESET} 🟢 Consultation  — FIRM_RT  ≤ 300s")
    print(f"  {CYAN}[4]{RESET} 🎭 Scénario DÉMO — 3 requêtes automatiques")
    print(f"  {CYAN}[5]{RESET} 🔁 Stress test   — Envoi rapide 5 requêtes")
    print(f"  {RED}[0]{RESET} ✖  Quitter\n")


def scenario_demo(node: DocteurNode):
    """Scénario de démo : Consultation → Urgences → Réanimation (EDF override visible)."""
    print(f"\n{CYAN}{BOLD}═══ SCÉNARIO DÉMO ═══{RESET}")

    print(f"\n{GREEN}[1/3] Consultation FIRM_RT...{RESET}")
    med = node.envoyer("consultation")
    print(f"      ✓ Envoyé : {med}")
    time.sleep(3)

    print(f"\n{ORANGE}[2/3] Urgences SOFT_RT...{RESET}")
    med = node.envoyer("urgences")
    print(f"      ✓ Envoyé : {med}")
    time.sleep(3)

    print(f"\n{RED}[3/3] Réanimation HARD_RT — EDF OVERRIDE !{RESET}")
    med = node.envoyer("reanimation")
    print(f"      ✓ Envoyé : {med}")
    print(f"\n{BOLD}→ Le robot doit interrompre sa mission et aller en Réanimation !{RESET}")


def stress_test(node: DocteurNode):
    """5 requêtes en 5 secondes pour tester EDF."""
    depts = ["consultation", "urgences", "reanimation", "urgences", "consultation"]
    print(f"\n{CYAN}{BOLD}═══ STRESS TEST — 5 REQUÊTES ═══{RESET}")
    for i, dept in enumerate(depts):
        med = node.envoyer(dept)
        rt  = TYPES_RT[dept]
        print(f"  [{i+1}/5] {rt['symbol']} {dept:12} | {med}")
        time.sleep(1)
    print(f"\n{BOLD}→ EDF doit ordonner par deadline croissante{RESET}")


def main():
    rclpy.init()
    node = DocteurNode()

    # Pause pour que ROS2 soit prêt
    time.sleep(1)

    print(f"\n{BOLD}{CYAN}PharmaBot — Simulateur Médecin{RESET}")
    print(f"Publié sur : {BOLD}/pharmabot/requete{RESET}\n")

    while rclpy.ok():
        menu()
        try:
            choix = input(f"  {BOLD}Votre choix : {RESET}").strip()
        except (KeyboardInterrupt, EOFError):
            break

        if choix == "0":
            break
        elif choix == "1":
            med = node.envoyer("reanimation")
            rt  = TYPES_RT["reanimation"]
            print(f"\n  {RED}✓ Envoyé → Réanimation | HARD_RT ≤ 30s | {med}{RESET}")
        elif choix == "2":
            med = node.envoyer("urgences")
            print(f"\n  {ORANGE}✓ Envoyé → Urgences | SOFT_RT ≤ 120s | {med}{RESET}")
        elif choix == "3":
            med = node.envoyer("consultation")
            print(f"\n  {GREEN}✓ Envoyé → Consultation | FIRM_RT ≤ 300s | {med}{RESET}")
        elif choix == "4":
            scenario_demo(node)
        elif choix == "5":
            stress_test(node)
        else:
            print(f"  {RED}Choix invalide{RESET}")

        # Spinner ROS2 bref
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.5)

    print(f"\n{GREEN}Au revoir !{RESET}\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

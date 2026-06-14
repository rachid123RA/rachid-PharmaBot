#!/bin/bash
# ═══════════════════════════════════════════════════════════════
#  PharmaBot — Lanceur macOS (sans VM, sans Linux)
#  Compatible Apple Silicon M1/M2/M3/M4 — macOS 14/15
# ═══════════════════════════════════════════════════════════════

BOLD="\033[1m"
GREEN="\033[32m"
BLUE="\033[34m"
YELLOW="\033[33m"
RED="\033[31m"
RESET="\033[0m"

clear
echo -e "${BOLD}${BLUE}"
echo "  ██████╗ ██╗  ██╗ █████╗ ██████╗ ███╗   ███╗ █████╗ ██████╗  ██████╗ ████████╗"
echo "  ██╔══██╗██║  ██║██╔══██╗██╔══██╗████╗ ████║██╔══██╗██╔══██╗██╔═══██╗╚══██╔══╝"
echo "  ██████╔╝███████║███████║██████╔╝██╔████╔██║███████║██████╔╝██║   ██║   ██║   "
echo "  ██╔═══╝ ██╔══██║██╔══██║██╔══██╗██║╚██╔╝██║██╔══██║██╔══██╗██║   ██║   ██║   "
echo "  ██║     ██║  ██║██║  ██║██║  ██║██║ ╚═╝ ██║██║  ██║██████╔╝╚██████╔╝   ██║   "
echo "  ╚═╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝╚═╝  ╚═╝╚═════╝  ╚═════╝    ╚═╝   "
echo -e "${RESET}"
echo -e "  ${BOLD}Robot Autonome de Livraison Pharmaceutique${RESET}"
echo -e "  ${YELLOW}Ibn Tofaïl University — Phase 1+2+3+4${RESET}"
echo ""
echo "═══════════════════════════════════════════════════════════"
echo ""

# ── Détecter le mode de lancement ──────────────────────────────
echo -e "${BOLD}Choisissez le mode de lancement :${RESET}"
echo ""
echo "  ${GREEN}[1]${RESET} 🧪 Tests Phase 1+2   — Python natif (SANS Docker)"
echo "  ${GREEN}[2]${RESET} 🧪 Tests Phase 3+4   — Python natif (SANS Docker)"
echo "  ${GREEN}[3]${RESET} 🧪 Tous les tests    — Python natif (SANS Docker)"
echo ""
echo "  ${BLUE}[4]${RESET} 🐳 Tests P1+P2       — Via Docker"
echo "  ${BLUE}[5]${RESET} 🐳 Tests P3+P4       — Via Docker"
echo "  ${BLUE}[6]${RESET} 🐳 Shell ROS2        — Via Docker (interactif)"
echo ""
echo "  ${YELLOW}[7]${RESET} ℹ️  Voir l'état du projet"
echo "  ${RED}[0]${RESET} ✖  Quitter"
echo ""
echo "═══════════════════════════════════════════════════════════"
read -p "  Votre choix : " choix

case $choix in

  1)
    echo ""
    echo -e "${GREEN}▶ Tests Phase 1+2 (Python natif)${RESET}"
    echo ""
    python3 tests/run_tests_phase1_phase2.py -v
    ;;

  2)
    echo ""
    echo -e "${GREEN}▶ Tests Phase 3+4 (Python natif)${RESET}"
    echo ""
    python3 tests/run_tests_phase3_phase4.py -v
    ;;

  3)
    echo ""
    echo -e "${GREEN}▶ Tous les tests (Python natif)${RESET}"
    echo ""
    echo "════ PHASE 1+2 ════"
    python3 tests/run_tests_phase1_phase2.py -v
    echo ""
    echo "════ PHASE 3+4 ════"
    python3 tests/run_tests_phase3_phase4.py -v
    ;;

  4)
    echo ""
    echo -e "${BLUE}▶ Tests Phase 1+2 via Docker${RESET}"
    if ! command -v docker &>/dev/null; then
      echo -e "${RED}❌ Docker n'est pas installé.${RESET}"
      echo "   Télécharger : https://desktop.docker.com/mac/main/arm64/Docker.dmg"
    else
      docker compose run --rm pharmabot-tests
    fi
    ;;

  5)
    echo ""
    echo -e "${BLUE}▶ Tests Phase 3+4 via Docker${RESET}"
    if ! command -v docker &>/dev/null; then
      echo -e "${RED}❌ Docker n'est pas installé.${RESET}"
      echo "   Télécharger : https://desktop.docker.com/mac/main/arm64/Docker.dmg"
    else
      docker compose run --rm pharmabot-p34
    fi
    ;;

  6)
    echo ""
    echo -e "${BLUE}▶ Shell ROS2 interactif via Docker${RESET}"
    echo -e "   ${YELLOW}Tapez 'exit' pour quitter${RESET}"
    if ! command -v docker &>/dev/null; then
      echo -e "${RED}❌ Docker n'est pas installé.${RESET}"
      echo "   Télécharger : https://desktop.docker.com/mac/main/arm64/Docker.dmg"
    else
      docker compose run --rm pharmabot-shell
    fi
    ;;

  7)
    echo ""
    echo -e "${BOLD}État du projet PharmaBot${RESET}"
    echo ""
    echo "  📁 Dossier    : $(pwd)"
    echo "  🐍 Python     : $(python3 --version 2>&1)"
    echo "  🐳 Docker     : $(docker --version 2>/dev/null || echo 'Non installé')"
    echo "  📦 Nœuds      : 17 (Phase 1+2+3+4)"
    echo "  🧪 Tests P1+2 : 10 scénarios"
    echo "  🧪 Tests P3+4 : 22 scénarios"
    echo ""
    echo "  🔗 GitHub     : https://github.com/rachid123RA/rachid-PharmaBot"
    echo ""
    ;;

  0)
    echo ""
    echo -e "${YELLOW}Au revoir !${RESET}"
    echo ""
    exit 0
    ;;

  *)
    echo -e "${RED}Choix invalide.${RESET}"
    ;;

esac

echo ""
echo "═══════════════════════════════════════════════════════════"

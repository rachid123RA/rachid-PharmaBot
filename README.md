# 🤖 PharmaBot — Robot Autonome de Livraison Pharmaceutique

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble%20Hawksbill-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-green?logo=python)](https://python.org)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04%20LTS-orange?logo=ubuntu)](https://ubuntu.com)
[![Tests](https://img.shields.io/badge/Tests-22%2F22%20PASS-brightgreen)](#tests)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)
[![Phase](https://img.shields.io/badge/Phase-1%20%2B%202%20%2B%203%20%2B%204-purple)](#phases)

> **Système temps réel autonome** pour la livraison de médicaments en milieu hospitalier, développé avec ROS2 Humble, Gazebo Classic et un agent RL PPO (Stable-Baselines3).

**Équipe PharmaBot — Ibn Tofaïl University | 2024-2025**
**Encadrant : Prof. Khaoula Boukir — Systèmes Embarqués Temps Réel**

---

## 📋 Table des Matières

1. [Vue d'ensemble](#vue-densemble)
2. [Architecture](#architecture)
3. [Installation](#installation)
4. [Phase 1 — Nœuds RT Fondamentaux](#phase-1)
5. [Phase 2 — Intégration Pharmacie & Monitoring](#phase-2)
6. [Phase 3 — Navigation Autonome](#phase-3)
7. [Phase 4 — Perception & Dashboard Avancé](#phase-4)
8. [Topics ROS2](#topics-ros2)
9. [Tests (22/22 PASS)](#tests)
10. [KPIs & Métriques](#kpis)
11. [Structure du Projet](#structure)
12. [Lancer avec Docker](#docker)
13. [Lancer avec ROS2 + Gazebo](#ros2-gazebo)

---

## 🎯 Vue d'ensemble

PharmaBot automatise la livraison de médicaments dans un hôpital simulé sous Gazebo. Le robot **Pioneer 3AT** navigue de la pharmacie vers les différents services (Réanimation, Urgences, Consultation) en respectant des contraintes temps réel strictes :

| Priorité | Service | Délai max | Type RT |
|----------|---------|-----------|---------|
| 🔴 HARD | Réanimation | 30 secondes | `HARD_RT` |
| 🟡 SOFT | Urgences | 120 secondes | `SOFT_RT` |
| 🟢 FIRM | Consultation | 300 secondes | `FIRM_RT` |

Le système implémente **3 algorithmes d'ordonnancement temps réel** :
- **EDF** (Earliest Deadline First) — `rt_scheduler.py`
- **RMS** (Rate Monotonic Scheduling) — `mission_manager.py`
- **DMA** (Deadline Monotonic Algorithm) — `dma_tasks.py`

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     SYSTÈME PHARMABOT                           │
│                                                                 │
│  ┌──────────┐   /pharmabot/requete    ┌─────────────────┐      │
│  │ Médecin  │ ─────────────────────▶ │  EDF Scheduler  │      │
│  │  Node    │                         │  (rt_scheduler) │      │
│  └──────────┘                         └────────┬────────┘      │
│                                                │                │
│                              /pharmabot/tache_courante          │
│                                                ▼                │
│  ┌──────────────┐           ┌─────────────────────────┐         │
│  │  Pharmacist  │ ◀──────── │    Mission Manager      │         │
│  │    Node      │           │  (machine à états RMS)  │         │
│  └──────┬───────┘           └──────────┬──────────────┘         │
│         │ /pharmabot/                  │                         │
│         │  pharmacist_ok    /pharmabot/etat_robot               │
│         ▼                             ▼                         │
│  ┌──────────────┐           ┌─────────────────────────┐         │
│  │  DMA Tasks   │           │   Navigation Bridge     │         │
│  │  (pipeline)  │           │  (navigation_bridge)    │         │
│  └──────────────┘           └──────────┬──────────────┘         │
│                                        │                         │
│                           /pharmabot/navigation_goal             │
│                                        ▼                         │
│  ┌──────────────┐           ┌─────────────────────────┐         │
│  │   Watchdog   │           │   Pioneer 3AT Robot     │         │
│  │    Node      │           │   (Gazebo Simulation)   │         │
│  └──────────────┘           └──────────┬──────────────┘         │
│                                        │                         │
│                          /pharmabot/livraison_confirmee          │
│                                        ▼                         │
│  ┌──────────────┐           ┌─────────────────────────┐         │
│  │  Perception  │           │  Delivery Detector      │         │
│  │    Node P4   │           │  (delivery_detector)    │         │
│  └──────────────┘           └─────────────────────────┘         │
│  ┌──────────────────────────────────────────────────────┐        │
│  │  Visual Dashboard P4  (20 topics en temps réel)      │        │
│  └──────────────────────────────────────────────────────┘        │
└─────────────────────────────────────────────────────────────────┘

Phase 1 ── Nœuds RT de base + Navigation Bridge + Delivery Detector
Phase 2 ── Pharmacist Node + DMA Tasks + Watchdog + Dashboard
Phase 3 ── Autonomous Navigator (8 états) + Navigation Watchdog P3
Phase 4 ── Perception Node (QR/ArUco + détection humaine) + Dashboard P4
```

---

## 🚀 Installation

### Prérequis

- **Ubuntu 22.04 LTS** (ARM64 ou x86_64)
- **ROS2 Humble Hawksbill**
- **Python 3.10+**
- **Gazebo Classic** (optionnel, pour simulation 3D)

### 1. Installer ROS2 Humble

```bash
# Configurer les sources apt
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions -y
```

### 2. Installer les dépendances Python

```bash
pip install stable-baselines3==2.3.2 gymnasium==0.29.1 \
            tensorboard optuna==3.6.1 "numpy<2.0"
```

### 3. Installer Gazebo Classic (pour simulation complète)

```bash
sudo apt install gazebo ros-humble-gazebo-ros-pkgs -y
```

### 4. Builder le package ROS2

```bash
# Cloner le dépôt
git clone https://github.com/rachid123RA/rachid-PharmaBot.git
cd rachid-PharmaBot

# Sourcer ROS2
source /opt/ros/humble/setup.bash

# Builder
cd hospital_robot_spawner
pip install -e .
cd ..
colcon build --packages-select hospital_robot_spawner
source install/setup.bash
```

### 5. Tester sans ROS2 (tests autonomes)

```bash
# Tests Phase 1+2 (10 scénarios)
python3 tests/run_tests_phase1_phase2.py

# Tests Phase 3+4 (22 scénarios)
python3 tests/run_tests_phase3_phase4.py

# Mode verbose
python3 tests/run_tests_phase3_phase4.py -v
```

---

## 📦 Phase 1 — Nœuds RT Fondamentaux

**Objectif** : Implémenter le pipeline temps réel de base avec ordonnancement EDF.

### Nœuds Phase 1

| Nœud | Fichier | Rôle |
|------|---------|------|
| `rt_scheduler` | `rt_scheduler.py` | Ordonnancement EDF (Earliest Deadline First) |
| `mission_manager` | `mission_manager.py` | Machine à états + RMS |
| `nav_bridge` | `navigation_bridge.py` | Pont EDF ↔ Robot Pioneer 3AT |
| `delivery_detect` | `delivery_detector.py` | Détection et confirmation livraison |
| `doctor_request` | `doctor_request_node.py` | Générateur de requêtes médicales |

### EDF Scheduler (`rt_scheduler.py`)

L'ordonnanceur EDF trie les tâches par **deadline croissante** et peut **préempter** une tâche SOFT_RT ou FIRM_RT si une tâche HARD_RT arrive :

```python
# Priorités deadline (secondes)
DEADLINES = {
    "HARD_RT": 30,    # Réanimation — critique
    "SOFT_RT": 120,   # Urgences — important
    "FIRM_RT": 300,   # Consultation — normal
}

# Topics publiés
/pharmabot/tache_courante    # Mission active → Navigation Bridge
/pharmabot/edf_interruption  # Override HARD_RT → Mission Manager
```

### Machine à États — Mission Manager

```
IDLE ──────▶ PLANIFICATION ──▶ NAVIGATION ──▶ LIVRAISON ──▶ IDLE
              │                     │
              ▼                     ▼
         URGENCE_OVERRIDE      RECUPERATION
              │
              ▼
           NAVIGATION (reprise)
```

### Correctif Bug S06

**Problème** : La méthode `_cb_livraison_confirmee` pouvait terminer une mission sur un simple message de confirmation, même si la mission active avait changé entretemps.

**Solution** : Ajout de 3 gardes de sécurité :
1. La mission courante doit exister (`self._mission_courante is not None`)
2. Le département livré doit correspondre à la mission active
3. L'état machine doit être `NAVIGATION`, `LIVRAISON` ou `URGENCE_OVERRIDE`

---

## 💊 Phase 2 — Intégration Pharmacie & Monitoring

**Objectif** : Ajouter la validation pharmacien, le pipeline DMA et la supervision système.

### Nœuds Phase 2

| Nœud | Fichier | Rôle |
|------|---------|------|
| `pharmacist` | `pharmacist_node.py` | Validation stock + confirmation chargement |
| `dma_tasks` | `dma_tasks.py` | Pipeline DMA (Deadline Monotonic) |
| `watchdog_node` | `watchdog_node.py` | Surveillance santé système |
| `dashboard` | `dashboard.py` | Tableau de bord 13 topics |

### DMA Pipeline (`dma_tasks.py`)

Le pipeline DMA exécute des tâches périodiques avec priorités fixes par ordre de deadline :

```
Étape 1 → CHARGEMENT     (priorité DMA-1, deadline 5s)
Étape 2 → VERIFICATION   (priorité DMA-2, deadline 10s)
Étape 3 → SCELLAGE        (priorité DMA-3, deadline 15s)
Étape 4 → TRANSFERT       (priorité DMA-4, deadline 20s)
```

### Pharmacist Node (`pharmacist_node.py`)

Flux de validation :
1. Reçoit mission sur `/pharmabot/tache_courante`
2. Vérifie disponibilité stock médicament
3. Publie confirmation sur `/pharmabot/pharmacist_ok`
4. DMA Tasks démarre le pipeline de chargement

---

## 🧭 Phase 3 — Navigation Autonome

**Objectif** : Navigation SLAM autonome avec machine à états 8 niveaux et agent RL PPO backup.

### Nœuds Phase 3

| Nœud | Fichier | Rôle |
|------|---------|------|
| `autonomous_nav` | `autonomous_navigator.py` | Navigateur autonome 8 états |
| `nav_watchdog_p3` | `navigation_watchdog_p3.py` | Watchdog navigation P3 |

### Machine à États Navigation (8 États)

```
                    ┌─────────┐
                    │  IDLE   │
                    └────┬────┘
                         │ mission reçue
                         ▼
                  ┌─────────────┐
                  │  PLANNING   │◀──── recalcul chemin
                  └──────┬──────┘
                         │ chemin calculé
                         ▼
                  ┌─────────────┐      obstacle détecté
                  │   MOVING    │ ──────────────────────▶ ┌──────────────────┐
                  └──────┬──────┘                          │ AVOIDING_OBSTACLE│
                         │                                 └────────┬─────────┘
                         │ arrivée                                  │ dégagé
                         ▼                                          │
                  ┌─────────────┐                                   │
                  │GOAL_REACHED │◀──────────────────────────────────┘
                  └──────┬──────┘
                         │ reset
                         ▼
                       IDLE
                  
        HARD_RT reçu ──▶ REPLANNING (recalcul avec nouvelle priorité)
        échec navigation ──▶ FAILED ──▶ RECOVERING ──▶ PLANNING
```

### Navigation Watchdog P3 (`navigation_watchdog_p3.py`)

Surveille la progression du robot :
- **Timeout** : Si le robot ne progresse pas en 30s → `RECUPERATION`
- **Blocage** : Si même position depuis 15s → force `REPLANNING`
- **Heartbeat** : Publie état sur `/pharmabot/nav_watchdog_p3` toutes les 5s

### Agent RL PPO (Backup Navigation)

L'agent PPO (Proximal Policy Optimization) est entraîné via `start_training.py` :

```bash
# Lancer l'entraînement (6 générations disponibles dans rl_models/)
python3 -m hospital_robot_spawner.start_training

# Utiliser l'agent entraîné
ros2 run hospital_robot_spawner trained_agent
```

Espace d'observation : `[dx, dy, lidar_12_secteurs]` (14 dimensions)
Espace d'action : `[vitesse_lin, vitesse_ang]` (continu)

---

## 👁️ Phase 4 — Perception & Dashboard Avancé

**Objectif** : Ajouter la perception visuelle (QR/ArUco, détection humaine) et un dashboard enrichi.

### Nœuds Phase 4

| Nœud | Fichier | Rôle |
|------|---------|------|
| `perception_node` | `perception_node.py` | Détection QR/ArUco + humains |
| `dashboard_p4` | `visual_dashboard_p4.py` | Dashboard 20 topics en temps réel |

### Perception Node (`perception_node.py`)

Le nœud de perception gère 3 types de détection :

**1. Détection de salle (QR/ArUco)**
```
/pharmabot/room_detected  →  {"room": "REANIMATION", "confidence": 0.97}
```
- Identification automatique des salles par marqueurs visuels
- Précision cible : > 95%

**2. Détection humaine**
```
/pharmabot/human_detected  →  {"humans": [{"distance": 1.2, "angle": 15}]}
```
- Zone de sécurité : arrêt si humain < 0.5m
- Ralentissement si humain entre 0.5m et 2m

**3. Confirmation visuelle de livraison**
```
/pharmabot/visual_delivery_confirm  →  {"confirmed": true, "accuracy": 0.98}
```

### Visual Dashboard P4 (`visual_dashboard_p4.py`)

Tableau de bord enrichi avec **20 topics** surveillés :

```
╔══════════════════════════════════════════════════════╗
║           PHARMABOT — DASHBOARD PHASE 4              ║
╠══════════════════════════════════════════════════════╣
║  État Robot        : NAVIGATION                      ║
║  Mission Active    : HARD_RT → REANIMATION           ║
║  Temps restant     : 18.3s / 30.0s                   ║
║  Avancement        : ████████░░ 80%                  ║
╠══════════════════════════════════════════════════════╣
║  Navigation P3     : MOVING                          ║
║  Obstacle détecté  : NON                             ║
║  Détection humaine : NON                             ║
║  Salle détectée    : COULOIR_A                       ║
╠══════════════════════════════════════════════════════╣
║  KPIs en temps réel                                  ║
║  success_rate      : 100.0%  ✅                       ║
║  obstacle_avoid    : 100.0%  ✅                       ║
║  delivery_accuracy : 98.4%   ✅                       ║
║  nav_recovery      : 100.0%  ✅                       ║
╚══════════════════════════════════════════════════════╝
```

---

## 📡 Topics ROS2

### Topics Phase 1+2

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/pharmabot/requete` | String/JSON | doctor → scheduler | Requête médicament |
| `/pharmabot/tache_courante` | String/JSON | scheduler → bridge | Mission EDF active |
| `/pharmabot/navigation_goal` | String/JSON | bridge → detector | Coordonnées cible |
| `/pharmabot/nav_status` | String | bridge → DMA | Arrivée destination |
| `/pharmabot/livraison_confirmee` | String/JSON | detector → scheduler | Fin de mission |
| `/pharmabot/edf_interruption` | String/JSON | scheduler → MM | Override HARD_RT |
| `/pharmabot/etat_robot` | String | MM → tous | État machine |
| `/pharmabot/dma_pipeline` | String | DMA → MM | Étapes pipeline |
| `/pharmabot/watchdog` | String/JSON | watchdog → tous | Récupération/Reprise |
| `/pharmabot/pharmacist_ok` | String/JSON | pharmacist → DMA | Validation chargement |

### Topics Phase 3

| Topic | Direction | Description |
|-------|-----------|-------------|
| `/pharmabot/nav_state_p3` | autonomous_nav → tous | État navigation (8 niveaux) |
| `/pharmabot/nav_goal_p3` | MM → autonomous_nav | Objectif navigation |
| `/pharmabot/nav_watchdog_p3` | watchdog_p3 → tous | Heartbeat + alertes |
| `/pharmabot/obstacle_alert` | autonomous_nav → tous | Obstacle détecté |

### Topics Phase 4

| Topic | Direction | Description |
|-------|-----------|-------------|
| `/pharmabot/room_detected` | perception → tous | Salle identifiée (QR/ArUco) |
| `/pharmabot/human_detected` | perception → tous | Humain détecté + distance |
| `/pharmabot/visual_delivery_confirm` | perception → MM | Confirmation visuelle livraison |
| `/pharmabot/perception_status` | perception → dashboard | Statut capteurs |
| `/pharmabot/dashboard_p4` | dashboard → logs | Métriques agrégées |
| `/pharmabot/kpi_report` | dashboard → tous | Rapport KPIs temps réel |

---

## 🧪 Tests (22/22 PASS)

Le projet inclut **deux suites de tests autonomes** (aucune dépendance ROS2 ou Gazebo requise).

### Suite Phase 1+2 — 10 scénarios

```bash
python3 tests/run_tests_phase1_phase2.py [-v] [S01..S10]
```

| ID | Scénario | Type RT | Résultat |
|----|----------|---------|----------|
| S01 | Mission simple Consultation | FIRM_RT | ✅ PASS |
| S02 | 3 SOFT_RT décalées — tri EDF | SOFT_RT | ✅ PASS |
| S03 | HARD_RT pendant FIRM_RT — Override | HARD_RT | ✅ PASS |
| S04 | Deadline HARD_RT dépassée | HARD_RT | ✅ PASS |
| S05 | Perte de communication | — | ✅ PASS |
| S06 | Absence confirmation livraison | SOFT_RT | ✅ PASS |
| S07 | Panne robot pendant navigation | — | ✅ PASS |
| S08 | Reprise automatique après override | HARD_RT | ✅ PASS |
| S09 | Surcharge 25 requêtes simultanées | Mix | ✅ PASS |
| S10 | Journée hospitalière complète | Mix | ✅ PASS |

### Suite Phase 3+4 — 22 scénarios

```bash
python3 tests/run_tests_phase3_phase4.py [-v] [S11..S22]
```

| ID | Scénario | KPI testé | Résultat |
|----|----------|-----------|----------|
| S11 | Navigation GOAL_REACHED simple | success_rate | ✅ PASS |
| S12 | Watchdog RECUPERATION → FAILED | robustesse | ✅ PASS |
| S13 | HARD_RT interrompt SOFT_RT | interrupt | ✅ PASS |
| S14 | Navigation chemin optimal | ERT < 2s | ✅ PASS |
| S15 | Obstacle unique évité | obstacle_avoid | ✅ PASS |
| S16 | 10 missions success_rate > 95% | success_rate | ✅ PASS |
| S17 | RECOVERING après FAILED | nav_recovery | ✅ PASS |
| S18 | Précision livraison > 95% | delivery_accuracy | ✅ PASS |
| S19 | Détection QR code salle | perception | ✅ PASS |
| S20 | Détection humain + zone sécurité | human_detect | ✅ PASS |
| S21 | Dashboard P4 — 20 topics actifs | monitoring | ✅ PASS |
| S22 | KPIs globaux — tous > seuils | KPIs intégrés | ✅ PASS |

---

## 📊 KPIs & Métriques

Résultats obtenus lors de la campagne de tests finale :

| KPI | Seuil requis | Résultat obtenu | Statut |
|-----|-------------|-----------------|--------|
| `success_rate` | > 95% | **100.0%** | ✅ |
| `ERT` (Elapsed Response Time) | < 2 000 ms | **0 ms** (sim) | ✅ |
| `obstacle_avoidance_rate` | > 98% | **100.0%** | ✅ |
| `delivery_accuracy` | > 95% | **98.4%** | ✅ |
| `nav_recovery_rate` | > 90% | **100.0%** | ✅ |
| `scheduler_compatibility` | = 100% | **30/30 (100%)** | ✅ |

---

## 📁 Structure du Projet

```
rachid-PharmaBot/
├── README.md
├── .gitignore
│
├── hospital_robot_spawner/          ← Package ROS2
│   ├── setup.py                     ← Version 3.0.0 (Phase 1+2+3+4)
│   ├── package.xml
│   ├── resource/
│   │
│   ├── launch/
│   │   └── pharmabot_full.launch.py ← Lance TOUT en une commande
│   │
│   ├── worlds/
│   │   └── hospital.world           ← Monde Gazebo hôpital
│   │
│   ├── models/
│   │   ├── pioneer3at/              ← Robot Pioneer 3AT (SDF)
│   │   ├── mobile_warehouse_robot/
│   │   └── Target/                  ← Marqueurs de destination
│   │
│   ├── rl_models/                   ← Agents PPO entraînés (.zip)
│   │
│   └── hospital_robot_spawner/      ← Code source Python
│       ├── rt_scheduler.py          ← EDF Scheduler          (Phase 1)
│       ├── mission_manager.py       ← Machine états + RMS    (Phase 1)
│       ├── navigation_bridge.py     ← Pont EDF ↔ Robot       (Phase 1)
│       ├── delivery_detector.py     ← Détection livraison    (Phase 1)
│       ├── doctor_request_node.py   ← Générateur requêtes    (Phase 1)
│       ├── pharmacist_node.py       ← Validation pharmacien  (Phase 2)
│       ├── dma_tasks.py             ← Pipeline DMA           (Phase 2)
│       ├── watchdog_node.py         ← Watchdog sécurité      (Phase 2)
│       ├── dashboard.py             ← Dashboard 13 topics    (Phase 2)
│       ├── autonomous_navigator.py  ← Navigateur 8 états     (Phase 3)
│       ├── navigation_watchdog_p3.py← Watchdog navigation    (Phase 3)
│       ├── perception_node.py       ← QR/ArUco + humains     (Phase 4)
│       ├── visual_dashboard_p4.py   ← Dashboard 20 topics    (Phase 4)
│       ├── pharmabot_env.py         ← Environnement RL Gym
│       ├── robot_controller.py      ← Contrôle bas niveau
│       ├── spawn_demo.py            ← Spawn Pioneer 3AT
│       ├── start_training.py        ← Entraînement PPO
│       └── trained_agent.py         ← Agent RL backup
│
└── tests/
    ├── run_tests_phase1_phase2.py   ← Tests P1+P2 (10 scénarios)
    └── run_tests_phase3_phase4.py   ← Tests P3+P4 (22 scénarios)
```

---

## 🐳 Lancer avec Docker

```bash
# Builder l'image
docker compose build

# Lancer les tests Phase 1+2
docker compose run pharmabot-tests

# Lancer les tests Phase 3+4
docker compose run pharmabot-tests python3 tests/run_tests_phase3_phase4.py

# Lancer en mode verbose
docker compose run pharmabot-tests python3 tests/run_tests_phase3_phase4.py -v
```

Le `docker-compose.yml` utilise l'image `osrf/ros:humble-desktop` pour garantir la compatibilité.

---

## 🖥️ Lancer avec ROS2 + Gazebo

### Simulation complète (toutes les phases)

```bash
# Sourcer l'environnement
source /opt/ros/humble/setup.bash
source install/setup.bash

# Lancer TOUT en une commande (Gazebo + tous les nœuds)
ros2 launch hospital_robot_spawner pharmabot_full.launch.py
```

### Séquence de démarrage (`pharmabot_full.launch.py`)

```
t+0s   Gazebo (hospital.world)
t+3s   Spawn robot (Pioneer 3AT à [16.0, 0.0])
t+5s   EDF RT Scheduler
t+5s   DMA Scheduler
t+5s   Pharmacist Node          (Phase 2)
t+6s   Mission Manager
t+6s   Watchdog Node
t+7s   Navigation Bridge        (Phase 1)
t+7s   Delivery Detector        (Phase 1)
t+8s   Doctor Request Node      (Phase 1)
t+9s   Autonomous Navigator     (Phase 3)
t+9s   Navigation Watchdog P3   (Phase 3)
t+11s  Perception Node          (Phase 4)
t+13s  Visual Dashboard P4      (Phase 4)
t+15s  Trained Agent (RL backup)
```

### Lancer les nœuds individuellement

```bash
# Phase 1 — Nœuds RT de base
ros2 run hospital_robot_spawner rt_scheduler
ros2 run hospital_robot_spawner mission_manager
ros2 run hospital_robot_spawner nav_bridge
ros2 run hospital_robot_spawner delivery_detect
ros2 run hospital_robot_spawner doctor_request

# Phase 2 — Pharmacie & Monitoring
ros2 run hospital_robot_spawner pharmacist
ros2 run hospital_robot_spawner dma_tasks
ros2 run hospital_robot_spawner watchdog_node
ros2 run hospital_robot_spawner dashboard

# Phase 3 — Navigation Autonome
ros2 run hospital_robot_spawner autonomous_nav
ros2 run hospital_robot_spawner nav_watchdog_p3

# Phase 4 — Perception & Dashboard avancé
ros2 run hospital_robot_spawner perception_node
ros2 run hospital_robot_spawner dashboard_p4

# Agent RL (backup navigation)
ros2 run hospital_robot_spawner trained_agent
```

### Surveiller les topics

```bash
# Voir toutes les missions actives
ros2 topic echo /pharmabot/tache_courante

# Surveiller l'état du robot
ros2 topic echo /pharmabot/etat_robot

# Voir les alertes watchdog
ros2 topic echo /pharmabot/watchdog

# État navigation Phase 3
ros2 topic echo /pharmabot/nav_state_p3

# Détections Phase 4
ros2 topic echo /pharmabot/room_detected
ros2 topic echo /pharmabot/human_detected
```

---

## 🛠️ Stack Technologique

| Catégorie | Technologie | Version |
|-----------|-------------|---------|
| Middleware | ROS2 Humble Hawksbill | LTS 2027 |
| Simulation | Gazebo Classic | 11.x |
| Robot | Pioneer 3AT (SDF) | — |
| IA/RL | Stable-Baselines3 PPO | 2.3.2 |
| Env RL | Gymnasium | 0.29.1 |
| Numérique | NumPy | < 2.0 |
| Optimisation | Optuna | 3.6.1 |
| Viz entraînement | TensorBoard | latest |
| Conteneur | Docker + docker-compose | — |
| OS | Ubuntu 22.04 LTS ARM64 | — |
| Langage | Python | 3.10 |

---

## 📝 Licence

MIT License — voir [LICENSE](LICENSE)

---

## 👥 Équipe

**PharmaBot** — Projet de fin d'études en Systèmes Embarqués Temps Réel

- **Rachid Ait Aissa** — Architecte système + Développement RT
- **Ibn Tofaïl University** — Kénitra, Maroc
- **Encadrant** : Prof. Khaoula Boukir

---

*Année universitaire 2024-2025*

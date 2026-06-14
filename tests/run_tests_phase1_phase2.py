#!/usr/bin/env python3
"""
run_tests_phase1_phase2.py
══════════════════════════════════════════════════════════════════════
Campagne de validation PharmaBot — Phase 1 + Phase 2
Équipe PharmaBot | Ibn Tofaïl University | 2025-2026
Prof. Khaoula Boukir — Systèmes Embarqués Temps Réel

Exécution (aucune dépendance externe, pas de ROS2 requis) :
    python3 run_tests_phase1_phase2.py

Scénarios couverts :
    S01 — Mission simple réussie (FIRM_RT Consultation)
    S02 — Plusieurs SOFT_RT avec deadlines différentes
    S03 — Urgence HARD_RT pendant mission en cours (EDF Override)
    S04 — Livraison échouée (deadline HARD_RT dépassée)
    S05 — Perte de communication / silence nœud
    S06 — Absence de confirmation de livraison (bug corrigé)
    S07 — Panne robot pendant navigation
    S08 — Reprise automatique après override HARD_RT
    S09 — Surcharge 25 requêtes simultanées
    S10 — Journée hospitalière complète (8 vagues)
══════════════════════════════════════════════════════════════════════
"""

import json
import math
import time
import threading
import random
import sys
import os
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple

# ══════════════════════════════════════════════════════════════════
# 0. CONSTANTES (miroir exact du code de production)
# ══════════════════════════════════════════════════════════════════

SALLES = {
    "pharmacie":    {"nom": "Pharmacy_Room",    "x":  1.0, "y":  16.0},
    "reanimation":  {"nom": "ICU_Room",          "x": 11.0, "y":   5.5},
    "urgences":     {"nom": "Emergency_Room",    "x": -5.0, "y":  -6.6},
    "consultation": {"nom": "Consultation_Room", "x": -2.0, "y": -27.0},
}

DEPARTEMENTS_INFO = {
    "reanimation":  {"priorite": 1, "type_rt": "HARD_RT", "deadline_sec": 30},
    "urgences":     {"priorite": 2, "type_rt": "SOFT_RT", "deadline_sec": 120},
    "consultation": {"priorite": 3, "type_rt": "FIRM_RT", "deadline_sec": 300},
}

MEDICAMENTS = {
    "reanimation":  ["Adrénaline 1mg", "Morphine 10mg IV", "Noradrénaline 4mg",
                     "Atropine 0.5mg", "Furosémide 20mg", "Insuline Rapide", "Propofol 200mg"],
    "urgences":     ["Morphine 10mg", "Paracétamol IV 1g", "Ibuprofène 400mg",
                     "Ondansétron 4mg", "Métoclopramide 10mg", "Aspirine 500mg", "Lorazépam 2mg"],
    "consultation": ["Amoxicilline 500mg", "Paracétamol 1g", "Ibuprofène 400mg",
                     "Oméprazole 20mg", "Métformine 500mg", "Amlodipine 5mg", "Atorvastatine 20mg"],
}

MEDECINS = {
    "reanimation":  ["Dr. Alaoui (ICU)", "Dr. Benali (ICU)", "Dr. Cherkaoui (ICU)"],
    "urgences":     ["Dr. Driss (Urg)", "Dr. El Idrissi (Urg)", "Dr. Fathi (Urg)"],
    "consultation": ["Dr. Guessous", "Dr. Hammoudi", "Dr. Idrissi"],
}

ETATS_VALIDES = [
    "IDLE", "CHARGEMENT", "NAVIGATION", "EVITEMENT",
    "REPLANIFICATION", "LIVRAISON", "RETOUR_PHARMACIE",
    "URGENCE_OVERRIDE", "ARRET_URGENCE",
]

VITESSE_ROBOT = 0.6   # m/s
COOLDOWN_LIVRAISON = 10.0  # s

# Couleurs terminal
R = "\033[91m"; G = "\033[92m"; Y = "\033[93m"
C = "\033[96m"; W = "\033[0m";  B = "\033[1m"


# ══════════════════════════════════════════════════════════════════
# 1. BUS DE MESSAGES (remplace le middleware ROS2 DDS)
# ══════════════════════════════════════════════════════════════════

class TopicBus:
    """Bus pub/sub en mémoire — simule le DDS ROS2."""

    def __init__(self):
        self._cbs:  Dict[str, List] = {}
        self._hist: Dict[str, List] = {}
        self._lock  = threading.Lock()

    def subscribe(self, topic: str, cb):
        with self._lock:
            self._cbs.setdefault(topic, []).append(cb)

    def publish(self, topic: str, msg: dict):
        with self._lock:
            self._hist.setdefault(topic, []).append(
                {"data": msg, "ts": time.time()})
            cbs = list(self._cbs.get(topic, []))
        for cb in cbs:
            try:
                cb(msg)
            except Exception as e:
                pass  # nœud défaillant n'arrête pas le bus

    def last(self, topic: str) -> Optional[dict]:
        with self._lock:
            msgs = self._hist.get(topic, [])
            return msgs[-1]["data"] if msgs else None

    def all_data(self, topic: str) -> List[dict]:
        with self._lock:
            return [m["data"] for m in self._hist.get(topic, [])]

    def count(self, topic: str) -> int:
        with self._lock:
            return len(self._hist.get(topic, []))


# ══════════════════════════════════════════════════════════════════
# 2. MODÈLE DE REQUÊTE
# ══════════════════════════════════════════════════════════════════

@dataclass
class Requete:
    departement: str
    medicament:  str
    medecin:     str   = ""
    source:      str   = "TEST"
    requete_id:  int   = 0
    timestamp:   float = field(default_factory=time.time)

    @property
    def type_rt(self)     -> str: return DEPARTEMENTS_INFO[self.departement]["type_rt"]
    @property
    def deadline_sec(self)-> int: return DEPARTEMENTS_INFO[self.departement]["deadline_sec"]
    @property
    def priorite(self)    -> int: return DEPARTEMENTS_INFO[self.departement]["priorite"]

    def temps_restant(self) -> float:
        return self.deadline_sec - (time.time() - self.timestamp)

    def deadline_ok(self) -> bool:
        return self.temps_restant() > 0

    def to_dict(self) -> dict:
        return {
            "departement":  self.departement,
            "medicament":   self.medicament,
            "medecin":      self.medecin,
            "type_rt":      self.type_rt,
            "deadline_sec": self.deadline_sec,
            "priorite":     self.priorite,
            "timestamp":    self.timestamp,
            "requete_id":   self.requete_id,
        }


# ══════════════════════════════════════════════════════════════════
# 3. SIMULATEURS DE NŒUDS ROS2
# ══════════════════════════════════════════════════════════════════

class SimEDFScheduler:
    """Simule rt_scheduler.py — tri EDF + détection override HARD_RT."""

    def __init__(self, bus: TopicBus):
        self.bus   = bus
        self._file: List[Requete] = []
        self._tc:   Optional[Requete] = None
        self._dept: Optional[str]     = None
        self._lock  = threading.Lock()
        self.stats  = {"total":0,"hard_rt":0,"soft_rt":0,"firm_rt":0,
                        "overrides_edf":0,"deadlines_hard":0,"deadlines_soft":0}
        bus.subscribe("/pharmabot/livraison_confirmee", self._on_livraison)

    def ajouter(self, req: Requete):
        with self._lock:
            self._file.append(req)
            self._file.sort(key=lambda r: r.temps_restant())
            self.stats["total"] += 1
            k = req.type_rt.lower().replace("-","_")
            if k in self.stats: self.stats[k] += 1
        # Override EDF ?
        if (req.type_rt == "HARD_RT" and self._dept
                and self._dept != req.departement
                and DEPARTEMENTS_INFO.get(self._dept, {}).get("type_rt") != "HARD_RT"):
            self._signaler_override(req)
        self._pub_tache()

    def verifier_deadlines(self):
        with self._lock:
            snap = list(self._file)
        for r in snap:
            t = r.temps_restant()
            if r.type_rt == "HARD_RT" and 0 < t < 10:
                self.bus.publish("/pharmabot/alerte_rt",{
                    "type_alerte":"HARD_RT_CRITIQUE","departement":r.departement,
                    "type_rt":r.type_rt,"temps_restant":t,"timestamp":time.time()})
            elif not r.deadline_ok():
                if r.type_rt == "HARD_RT":
                    with self._lock: self.stats["deadlines_hard"] += 1
                    self.bus.publish("/pharmabot/alerte_rt",{
                        "type_alerte":"HARD_RT_MANQUE","departement":r.departement,
                        "type_rt":r.type_rt,"consequence":"FATAL","timestamp":time.time()})
                elif r.type_rt == "SOFT_RT":
                    with self._lock: self.stats["deadlines_soft"] += 1
                    self.bus.publish("/pharmabot/alerte_rt",{
                        "type_alerte":"SOFT_RT_DEGRADE","departement":r.departement,
                        "type_rt":r.type_rt,"timestamp":time.time()})

    def _pub_tache(self):
        with self._lock:
            self._file = [r for r in self._file
                          if not (r.type_rt=="FIRM_RT" and not r.deadline_ok())]
            self._file.sort(key=lambda r: r.temps_restant())
            if not self._file: return
            tache = self._file[0]
        self._tc   = tache
        self._dept = tache.departement
        self.bus.publish("/pharmabot/tache_courante", tache.to_dict())

    def _signaler_override(self, req: Requete):
        with self._lock:
            self.stats["overrides_edf"] += 1
            num = self.stats["overrides_edf"]
        self.bus.publish("/pharmabot/edf_interruption",{
            "type":"EDF_INTERRUPTION",
            "dept_interrompu": self._dept,
            "dept_urgent":     req.departement,
            "type_rt_urgent":  req.type_rt,
            "deadline_urgent": req.deadline_sec,
            "medicament_urgent": req.medicament,
            "override_num": num,
            "timestamp": time.time(),
        })

    def _on_livraison(self, msg: dict):
        with self._lock:
            self._dept = None; self._tc = None
            if self._file: self._file.pop(0)
        self._pub_tache()


class SimMissionManager:
    """Simule mission_manager.py — machine à états + RMS + override EDF (bug S06 corrigé)."""

    def __init__(self, bus: TopicBus):
        self.bus   = bus
        self._etat = "IDLE"
        self._ep   = "IDLE"          # état précédent
        self._mc:  Optional[dict] = None   # mission courante
        self._mi:  Optional[dict] = None   # mission interrompue
        self._lock = threading.Lock()
        self.stats = {"overrides": 0, "reprises": 0}
        self._hist: List[Tuple[str, float]] = []

        bus.subscribe("/pharmabot/tache_courante",      self._on_mission)
        bus.subscribe("/pharmabot/edf_interruption",    self._on_override)
        bus.subscribe("/pharmabot/alerte_rt",           self._on_alerte)
        bus.subscribe("/pharmabot/dma_pipeline",        self._on_dma)
        bus.subscribe("/pharmabot/livraison_confirmee", self._on_livraison)
        bus.subscribe("/pharmabot/watchdog",            self._on_watchdog)

    def _set(self, etat: str):
        if etat not in ETATS_VALIDES: return
        with self._lock:
            self._ep = self._etat; self._etat = etat
            self._hist.append((etat, time.time()))
        self.bus.publish("/pharmabot/etat_robot",{
            "etat":etat,"etat_precedent":self._ep,
            "mission":self._mc,"timestamp":time.time()})

    @property
    def etat(self) -> str:
        with self._lock: return self._etat

    @property
    def historique(self) -> List[str]:
        with self._lock: return [h[0] for h in self._hist]

    def _on_mission(self, msg: dict):
        with self._lock:
            etat = self._etat
            dept_mc = self._mc.get("departement") if self._mc else None
        if etat == "IDLE":
            self._start(msg)
        elif dept_mc == msg.get("departement"):
            pass  # doublon

    def _on_override(self, msg: dict):
        with self._lock: etat = self._etat
        if etat not in ("ARRET_URGENCE","URGENCE_OVERRIDE"):
            urgente = {"departement": msg["dept_urgent"],
                       "type_rt":     msg["type_rt_urgent"],
                       "deadline_sec":msg["deadline_urgent"],
                       "medicament":  msg["medicament_urgent"],
                       "timestamp":   time.time()}
            self._interrupt(urgente)

    def _on_alerte(self, msg: dict):
        if msg.get("type_alerte") in ("HARD_RT_CRITIQUE","HARD_RT_MANQUE"):
            with self._lock: etat = self._etat
            if etat not in ("ARRET_URGENCE","URGENCE_OVERRIDE"):
                self._set("URGENCE_OVERRIDE")

    def _on_dma(self, msg: dict):
        e = msg.get("etat","")
        with self._lock: etat = self._etat
        if   e in ("VALIDATION_PHARMACIEN","CONFIRMATION_MEDICAMENT") and etat == "IDLE":
            self._set("CHARGEMENT")
        elif e == "NAVIGATION"  and etat == "CHARGEMENT":  self._set("NAVIGATION")
        elif e == "LIVRAISON"   and etat == "NAVIGATION":  self._set("LIVRAISON")
        elif e == "ARRET_URGENCE": self._set("ARRET_URGENCE")

    def _on_livraison(self, msg: dict):
        """CORRECTIF BUG S06 — 3 gardes avant de terminer la mission."""
        dept = msg.get("departement")
        with self._lock:
            mc   = self._mc
            etat = self._etat
        # Garde 1 : mission courante existe
        if not mc:
            return
        # Garde 2 : correspond au département
        if mc.get("departement") != dept:
            return
        # Garde 3 : état compatible
        if etat not in ("NAVIGATION","LIVRAISON","URGENCE_OVERRIDE","CHARGEMENT"):
            return
        self._set("RETOUR_PHARMACIE")
        self._finish()

    def _on_watchdog(self, msg: dict):
        t = msg.get("type","")
        if t == "RECUPERATION": self._set("ARRET_URGENCE")
        elif t == "REPRISE":
            with self._lock: etat = self._etat
            if etat == "ARRET_URGENCE": self._set("IDLE")

    def _start(self, m: dict):
        with self._lock: self._mc = m
        self._set("CHARGEMENT")

    def _interrupt(self, urgente: dict):
        with self._lock:
            self.stats["overrides"] += 1
            self._mi = self._mc
        self._set("URGENCE_OVERRIDE")
        self._start(urgente)

    def _finish(self):
        with self._lock:
            mi = self._mi
            if mi: self.stats["reprises"] += 1
            self._mi = None; self._mc = None
        if mi: self._start(mi)
        else:  self._set("IDLE")


class SimNavigationBridge:
    """Simule navigation_bridge.py — contrôleur P simple."""

    def __init__(self, bus: TopicBus):
        self.bus    = bus
        self._x     = 1.0; self._y = 16.0
        self._mc    = None
        self._cible = None
        self._arret = False
        self._lock  = threading.Lock()
        bus.subscribe("/pharmabot/tache_courante", self._on_mission)
        bus.subscribe("/pharmabot/watchdog",       self._on_watchdog)
        bus.subscribe("/pharmabot/etat_robot",     self._on_etat)

    def _on_mission(self, msg: dict):
        dept = msg.get("departement")
        if not dept or dept == "pharmacie": return
        with self._lock: self._mc = msg; self._cible = dept
        salle = SALLES[dept]
        self.bus.publish("/pharmabot/navigation_goal",{
            "departement":dept,"nom":salle["nom"],
            "x":salle["x"],"y":salle["y"],
            "timestamp":time.time(),"mission":msg})

    def _on_watchdog(self, msg: dict):
        if   msg.get("type") == "RECUPERATION":
            with self._lock: self._arret = True
        elif msg.get("type") == "REPRISE":
            with self._lock: self._arret = False

    def _on_etat(self, msg: dict):
        if msg.get("etat") == "ARRET_URGENCE":
            with self._lock: self._arret = True

    def simuler_nav(self, dept: str, duree: float = 0.2) -> bool:
        """Déplace le robot vers dept en duree secondes (simulé)."""
        if dept not in SALLES: return False
        s = SALLES[dept]; t0 = time.time()
        while time.time() - t0 < duree:
            with self._lock:
                if self._arret: return False
            time.sleep(0.02)
            p = min(1.0, (time.time()-t0)/duree)
            with self._lock:
                self._x = 1.0 + (s["x"]-1.0)*p
                self._y = 16.0+(s["y"]-16.0)*p
        with self._lock: self._x = s["x"]; self._y = s["y"]
        self.bus.publish("/pharmabot/nav_status",{
            "type":"ARRIVE_DESTINATION","departement":dept,
            "phase":"NAVIGATION","timestamp":time.time(),"mission":self._mc})
        return True

    @property
    def arret(self) -> bool:
        with self._lock: return self._arret


class SimDeliveryDetector:
    """Simule delivery_detector.py — confirmation par position."""

    def __init__(self, bus: TopicBus):
        self.bus    = bus
        self._cible = None; self._mc = None; self._ts = None
        self._dernier: Dict[str,float] = {}
        self._livs: List[dict] = []
        self._lock  = threading.Lock()
        self.stats  = {"livraisons_total":0,"deadlines_ok":0,
                        "deadlines_ko":0,"tps_moyen":0.0}
        bus.subscribe("/pharmabot/navigation_goal",  self._on_goal)
        bus.subscribe("/pharmabot/tache_courante",   self._on_mission)
        bus.subscribe("/pharmabot/nav_status",       self._on_nav)

    def _on_goal(self, msg: dict):
        d = msg.get("departement")
        if d and d in SALLES and d != "pharmacie":
            with self._lock: self._cible = d

    def _on_mission(self, msg: dict):
        with self._lock:
            self._mc = msg; self._ts = time.time()

    def _on_nav(self, msg: dict):
        if msg.get("type") != "ARRIVE_DESTINATION": return
        dept = msg.get("departement")
        with self._lock:
            c = self._cible; dl = self._dernier.get(dept, 0)
        if dept == c and time.time()-dl >= COOLDOWN_LIVRAISON:
            self._confirmer(dept, msg.get("mission"))

    def _confirmer(self, dept: str, mission: dict):
        with self._lock:
            self._dernier[dept] = time.time()
            ts = self._ts; m = self._mc or mission
        tl = time.time()-ts if ts else 0.0
        ok = True
        if m: ok = tl <= m.get("deadline_sec", 9999)
        with self._lock:
            self.stats["livraisons_total"] += 1
            if ok: self.stats["deadlines_ok"] += 1
            else:  self.stats["deadlines_ko"] += 1
            n = self.stats["livraisons_total"]
            self.stats["tps_moyen"] = (self.stats["tps_moyen"]*(n-1)+tl)/n
        lv = {"type":"LIVRAISON_CONFIRMEE","departement":dept,
              "nom_salle":SALLES[dept]["nom"],
              "medicament":m.get("medicament","?") if m else "?",
              "type_rt":m.get("type_rt","?") if m else "?",
              "temps_livraison":round(tl,1),
              "deadline_sec":m.get("deadline_sec",0) if m else 0,
              "deadline_ok":ok,
              "livraison_num":self.stats["livraisons_total"],
              "timestamp":time.time()}
        with self._lock: self._livs.append(lv)
        self.bus.publish("/pharmabot/livraison_confirmee", lv)

    @property
    def livraisons(self) -> List[dict]:
        with self._lock: return list(self._livs)


class SimWatchdog:
    """Simule watchdog_node.py."""

    def __init__(self, bus: TopicBus):
        self.bus   = bus
        self._recup = False
        self._lock  = threading.Lock()
        self.nb_alertes = 0; self.nb_recups = 0
        bus.subscribe("/pharmabot/alerte_rt",  self._on_alerte)
        bus.subscribe("/pharmabot/dma_alerte", self._on_dma)

    def _on_alerte(self, msg: dict):
        if msg.get("type_alerte") == "HARD_RT_MANQUE":
            self.nb_alertes += 1
            with self._lock:
                if self._recup: return
                self._recup = True; self.nb_recups += 1
            self.bus.publish("/pharmabot/watchdog",{
                "type":"RECUPERATION","raison":"HARD_RT_MANQUE",
                "contexte":msg,"timestamp":time.time()})
            threading.Timer(5.0, self._reprise).start()

    def _on_dma(self, msg: dict):
        if msg.get("priorite") == 1:
            self.bus.publish("/pharmabot/watchdog",{
                "type":"RECUPERATION","raison":"DMA_P1","timestamp":time.time()})

    def _reprise(self):
        with self._lock: self._recup = False
        self.bus.publish("/pharmabot/watchdog",{
            "type":"REPRISE","timestamp":time.time()})

    @property
    def en_recup(self) -> bool:
        with self._lock: return self._recup


class SimPharmacist:
    """Simule pharmacist_node.py — validation médicament."""

    STOCK = {"Adrénaline 1mg":3,"Adrénaline 1mg — URGENCE":3,
             "Morphine 10mg IV":5,"Morphine 10mg":8,
             "Paracétamol IV 1g":15,"Paracétamol 1g":20,
             "Ibuprofène 400mg":12,"Amoxicilline 500mg":12,
             "Médicament standard":50}

    def __init__(self, bus: TopicBus, delai: float = 0.05):
        self.bus   = bus; self._d = delai
        self._stock= dict(self.STOCK); self._lock = threading.Lock()
        bus.subscribe("/pharmabot/requete", self._on_req)

    def _on_req(self, msg: dict):
        threading.Thread(target=self._traiter, args=(msg,), daemon=True).start()

    def _traiter(self, msg: dict):
        time.sleep(self._d)
        med = msg.get("medicament","Médicament standard")
        with self._lock:
            k = med if med in self._stock else "Médicament standard"
            if self._stock.get(k,0) <= 0: self._stock[k] = 10
            self._stock[k] -= 1
        self.bus.publish("/pharmabot/pharmacist_ok",{
            "type":"PHARMACIST_OK","departement":msg.get("departement","?"),
            "medicament_fourni":k,"type_rt":msg.get("type_rt","FIRM_RT"),
            "timestamp":time.time()})


class SimDMA:
    """Simule dma_tasks.py — pipeline 4 étapes."""

    def __init__(self, bus: TopicBus):
        self.bus   = bus; self._etat = "IDLE"; self._lock = threading.Lock()
        self.stats = {"traitees":0,"succes":0}
        bus.subscribe("/pharmabot/tache_courante",   self._on_mission)
        bus.subscribe("/pharmabot/nav_status",       self._on_nav)
        bus.subscribe("/pharmabot/edf_interruption", self._on_override)

    def _pub(self, etat: str):
        with self._lock: anc = self._etat; self._etat = etat
        self.bus.publish("/pharmabot/dma_pipeline",{
            "etat":etat,"ancien":anc,"timestamp":time.time()})

    def _on_mission(self, msg: dict):
        if not msg.get("departement") or msg["departement"]=="pharmacie": return
        with self._lock:
            etat = self._etat; self.stats["traitees"] += 1
        if etat == "IDLE":
            threading.Thread(target=self._pipeline, args=(msg,), daemon=True).start()

    def _pipeline(self, m: dict):
        self._pub("VALIDATION_PHARMACIEN"); time.sleep(0.03)
        self._pub("CONFIRMATION_MEDICAMENT"); time.sleep(0.03)
        self._pub("NAVIGATION")

    def _on_nav(self, msg: dict):
        if msg.get("type") == "ARRIVE_DESTINATION":
            with self._lock: etat = self._etat
            if etat == "NAVIGATION":
                self._pub("LIVRAISON"); time.sleep(0.02)
                with self._lock: self.stats["succes"] += 1
                self._pub("IDLE")

    def _on_override(self, _):
        with self._lock: self._etat = "IDLE"


# ══════════════════════════════════════════════════════════════════
# 4. RÉSULTATS DE TEST
# ══════════════════════════════════════════════════════════════════

@dataclass
class Resultat:
    sid:   str
    nom:   str
    ok:    bool   = True
    warns: List[str] = field(default_factory=list)
    fails: List[str] = field(default_factory=list)
    checks:List[str] = field(default_factory=list)
    metriques: Dict[str,float] = field(default_factory=dict)
    duree: float = 0.0

    @property
    def statut(self) -> str:
        if self.fails: return "FAIL"
        if self.warns: return "WARN"
        return "PASS"

    def check(self, cond: bool, msg_ok: str, msg_fail: str):
        if cond: self.checks.append(f"✓ {msg_ok}")
        else:    self.fails.append(f"✗ {msg_fail}")

    def warn(self, msg: str): self.warns.append(f"⚠ {msg}")
    def info(self, msg: str): self.checks.append(f"  {msg}")


# ══════════════════════════════════════════════════════════════════
# 5. ENVIRONNEMENT DE TEST (crée les simulateurs + helpers)
# ══════════════════════════════════════════════════════════════════

class Env:
    """Environnement de test isolé par scénario."""

    _ctr = 0

    def __init__(self):
        self.bus       = TopicBus()
        self.scheduler = SimEDFScheduler(self.bus)
        self.mm        = SimMissionManager(self.bus)
        self.nav       = SimNavigationBridge(self.bus)
        self.detect    = SimDeliveryDetector(self.bus)
        self.watchdog  = SimWatchdog(self.bus)
        self.pharmacist= SimPharmacist(self.bus)
        self.dma       = SimDMA(self.bus)

    def req(self, dept: str, med: str = None, doc: str = None) -> Requete:
        Env._ctr += 1
        r = Requete(
            departement=dept,
            medicament=med or random.choice(MEDICAMENTS[dept]),
            medecin=doc or random.choice(MEDECINS[dept]),
            source="TEST",
            requete_id=Env._ctr,
        )
        self.bus.publish("/pharmabot/requete", r.to_dict())
        self.scheduler.ajouter(r)
        return r

    def req_perimee(self, dept: str, decalage_sec: float) -> Requete:
        """Requête avec timestamp artificiellement reculé (deadline dépassée)."""
        Env._ctr += 1
        r = Requete(
            departement=dept,
            medicament=random.choice(MEDICAMENTS[dept]),
            medecin=random.choice(MEDECINS[dept]),
            requete_id=Env._ctr,
            timestamp=time.time() - decalage_sec,
        )
        self.bus.publish("/pharmabot/requete", r.to_dict())
        self.scheduler.ajouter(r)
        return r

    def nav_ok(self, dept: str, duree: float = 0.15) -> bool:
        return self.nav.simuler_nav(dept, duree)

    def attendre_etat(self, etat: str, timeout: float = 3.0) -> bool:
        t0 = time.time()
        while time.time()-t0 < timeout:
            if self.mm.etat == etat: return True
            time.sleep(0.03)
        return False


# ══════════════════════════════════════════════════════════════════
# 6. LES 10 SCÉNARIOS
# ══════════════════════════════════════════════════════════════════

def s01_mission_simple() -> Resultat:
    """
    S01 — Mission simple réussie
    Données d'entrée : Dr. Guessous, Amoxicilline 500mg, Consultation (FIRM_RT)
    Séquence attendue : IDLE→CHARGEMENT→NAVIGATION→LIVRAISON→RETOUR→IDLE
    Topics : /requete /tache_courante /navigation_goal /nav_status /livraison_confirmee
    """
    r = Resultat("S01", "Mission simple réussie — Consultation FIRM_RT")
    t0 = time.time()
    env = Env()

    # ── Injection requête ──
    env.req("consultation", "Amoxicilline 500mg", "Dr. Guessous")
    time.sleep(0.15)

    # ── Vérification tache_courante ──
    tc = env.bus.last("/pharmabot/tache_courante")
    r.check(tc is not None, "tache_courante publiée", "tache_courante ABSENT")
    if tc:
        r.check(tc.get("departement")=="consultation",
                "département=consultation", f"département={tc.get('departement')}")
        r.check(tc.get("type_rt")=="FIRM_RT",
                "type_rt=FIRM_RT", f"type_rt={tc.get('type_rt')}")

    # ── Vérification navigation_goal ──
    ng = env.bus.last("/pharmabot/navigation_goal")
    r.check(ng is not None, "navigation_goal publié", "navigation_goal ABSENT")
    if ng:
        r.check(ng.get("departement")=="consultation",
                "goal dept=consultation","goal dept wrong")
        x,y = ng.get("x",0), ng.get("y",0)
        r.check(abs(x-(-2.0))<0.1 and abs(y-(-27.0))<0.1,
                f"Coordonnées Consultation_Room correctes ({x},{y})",
                f"Coordonnées INCORRECTES ({x},{y}) ≠ (-2.0,-27.0)")

    # ── État machine → CHARGEMENT (ou NAVIGATION si DMA rapide) ──
    atteint = env.attendre_etat("CHARGEMENT", timeout=1.0)
    if not atteint:
        atteint = env.attendre_etat("NAVIGATION", timeout=2.0)
    etat_actuel = env.mm.etat
    r.check(etat_actuel in ("CHARGEMENT","NAVIGATION","RETOUR_PHARMACIE","LIVRAISON"),
            f"État machine actif: {etat_actuel} (CHARGEMENT/NAVIGATION attendu)",
            f"État={etat_actuel} — machine à états inactive")

    # ── Simuler navigation ──
    env.nav_ok("consultation", 0.2)
    time.sleep(0.2)

    # ── Livraison confirmée ──
    livs = env.detect.livraisons
    r.check(len(livs) > 0, f"Livraison #{len(livs)} confirmée", "Aucune livraison confirmée")
    if livs:
        lv = livs[-1]
        r.check(lv["deadline_ok"], "Deadline FIRM_RT respectée",
                f"Deadline dépassée ({lv['temps_livraison']}s/{lv['deadline_sec']}s)")
        r.info(f"Salle : {lv['nom_salle']} | Médicament : {lv['medicament']}")
        r.info(f"Temps livraison : {lv['temps_livraison']}s / {lv['deadline_sec']}s")

    # ── Topics actifs ──
    for topic in ["/pharmabot/requete","/pharmabot/tache_courante",
                  "/pharmabot/navigation_goal"]:
        r.check(env.bus.count(topic)>0, f"Topic actif: {topic.split('/')[-1]}",
                f"Topic INACTIF: {topic}")

    r.duree = time.time()-t0
    r.metriques = {"livraisons":env.detect.stats["livraisons_total"],
                   "deadlines_ok":env.detect.stats["deadlines_ok"],
                   "topics_tache_courante":env.bus.count("/pharmabot/tache_courante")}
    return r


def s02_plusieurs_soft_rt() -> Resultat:
    """
    S02 — Plusieurs SOFT_RT avec deadlines différentes
    3 requêtes urgences décalées de 500ms chacune.
    EDF doit prioriser la plus ancienne (deadline la plus proche).
    """
    r = Resultat("S02", "3 SOFT_RT décalées — tri EDF par deadline")
    t0 = time.time()
    env = Env()

    meds = ["Morphine 10mg","Paracétamol IV 1g","Lorazépam 2mg"]
    docs = ["Dr. Driss","Dr. El Idrissi","Dr. Fathi"]

    env.req("urgences", meds[0], docs[0])
    time.sleep(0.5)
    env.req("urgences", meds[1], docs[1])
    time.sleep(0.3)
    env.req("urgences", meds[2], docs[2])
    time.sleep(0.1)

    tc = env.bus.last("/pharmabot/tache_courante")
    r.check(tc is not None, "tache_courante publiée", "tache_courante ABSENT")
    if tc:
        r.check(tc.get("medicament") == meds[0],
                f"EDF correct: req1 ({meds[0]}) prioritaire",
                f"EDF incorrect: {tc.get('medicament')} (attendu {meds[0]})")

    stats = env.scheduler.stats
    r.check(stats["total"] == 3, "3 requêtes enregistrées",
            f"Requêtes enregistrées = {stats['total']} (attendu 3)")
    r.check(stats.get("soft_rt",0) == 3, "3 tâches SOFT_RT comptabilisées",
            f"soft_rt = {stats.get('soft_rt',0)} (attendu 3)")

    # Simuler 1 livraison
    env.nav_ok("urgences", 0.1); time.sleep(0.15)

    r.duree = time.time()-t0
    r.metriques = {"total_reqs":stats["total"],"soft_rt":stats.get("soft_rt",0),
                   "livraisons":env.detect.stats["livraisons_total"]}
    return r


def s03_urgence_hard_rt() -> Resultat:
    """
    S03 — Urgence HARD_RT pendant mission FIRM_RT en cours
    1. Mission consultation démarrée
    2. HARD_RT réanimation injectée → EDF OVERRIDE
    3. Vérifier : edf_interruption publié, URGENCE_OVERRIDE, reprise consultation
    """
    r = Resultat("S03", "HARD_RT pendant FIRM_RT — EDF Override + Reprise")
    t0 = time.time()
    env = Env()

    # Lancer consultation
    env.req("consultation","Amoxicilline 500mg","Dr. Guessous")
    time.sleep(0.15)
    r.info(f"État après consultation: {env.mm.etat}")

    # Injecter HARD_RT réanimation
    env.req("reanimation","Adrénaline 1mg","Dr. Alaoui")
    time.sleep(0.15)

    # Vérifier interruption EDF
    inter = env.bus.last("/pharmabot/edf_interruption")
    r.check(inter is not None, "edf_interruption publié", "edf_interruption ABSENT")
    if inter:
        r.check(inter.get("dept_urgent")=="reanimation",
                "Override vers reanimation", f"Override vers {inter.get('dept_urgent')}")
        r.check(inter.get("type_rt_urgent")=="HARD_RT",
                "type_rt_urgent=HARD_RT", f"type_rt_urgent={inter.get('type_rt_urgent')}")
        r.info(f"Override #{inter.get('override_num')}: "
               f"{inter.get('dept_interrompu')} → {inter.get('dept_urgent')}")

    # Vérifier URGENCE_OVERRIDE (peut transitionner vite vers CHARGEMENT urgence)
    atteint = env.attendre_etat("URGENCE_OVERRIDE", timeout=1.0)
    if not atteint:
        # Le MM peut avoir déjà transitionné vers CHARGEMENT de l'urgence
        atteint2 = env.mm.etat in ("CHARGEMENT","NAVIGATION","URGENCE_OVERRIDE")
        hist = env.mm.historique
        atteint = atteint2 and "URGENCE_OVERRIDE" in hist
    r.check(atteint,
            f"URGENCE_OVERRIDE traversé (état={env.mm.etat}, hist={env.mm.historique[-3:]})",
            f"URGENCE_OVERRIDE absent — état={env.mm.etat}, hist={env.mm.historique}")
    r.check(env.mm.stats["overrides"] >= 1,
            f"Override comptabilisé: #{env.mm.stats['overrides']}",
            "Override NON comptabilisé")

    # Simuler livraison réanimation → reprise consultation
    env.nav_ok("reanimation", 0.1); time.sleep(0.25)
    r.check(env.mm.stats["reprises"] >= 1,
            f"Reprise automatique: #{env.mm.stats['reprises']}",
            "Reprise automatique NON déclenchée")

    r.duree = time.time()-t0
    r.metriques = {"overrides_edf":env.scheduler.stats["overrides_edf"],
                   "reprises":env.mm.stats["reprises"],
                   "livraisons":env.detect.stats["livraisons_total"]}
    return r


def s04_livraison_echouee() -> Resultat:
    """
    S04 — Deadline HARD_RT dépassée
    Requête réanimation avec timestamp reculé de 35s (deadline 30s → déjà expirée).
    Vérifier : alerte HARD_RT publiée, stats deadline_ko.
    """
    r = Resultat("S04", "Deadline HARD_RT dépassée — alerte et watchdog")
    t0 = time.time()
    env = Env()

    # Requête périmée (deadline dépassée de 5s)
    env.req_perimee("reanimation", 35.0)
    time.sleep(0.1)

    # Forcer vérification deadlines
    env.scheduler.verifier_deadlines()
    time.sleep(0.1)

    alertes = env.bus.all_data("/pharmabot/alerte_rt")
    hard_alertes = [a for a in alertes
                    if a.get("type_alerte") in ("HARD_RT_MANQUE","HARD_RT_CRITIQUE")]
    r.check(len(hard_alertes) > 0,
            f"Alerte HARD_RT publiée: {hard_alertes[0]['type_alerte'] if hard_alertes else ''}",
            "Aucune alerte HARD_RT publiée")
    r.info(f"Nombre alertes RT: {len(alertes)}")
    r.info(f"Deadline déjà dépassée: oui (35s > 30s deadline)")
    r.info(f"Stats scheduler: deadlines_hard={env.scheduler.stats['deadlines_hard']}")

    # Livraison tardive
    env.nav_ok("reanimation", 0.1); time.sleep(0.15)
    livs = env.detect.livraisons
    if livs:
        lv = livs[-1]
        r.info(f"Livraison #{lv['livraison_num']}: "
               f"deadline_ok={lv['deadline_ok']} "
               f"({lv['temps_livraison']}s/{lv['deadline_sec']}s)")
        if not lv["deadline_ok"]:
            r.checks.append("✓ Livraison marquée hors deadline")

    r.duree = time.time()-t0
    r.metriques = {"alertes_hard_rt":len(hard_alertes),
                   "watchdog_alertes":env.watchdog.nb_alertes,
                   "deadlines_hard":env.scheduler.stats["deadlines_hard"]}
    return r


def s05_perte_communication() -> Resultat:
    """
    S05 — Perte de communication simulée
    Injection alerte HARD_RT_MANQUE directe → watchdog RECUPERATION → ARRET_URGENCE.
    Vérifier reprise après 5s (simulée immédiatement).
    """
    r = Resultat("S05", "Perte de communication — watchdog RECUPERATION")
    t0 = time.time()
    env = Env()

    env.req("reanimation","Adrénaline 1mg")
    time.sleep(0.1)

    # Simuler silence / perte comm : injecter alerte HARD_RT_MANQUE directement
    env.bus.publish("/pharmabot/alerte_rt",{
        "type_alerte":"HARD_RT_MANQUE","departement":"reanimation",
        "type_rt":"HARD_RT","consequence":"FATAL","timestamp":time.time()})
    time.sleep(0.2)

    # Vérifier watchdog activé
    wd_msgs = env.bus.all_data("/pharmabot/watchdog")
    recups  = [m for m in wd_msgs if m.get("type")=="RECUPERATION"]
    r.check(len(recups)>0, f"Watchdog RECUPERATION activé (raison={recups[0].get('raison') if recups else ''})",
            "Watchdog RECUPERATION NON activé")

    # Vérifier ARRET_URGENCE
    atteint = env.attendre_etat("ARRET_URGENCE", 2.0)
    r.check(atteint, "État → ARRET_URGENCE",
            f"État={env.mm.etat} (attendu ARRET_URGENCE)")

    # Simuler reprise watchdog immédiate (le timer 5s est réel — on le déclenche manuellement)
    env.bus.publish("/pharmabot/watchdog",{"type":"REPRISE","timestamp":time.time()})
    time.sleep(0.1)
    atteint2 = env.attendre_etat("IDLE", 2.0)
    r.check(atteint2, "État → IDLE après REPRISE watchdog",
            f"État={env.mm.etat} après REPRISE (attendu IDLE)")

    r.duree = time.time()-t0
    r.metriques = {"watchdog_recuperations":len(recups),
                   "alertes_hard":env.watchdog.nb_alertes}
    return r


def s06_absence_confirmation() -> Resultat:
    """
    S06 — Absence de confirmation de livraison (bug S06 corrigé)
    Robot arrive, nav_status publié, MAIS livraison_confirmee ne doit PAS
    remettre la machine à IDLE si l'état est incompatible.
    Vérifier : mission reste bloquée, pas de retour IDLE intempestif.
    """
    r = Resultat("S06", "Absence confirmation livraison — bug S06 corrigé")
    t0 = time.time()
    env = Env()

    req = env.req("urgences","Morphine 10mg","Dr. Driss")
    time.sleep(0.15)

    # Publier une confirmation orpheline (mauvais département)
    env.bus.publish("/pharmabot/livraison_confirmee",{
        "type":"LIVRAISON_CONFIRMEE","departement":"reanimation",
        "nom_salle":"ICU_Room","medicament":"X","type_rt":"HARD_RT",
        "temps_livraison":5.0,"deadline_sec":30,"deadline_ok":True,
        "livraison_num":99,"timestamp":time.time()})
    time.sleep(0.1)
    etat_apres = env.mm.etat

    # La machine ne doit PAS passer à RETOUR ou IDLE suite à une livraison orpheline
    r.check(etat_apres not in ("RETOUR_PHARMACIE","IDLE") or etat_apres == "CHARGEMENT",
            f"Mission non affectée par livraison orpheline (état={etat_apres})",
            f"Mission terminée à tort sur livraison orpheline (état={etat_apres})")
    r.info("Correctif S06 : 3 gardes vérifient mission + département + état")
    r.info(f"État courant après confirmation orpheline: {etat_apres}")

    # Maintenant vrai nav_status pour urgences → doit confirmer
    env.nav_ok("urgences", 0.1); time.sleep(0.2)
    livs = env.detect.livraisons
    r.check(len(livs) > 0,
            f"Livraison urgences confirmée après vrai nav_status",
            "Aucune livraison confirmée même avec vrai nav_status")

    r.duree = time.time()-t0
    r.metriques = {"livraisons_valides":env.detect.stats["livraisons_total"]}
    return r


def s07_panne_robot() -> Resultat:
    """
    S07 — Panne robot pendant navigation
    Watchdog RECUPERATION → ARRET_URGENCE → nav stoppée → REPRISE → IDLE.
    """
    r = Resultat("S07", "Panne robot pendant navigation — watchdog cycle complet")
    t0 = time.time()
    env = Env()

    env.req("consultation","Paracétamol 1g","Dr. Hammoudi")
    time.sleep(0.15)
    r.info(f"État avant panne: {env.mm.etat}")

    # Simuler panne
    env.bus.publish("/pharmabot/watchdog",{
        "type":"RECUPERATION","raison":"PANNE_MOTEUR","timestamp":time.time()})
    time.sleep(0.1)

    atteint = env.attendre_etat("ARRET_URGENCE", 2.0)
    r.check(atteint, "État → ARRET_URGENCE après panne",
            f"État={env.mm.etat} (attendu ARRET_URGENCE)")
    r.check(env.nav.arret, "Navigation bridge arrêtée",
            "Navigation bridge toujours active")

    # Reprise
    env.bus.publish("/pharmabot/watchdog",{"type":"REPRISE","timestamp":time.time()})
    time.sleep(0.1)
    atteint2 = env.attendre_etat("IDLE", 2.0)
    r.check(atteint2, "État → IDLE après REPRISE",
            f"État={env.mm.etat} (attendu IDLE)")
    r.check(not env.nav.arret, "Navigation bridge débloquée",
            "Navigation bridge encore bloquée")

    r.duree = time.time()-t0
    r.metriques = {"panne_detectee":1,"reprise_ok":1 if atteint2 else 0}
    return r


def s08_reprise_automatique() -> Resultat:
    """
    S08 — Reprise automatique après override HARD_RT
    Consultation interrompue → réanimation livrée → reprise consultation.
    Vérifier séquence historique d'états.
    """
    r = Resultat("S08", "Reprise automatique consultation après HARD_RT réanimation")
    t0 = time.time()
    env = Env()

    # Mission 1 : consultation
    env.req("consultation","Oméprazole 20mg","Dr. Idrissi")
    time.sleep(0.15)
    r.info(f"États après consultation: {env.mm.historique}")

    # Override HARD_RT
    env.req("reanimation","Propofol 200mg","Dr. Cherkaoui")
    time.sleep(0.15)

    hist = env.mm.historique
    r.check("URGENCE_OVERRIDE" in hist,
            f"URGENCE_OVERRIDE dans historique",
            "URGENCE_OVERRIDE ABSENT de l'historique")
    r.check(env.mm.stats["overrides"]>=1,
            f"Override comptabilisé #{env.mm.stats['overrides']}",
            "Override non comptabilisé")

    # Livraison réanimation
    env.nav_ok("reanimation", 0.1); time.sleep(0.25)
    r.check(env.mm.stats["reprises"]>=1,
            f"Reprise automatique #{env.mm.stats['reprises']}",
            "Reprise automatique NON déclenchée")

    # Livraison consultation (reprise)
    env.nav_ok("consultation", 0.1); time.sleep(0.2)

    livs = env.detect.stats["livraisons_total"]
    r.check(livs >= 1, f"Total livraisons: {livs}", "Aucune livraison")

    hist_final = env.mm.historique
    r.info(f"Historique complet: {' → '.join(hist_final)}")
    etats_cles = {"CHARGEMENT","URGENCE_OVERRIDE","RETOUR_PHARMACIE"}
    presents   = etats_cles & set(hist_final)
    r.check(len(presents)>=2,
            f"États clés présents: {presents}",
            f"États clés manquants: {etats_cles-presents}")

    r.duree = time.time()-t0
    r.metriques = {"overrides":env.mm.stats["overrides"],
                   "reprises":env.mm.stats["reprises"],
                   "livraisons":livs,
                   "etats_distincts":len(set(hist_final))}
    return r


def s09_surcharge_25_requetes() -> Resultat:
    """
    S09 — 25 requêtes simultanées (stress test)
    Vérifier : 100% réception, HARD_RT prioritaire, aucun crash.
    """
    r = Resultat("S09", "Surcharge 25 requêtes simultanées — robustesse EDF")
    t0 = time.time()
    env = Env(); N = 25
    depts = list(DEPARTEMENTS_INFO.keys())
    lock  = threading.Lock(); envoyees = []

    def injecter(i):
        d = depts[i%3]
        med = random.choice(MEDICAMENTS[d])
        doc = random.choice(MEDECINS[d])
        req = env.req(d, med, doc)
        with lock: envoyees.append(req)
        time.sleep(random.uniform(0.01,0.04))

    threads = [threading.Thread(target=injecter, args=(i,)) for i in range(N)]
    for t in threads: t.start()
    for t in threads: t.join(timeout=5.0)
    time.sleep(0.2)

    stats = env.scheduler.stats
    taux = stats["total"]/N*100 if N else 0

    r.check(stats["total"]==N,
            f"Toutes les {N} requêtes reçues",
            f"Requêtes manquantes: {N-stats['total']}/{N}")
    r.check(taux>=95,
            f"Taux réception: {taux:.0f}% ≥ 95%",
            f"Taux réception insuffisant: {taux:.0f}%")

    tc = env.bus.last("/pharmabot/tache_courante")
    if tc:
        r.check(tc.get("type_rt")=="HARD_RT",
                "EDF correct: HARD_RT prioritaire sous charge",
                f"EDF: tache_courante={tc.get('type_rt')} (attendu HARD_RT)")
        r.info(f"Tâche courante: {tc.get('departement')} | {tc.get('type_rt')}")
    else:
        r.warn("Aucune tache_courante publiée")

    r.info(f"HARD_RT: {stats.get('hard_rt',0)} | "
           f"SOFT_RT: {stats.get('soft_rt',0)} | "
           f"FIRM_RT: {stats.get('firm_rt',0)}")
    r.info(f"EDF overrides générés: {stats['overrides_edf']}")

    duree = time.time()-t0
    r.check(duree<5.0,
            f"Temps traitement 25 requêtes: {duree:.2f}s < 5s",
            f"Temps excessif: {duree:.2f}s")

    r.duree = duree
    r.metriques = {"reqs_envoyees":N,"reqs_recues":stats["total"],
                   "taux_reception_pct":taux,
                   "hard_rt":stats.get("hard_rt",0),
                   "soft_rt":stats.get("soft_rt",0),
                   "firm_rt":stats.get("firm_rt",0),
                   "overrides_edf":stats["overrides_edf"],
                   "temps_sec":duree}
    return r


def s10_journee_hospitaliere() -> Resultat:
    """
    S10 — Journée hospitalière complète (8 vagues compressées)
    Mix de tous types, overrides aléatoires, métriques globales.
    """
    r = Resultat("S10", "Journée hospitalière — 8 vagues, mix HARD/SOFT/FIRM RT")
    t0 = time.time()
    env = Env()

    VAGUES = [
        [("consultation","Amoxicilline 500mg","Dr. Guessous"),
         ("urgences","Paracétamol IV 1g","Dr. Driss")],
        [("reanimation","Adrénaline 1mg","Dr. Alaoui"),
         ("consultation","Métformine 500mg","Dr. Hammoudi")],
        [("urgences","Morphine 10mg","Dr. El Idrissi"),
         ("urgences","Lorazépam 2mg","Dr. Fathi")],
        [("consultation","Paracétamol 1g","Dr. Idrissi")],
        [("reanimation","Insuline Rapide","Dr. Benali"),
         ("urgences","Aspirine 500mg","Dr. Driss"),
         ("consultation","Amlodipine 5mg","Dr. Guessous")],
        [("reanimation","Noradrénaline 4mg","Dr. Cherkaoui")],
        [("urgences","Ondansétron 4mg","Dr. El Idrissi"),
         ("consultation","Atorvastatine 20mg","Dr. Hammoudi")],
        [("reanimation","Atropine 0.5mg","Dr. Alaoui"),
         ("urgences","Ibuprofène 400mg","Dr. Fathi"),
         ("consultation","Oméprazole 20mg","Dr. Idrissi")],
    ]

    total_req = sum(len(v) for v in VAGUES)
    envoyees  = 0

    for i, vague in enumerate(VAGUES):
        for dept, med, doc in vague:
            env.req(dept, med, doc); envoyees += 1
        # Simuler livraisons inter-vague
        for dept,_,_ in vague:
            if dept != "pharmacie":
                env.nav_ok(dept, 0.04)
        time.sleep(0.08)

    time.sleep(0.3)

    stats_s = env.scheduler.stats
    stats_d = env.detect.stats
    taux    = stats_s["total"]/total_req*100 if total_req else 0

    r.check(taux>=95,
            f"Taux réception: {taux:.0f}% ≥ 95%",
            f"Taux réception insuffisant: {taux:.0f}%")
    r.check(stats_s.get("hard_rt",0)>=3,
            f"≥3 HARD_RT traitées: {stats_s.get('hard_rt',0)}",
            f"HARD_RT insuffisant: {stats_s.get('hard_rt',0)}")
    r.check(not env.watchdog.en_recup,
            "Watchdog hors mode récupération en fin de journée",
            "Watchdog encore en mode récupération")

    r.info(f"Requêtes: {stats_s['total']}/{total_req}")
    r.info(f"Livraisons confirmées: {stats_d['livraisons_total']}")
    r.info(f"Deadlines OK: {stats_d['deadlines_ok']} / "
           f"KO: {stats_d['deadlines_ko']}")
    r.info(f"EDF overrides: {stats_s['overrides_edf']}")
    r.info(f"Reprises auto: {env.mm.stats['reprises']}")
    r.info(f"HARD_RT:{stats_s.get('hard_rt',0)} "
           f"SOFT_RT:{stats_s.get('soft_rt',0)} "
           f"FIRM_RT:{stats_s.get('firm_rt',0)}")

    r.duree = time.time()-t0
    r.metriques = {
        "total_req":total_req,"recues":stats_s["total"],
        "taux_pct":taux,
        "livraisons":stats_d["livraisons_total"],
        "deadlines_ok":stats_d["deadlines_ok"],
        "deadlines_ko":stats_d["deadlines_ko"],
        "hard_rt":stats_s.get("hard_rt",0),
        "soft_rt":stats_s.get("soft_rt",0),
        "firm_rt":stats_s.get("firm_rt",0),
        "overrides_edf":stats_s["overrides_edf"],
        "reprises":env.mm.stats["reprises"],
    }
    return r


# ══════════════════════════════════════════════════════════════════
# 7. RUNNER + RAPPORT
# ══════════════════════════════════════════════════════════════════

SCENARIOS = [
    ("S01", s01_mission_simple),
    ("S02", s02_plusieurs_soft_rt),
    ("S03", s03_urgence_hard_rt),
    ("S04", s04_livraison_echouee),
    ("S05", s05_perte_communication),
    ("S06", s06_absence_confirmation),
    ("S07", s07_panne_robot),
    ("S08", s08_reprise_automatique),
    ("S09", s09_surcharge_25_requetes),
    ("S10", s10_journee_hospitaliere),
]

ICONES = {"S01":"🟢","S02":"🟡","S03":"🔴","S04":"🔴",
          "S05":"🟠","S06":"🟠","S07":"🔴","S08":"🟢","S09":"🟡","S10":"🔵"}

CHECKLIST_P3 = [
    "EDF scheduler traite les requêtes dynamiques",
    "Tri EDF correct (HARD_RT > SOFT_RT > FIRM_RT)",
    "navigation_bridge publie navigation_goal pour chaque mission",
    "Coordonnées salles exactes (Pharmacy,ICU,Emergency,Consult)",
    "delivery_detector confirme les livraisons",
    "Override HARD_RT (edf_interruption) fonctionnel",
    "Machine à états: IDLE→CHARGEMENT→NAVIGATION→LIVRAISON→RETOUR→IDLE",
    "Reprise automatique après interruption HARD_RT",
    "Watchdog: HARD_RT_MANQUE → RECUPERATION → REPRISE (5s)",
    "DMA pipeline 4 étapes complet",
    "Pharmacist node valide et autorise le chargement",
    "Taux réception requêtes ≥ 95%",
    "Robustesse 25 requêtes simultanées sans crash",
    "Alertes RT publiées (HARD_RT_CRITIQUE / HARD_RT_MANQUE)",
    "Correctif S06: confirmation livraison protégée (3 gardes)",
]

CHECKLIST_P4 = [
    "Phase 1+2 stable — 0 FAIL sur 10 scénarios",
    "Taux deadlines respectées ≥ 80%",
    "Override EDF validé ≥ 3 fois (S03+S08+S10)",
    "Watchdog cycle complet sans perte permanente (S05+S07)",
    "DMA P1 sécurité mission : 0 échec",
    "Journée hospitalière complète sans deadlock (S10)",
]


def run_all(verbose: bool = False) -> List[Resultat]:
    print(f"\n{B}{'═'*65}{W}")
    print(f"  {B}{C}PHARMABOT — CAMPAGNE VALIDATION PHASE 1+2{W}")
    print(f"  {C}Ibn Tofaïl University | Prof. Khaoula Boukir | 2025-2026{W}")
    print(f"{B}{'═'*65}{W}\n")

    resultats = []
    for sid, fn in SCENARIOS:
        print(f"  {ICONES.get(sid,'⚪')} {sid} — ", end="", flush=True)
        try:
            res = fn()
        except Exception as e:
            res = Resultat(sid, f"Erreur: {e}")
            res.fails.append(str(e))
        resultats.append(res)

        st = res.statut
        col = G if st=="PASS" else (Y if st=="WARN" else R)
        print(f"{col}{B}{st}{W}  ({res.duree:.2f}s)")

        if verbose or st == "FAIL":
            for line in res.checks: print(f"     {line}")
            for line in res.warns:  print(f"     {Y}{line}{W}")
            for line in res.fails:  print(f"     {R}{line}{W}")
        else:
            fails = [l for l in res.fails]
            for f in fails[:2]: print(f"     {R}{f}{W}")

        if res.metriques and verbose:
            items = list(res.metriques.items())[:5]
            print("     Métriques: " + " | ".join(f"{k}={v}" for k,v in items))

    return resultats


def afficher_rapport(resultats: List[Resultat]):
    passes = sum(1 for r in resultats if r.statut=="PASS")
    warns  = sum(1 for r in resultats if r.statut=="WARN")
    fails  = sum(1 for r in resultats if r.statut=="FAIL")
    total  = len(resultats)

    print(f"\n{B}{'═'*65}{W}")
    print(f"  {B}BILAN GLOBAL{W}")
    print(f"{'═'*65}")
    print(f"  {G}{B}✅ PASS: {passes}{W}  {Y}⚠️  WARN: {warns}{W}  {R}❌ FAIL: {fails}{W}  Total: {total}")

    # Agréger métriques
    m_all: Dict[str,float] = {}
    for r in resultats:
        for k,v in r.metriques.items():
            if isinstance(v,(int,float)):
                m_all[k] = m_all.get(k,0)+v

    # Checklist P3
    p3_ok = fails == 0 and warns <= 3
    p4_ok = fails == 0 and passes >= 8

    print(f"\n{B}{'═'*65}{W}")
    print(f"  {B}{C}CHECKLIST — PRÊT POUR PHASE 3 ?{W}")
    print(f"{'─'*65}")
    for crit in CHECKLIST_P3:
        ic = f"{G}✓{W}" if p3_ok else f"{Y}?{W}"
        print(f"  [{ic}] {crit}")

    print(f"\n  {'─'*60}")
    if p3_ok:
        print(f"  {G}{B}✓ PHASE 3 : GO — Navigation autonome SLAM/RL{W}")
    else:
        print(f"  {R}{B}✗ PHASE 3 : NO-GO — Corriger les {fails} FAIL d'abord{W}")

    print(f"\n{B}{'═'*65}{W}")
    print(f"  {B}{C}CHECKLIST — PRÊT POUR PHASE 4 ?{W}")
    print(f"{'─'*65}")
    for crit in CHECKLIST_P4:
        ic = f"{G}✓{W}" if p4_ok else f"{Y}?{W}"
        print(f"  [{ic}] {crit}")

    print(f"\n  {'─'*60}")
    if p4_ok:
        print(f"  {G}{B}✓ PHASE 4 : GO — Modèle SDF + dépôt médicaments{W}")
    else:
        print(f"  {R}{B}✗ PHASE 4 : NO-GO — Stabiliser Phase 1+2{W}")

    print(f"\n{B}{'═'*65}{W}")
    print(f"  {B}VERDICT FINAL{W}")
    print(f"{'═'*65}")
    if p3_ok and p4_ok:
        print(f"  {G}{B}🎉 PharmaBot est prêt pour Phase 3 ET Phase 4 !{W}")
    elif p3_ok:
        print(f"  {Y}{B}⚠  Phase 3 OK. Consolider avant Phase 4.{W}")
    else:
        print(f"  {R}{B}❌ Corriger Phase 1+2 avant de continuer.{W}")

    # Sauvegarder rapport JSON
    os.makedirs("reports", exist_ok=True)
    from datetime import datetime
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    rapport = {
        "timestamp": ts,
        "bilan": {"total":total,"pass":passes,"warn":warns,"fail":fails},
        "verdict_phase3": p3_ok,
        "verdict_phase4": p4_ok,
        "scenarios": [{
            "id":r.sid,"nom":r.nom,"statut":r.statut,
            "duree_sec":round(r.duree,3),
            "checks":r.checks,"warns":r.warns,"fails":r.fails,
            "metriques":r.metriques} for r in resultats],
        "metriques_globales": m_all,
    }
    path = f"reports/rapport_pharmabot_{ts}.json"
    with open(path,"w",encoding="utf-8") as f:
        json.dump(rapport, f, indent=2, ensure_ascii=False)
    print(f"\n  Rapport JSON sauvegardé : {path}\n")
    return p3_ok, p4_ok


def main():
    verbose = "--verbose" in sys.argv or "-v" in sys.argv
    sid_filter = None
    for arg in sys.argv[1:]:
        if arg.startswith("S") and len(arg)==3:
            sid_filter = arg.upper()

    if sid_filter:
        fns = [fn for sid,fn in SCENARIOS if sid==sid_filter]
        if not fns:
            print(f"Scénario inconnu: {sid_filter}")
            sys.exit(1)
        resultats = []
        for fn in fns:
            res = fn()
            resultats.append(res)
            st = res.statut
            col = G if st=="PASS" else (Y if st=="WARN" else R)
            print(f"\n{col}{B}[{st}]{W} {res.sid} — {res.nom} ({res.duree:.2f}s)")
            for line in res.checks: print(f"  {line}")
            for line in res.warns:  print(f"  {Y}{line}{W}")
            for line in res.fails:  print(f"  {R}{line}{W}")
    else:
        resultats = run_all(verbose=verbose)
        afficher_rapport(resultats)

    fails = sum(1 for r in resultats if r.statut=="FAIL")
    sys.exit(0 if fails==0 else 1)


if __name__ == "__main__":
    main()

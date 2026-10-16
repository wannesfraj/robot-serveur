export const UIController = (() => {
  const $ = id => document.getElementById(id);
  let batteryTestInitialized = false;
  let batteryTestPendingPct = null;
  let batteryControlsBusy = false;

  function log(message) {
    const element = document.createElement("div");
    element.textContent =
      `[${new Date().toLocaleTimeString()}] ${message}`;

    $("log").appendChild(element);
    $("log").scrollTop = $("log").scrollHeight;

    while ($("log").children.length > 30) {
      $("log").firstChild.remove();
    }
  }

  function setStatus(connected, text) {
    $("dot").classList.toggle("on", connected);
    $("status").textContent = text;
    if (!connected) $("driveStatus").textContent = "Robot déconnecté : commandes indisponibles.";
    else $("driveStatus").textContent = "Prêt : maintenez une direction pour déplacer le robot.";
  }

  function updateDrive(held, sent, reason = "") {
    document.querySelectorAll("#drive [data-d]").forEach(button => {
      button.classList.toggle("active", Boolean(held[button.dataset.d]) && sent);
    });
    if (!sent) {
      $("driveStatus").textContent = reason || "Commande non envoyée : robot déconnecté.";
      return;
    }
    const directions = [];
    if (held.up !== held.down) directions.push(held.up ? "Avance" : "Recule");
    if (held.left !== held.right) directions.push(held.left ? "Tourne à gauche" : "Tourne à droite");
    $("driveStatus").textContent = directions.length ? `Commande : ${directions.join(" et ")}` : "Commande d’arrêt envoyée.";
  }

  function updateSafety(safety = {}) {
    const reasons = {
      sensor_not_ready: "En attente des capteurs du robot.",
      sensor_timeout: "Données LiDAR interrompues : relancez le scan.",
      localization_timeout: "Position du robot indisponible.",
      emergency_stop: "Arrêt d’urgence actif."
    };
    let text = "";
    if (safety?.stop_active) {
      const reason = safety.reason || "";
      if (reason.startsWith("rotation_obstacle=")) text = "Rotation bloquée : un obstacle est trop proche du robot.";
      else if (reason.startsWith("front_obstacle=")) text = "Avance bloquée : un obstacle se trouve devant le robot.";
      else if (reason.startsWith("rear_obstacle=")) text = "Recul bloqué : un obstacle se trouve derrière le robot. Avancez pour vous éloigner.";
      else if (reason.startsWith("denied_zone=")) text = `Navigation stopped at denied zone: ${reason.slice(12)}.`;
      else text = reasons[reason] || "Robot arrêté : en attente d’une commande récente.";
    }
    $("safetyStatus").textContent = text;
    $("safetyStatus").hidden = !text;
  }

  function updateBatteryIndicator(battery) {
    const box = $("batteryBox");
    if (!battery || !Number.isFinite(Number(battery.soc_pct))) { box.hidden = true; return; }
    const soc = Math.max(0, Math.min(100, Number(battery.soc_pct)));
    const reserve = Number(battery.config?.reserve_pct ?? 20);
    box.hidden = false;
    $("batteryPct").textContent = `${soc.toFixed(1)} %`;
    const fill = $("batteryFill");
    fill.style.width = `${soc}%`;
    fill.className = soc < reserve ? "critical" : (soc < reserve + 10 ? "low" : "");
    $("batteryReserve").style.left = `${reserve}%`;
    const note = $("batteryNote");
    if (soc < reserve) { note.textContent = `Sous le seuil de ${reserve.toFixed(0)} % : les nouveaux trajets seront refusés.`; note.className = "warn"; }
    else { note.textContent = `Seuil de refus : ${reserve.toFixed(0)} %`; note.className = ""; }
  }

  function updateBattery(battery, estimate = null, navigation = null) {
    updateBatteryIndicator(battery);
    if (!battery || !Number.isFinite(Number(battery.soc_pct))) return;
    const soc = Math.max(0, Math.min(100, Number(battery.soc_pct)));
    const reserve = Number(battery.config?.reserve_pct ?? 15);
    const note = $("batteryNote");
    if (soc < reserve) {
      note.textContent = `Sous ${reserve.toFixed(0)} % : les nouveaux trajets seront refusés.`;
      note.className = "warn";
    } else if (soc <= 20) {
      note.textContent = `Charge basse : une mission peut finir entre ${reserve.toFixed(0)} et 20 %.`;
      note.className = "warn";
    } else {
      note.textContent = `Refus sous ${reserve.toFixed(0)} % · avertissement jusqu’à 20 %`;
      note.className = "";
    }

    const test = $("batteryTest");
    const simulated = battery.simulated === true;
    test.hidden = !simulated;
    if (simulated) {
      const slider = $("batteryTestSoc");
      if (!batteryTestInitialized) {
        slider.value = String(Math.round(soc));
        $("batteryTestValue").textContent = `${slider.value} %`;
        batteryTestInitialized = true;
      }
      if (batteryTestPendingPct !== null && Math.abs(soc - batteryTestPendingPct) < 0.15) {
        slider.value = String(Math.round(soc));
        $("batteryTestValue").textContent = `${slider.value} %`;
        $("batteryTestStatus").textContent = `Charge de test appliquée : ${soc.toFixed(1)} %.`;
        $("batteryTestStatus").className = "";
        batteryTestPendingPct = null;
      }
      const activeStates = new Set(["goal_active", "planning", "path_ready", "following", "aligning", "approaching", "braking", "settling", "turning_to_path", "recovering", "waiting_for_route", "battery_returning"]);
      const deliveryState = navigation?.delivery?.state;
      const activeDelivery = ["planning", "approaching", "waiting", "returning"].includes(deliveryState);
      const busy = activeDelivery || activeStates.has(navigation?.state);
      slider.disabled = busy;
      $("applyBatteryTest").disabled = busy || batteryTestPendingPct !== null;
      if (batteryTestPendingPct !== null) slider.disabled = true;
      if (busy && !batteryControlsBusy) $("batteryTestStatus").textContent = "Terminez ou arrêtez la mission avant de changer la charge.";
      else if (!busy && batteryControlsBusy && batteryTestPendingPct === null)
        $("batteryTestStatus").textContent = "Régler la charge avant de lancer une mission.";
      batteryControlsBusy = busy;
    }

    const forecast = $("batteryEstimate");
    if (!estimate) {
      forecast.textContent = "L’estimation complète s’affiche avant le départ.";
      forecast.className = "";
    } else if (estimate.accepted === false) {
      forecast.textContent = estimate.reason || "Mission refusée : batterie insuffisante.";
      forecast.className = "error";
    } else if (estimate.soc_after_pct !== null && estimate.soc_after_pct !== undefined
      && Number.isFinite(Number(estimate.distance_m)) && Number.isFinite(Number(estimate.soc_after_pct))) {
      const consumption = Number(estimate.required_pct ?? (Number(estimate.soc_pct) - Number(estimate.soc_after_pct)));
      forecast.textContent = `Parcours estimé : ${Number(estimate.distance_m).toFixed(1)} m · consommation ${consumption.toFixed(1)} % · arrivée ${Number(estimate.soc_after_pct).toFixed(1)} %.`;
      if (estimate.warning) {
        forecast.textContent += " Attention : rechargez après cette mission.";
        forecast.className = "warn";
      } else forecast.className = "";
    } else {
      forecast.textContent = estimate.reason || "Estimation de batterie indisponible.";
      forecast.className = estimate.reason ? "error" : "";
    }
  }

  function setBatteryTestPending(socPct) {
    batteryTestPendingPct = Number(socPct);
    $("batteryTestStatus").textContent = `Application de la charge à ${batteryTestPendingPct.toFixed(0)} %…`;
    $("batteryTestStatus").className = "";
    $("batteryTestSoc").disabled = true;
    $("applyBatteryTest").disabled = true;
  }

  function batteryTestError(reason) {
    batteryTestPendingPct = null;
    $("batteryTestStatus").textContent = reason || "Impossible d’appliquer la charge de test.";
    $("batteryTestStatus").className = "warn";
    $("batteryTestSoc").disabled = false;
    $("applyBatteryTest").disabled = false;
  }

  function setScanning(scanning) {
    $("scanBadge").textContent =
      scanning ? "SCAN ON" : "SCAN OFF";

    $("scan").textContent =
      scanning ? "Stop LiDAR Scan" : "Start 360° LiDAR Scan";

    $("scan").classList.toggle("active", scanning);
  }

  function updateRobot(robot) {
    $("x").textContent = robot.x.toFixed(2);
    $("y").textContent = robot.y.toFixed(2);
    $("yaw").textContent =
      (robot.yaw * 180 / Math.PI).toFixed(1);
  }

  function updateCounters({ hits, occupied, packets }) {
    if (hits !== undefined) {
      $("hits").textContent = Number(hits).toLocaleString();
    }
    if (occupied !== undefined) {
      $("occ").textContent = Number(occupied).toLocaleString();
    }
    if (packets !== undefined) {
      $("packets").textContent = Number(packets).toLocaleString();
    }
  }

  return {
    $,
    log,
    setStatus,
    updateDrive,
    updateSafety,
    updateBattery,
    setBatteryTestPending,
    batteryTestError,
    setScanning,
    updateRobot,
    updateCounters
  };
})();

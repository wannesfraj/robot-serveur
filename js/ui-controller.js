export const UIController = (() => {
  const $ = id => document.getElementById(id);
  let batteryTestInitialized = false;
  let batteryActualSoc = null;
  let batteryTestPendingPct = null;
  let batteryTestPendingRequestId = null;
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

  function rebaseBatteryEstimate(estimate, soc, reserve, warningPct) {
    if (!estimate || !Number.isFinite(Number(estimate.distance_m))) return estimate;
    const reportedAfter = estimate.soc_after_pct === null || estimate.soc_after_pct === undefined
      ? NaN : Number(estimate.soc_after_pct);
    const reportedSoc = estimate.soc_pct === null || estimate.soc_pct === undefined
      ? NaN : Number(estimate.soc_pct);
    const reportedUse = estimate.required_pct === null || estimate.required_pct === undefined
      ? NaN : Number(estimate.required_pct);
    const consumption = Number.isFinite(reportedUse)
      ? reportedUse
      : (Number.isFinite(reportedSoc) && Number.isFinite(reportedAfter)
        ? reportedSoc - reportedAfter : NaN);
    if (!Number.isFinite(consumption) || consumption < 0) return estimate;

    const arrival = soc - consumption;
    const accepted = arrival >= reserve;
    return {
      ...estimate,
      soc_pct: soc,
      required_pct: consumption,
      soc_after_pct: arrival,
      accepted,
      warning: accepted && arrival <= warningPct
    };
  }

  function updateBattery(battery, estimate = null, navigation = null) {
    updateBatteryIndicator(battery);
    if (!battery || !Number.isFinite(Number(battery.soc_pct))) return;
    const soc = Math.max(0, Math.min(100, Number(battery.soc_pct)));
    batteryActualSoc = soc;
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

    const activeStates = new Set(["goal_active", "planning", "path_ready", "following", "aligning", "approaching", "braking", "settling", "turning_to_path", "recovering", "waiting_for_route", "battery_returning"]);
    const deliveryState = navigation?.delivery?.state;
    const activeDelivery = ["planning", "approaching", "waiting", "returning"].includes(deliveryState);
    const busy = activeDelivery || activeStates.has(navigation?.state);
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
      if (batteryTestPendingPct !== null && battery.request_id === batteryTestPendingRequestId
          && Math.abs(soc - batteryTestPendingPct) < 0.15) {
        slider.value = String(Math.round(soc));
        $("batteryTestValue").textContent = `${slider.value} %`;
        $("batteryTestStatus").textContent = `Charge de test appliquée : ${soc.toFixed(1)} %.`;
        $("batteryTestStatus").className = "";
        batteryTestPendingPct = null;
      }
      slider.disabled = busy;
      $("applyBatteryTest").disabled = busy || batteryTestPendingPct !== null;
      if (batteryTestPendingPct !== null) slider.disabled = true;
      if (busy && !batteryControlsBusy) $("batteryTestStatus").textContent = "Terminez ou arrêtez la mission avant de changer la charge.";
      else if (!busy && batteryControlsBusy && batteryTestPendingPct === null)
        $("batteryTestStatus").textContent = "Régler la charge avant de lancer une mission.";
      batteryControlsBusy = busy;
    }

    const forecast = $("batteryEstimate");
    const currentEstimate = estimate && !busy
      ? rebaseBatteryEstimate(estimate, soc, reserve,
          Number(battery.config?.warning_pct ?? 20))
      : estimate;
    if (!currentEstimate) {
      forecast.textContent = "L\u2019estimation compl\u00e8te s\u2019affiche avant le d\u00e9part.";
      forecast.className = "";
    } else if (currentEstimate.soc_after_pct !== null && currentEstimate.soc_after_pct !== undefined
      && Number.isFinite(Number(currentEstimate.distance_m))
      && Number.isFinite(Number(currentEstimate.soc_after_pct))) {
      const consumption = Number(currentEstimate.required_pct ??
        (Number(currentEstimate.soc_pct) - Number(currentEstimate.soc_after_pct)));
      const arrival = Number(currentEstimate.soc_after_pct);
      forecast.textContent = `Parcours estim\u00e9 : ${Number(currentEstimate.distance_m).toFixed(1)} m \u00b7 consommation ${consumption.toFixed(1)} % \u00b7 arriv\u00e9e ${arrival.toFixed(1)} %.`;
      if (currentEstimate.accepted === false) {
        forecast.textContent += ` Mission refus\u00e9e : l\u2019arriv\u00e9e pr\u00e9vue est sous le seuil de ${reserve.toFixed(0)} %.`;
        forecast.className = "error";
      } else if (currentEstimate.warning) {
        forecast.textContent += " Attention : rechargez apr\u00e8s cette mission.";
        forecast.className = "warn";
      } else forecast.className = "";
    } else if (currentEstimate.accepted === false) {
      forecast.textContent = currentEstimate.reason || "Mission refus\u00e9e : batterie insuffisante.";
      forecast.className = "error";
    } else {
      forecast.textContent = currentEstimate.reason || "Estimation de batterie indisponible.";
      forecast.className = currentEstimate.reason ? "error" : "";
    }
  }

  function setBatteryTestPending(socPct, requestId) {
    batteryTestPendingPct = Number(socPct);
    batteryTestPendingRequestId = requestId;
    $("batteryTestStatus").textContent = `Application de la charge à ${batteryTestPendingPct.toFixed(0)} %…`;
    $("batteryTestStatus").className = "";
    $("batteryTestSoc").disabled = true;
    $("applyBatteryTest").disabled = true;
  }

  function batteryTestError(reason) {
    batteryTestPendingPct = null;
    batteryTestPendingRequestId = null;
    $("batteryTestStatus").textContent = reason || "Impossible d’appliquer la charge de test.";
    $("batteryTestStatus").className = "warn";
    if (batteryActualSoc !== null) {
      $("batteryTestSoc").value = String(Math.round(batteryActualSoc));
      $("batteryTestValue").textContent = `${$("batteryTestSoc").value} %`;
    }
    $("batteryTestSoc").disabled = batteryControlsBusy;
    $("applyBatteryTest").disabled = batteryControlsBusy;
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

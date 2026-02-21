let eventSource = null;
let sseReconnectTimer = null;
let sseRetryDelayMs = 1500;
let hasReceivedSseEvent = false;
let pwmRequestInFlight = false;
let pendingPwmValue = null;
let latestTelemetry = null;

const SSE_RETRY_BASE_MS = 1500;
const SSE_RETRY_MAX_MS = 12000;
const ACH_WINDOW_MS = 5000;
const MIN_BUILDING_PRESSURE_FOR_ACH = 9;
const FULL_APERTURE_DIAMETER_CM = 31;
const SEA_LEVEL_AIR_DENSITY = 1.225;
const DEFAULT_ALTITUDE_M = 650;
const SETTINGS_KEY = 'blower_ui_v2';
const OTA_CHUNK_SIZE = 768;

const API = Object.freeze({
    events: '/events',
    pwm: '/api/pwm',
    led: '/api/led',
    relay: '/api/relay',
    calibrate: '/api/calibrate',
    otaStatus: '/api/ota/status',
    otaBegin: '/api/ota/begin',
    otaChunk: '/api/ota/chunk',
    otaFinish: '/api/ota/finish',
    otaApply: '/api/ota/apply',
});

const PID = { kp: 1.15, ki: 0.2, kd: 0.06, deadband: 0.8, base: 62 };
const RAMP_RATIO = 0.97;

const state = {
    activeMode: 'manual',
    autoTestType: 'n50',
    semiTargetPa: null,
    achHistory: [],
    lastManualSpeed: 40,
    baseline: { done: false, everCalibrated: false, fanOffset: null, envelopeOffset: null },
    control: {
        active: false,
        stage: 'idle',
        targetPa: 50,
        lastSetPwm: null,
        pid: { integral: 0, lastError: 0, lastTime: Date.now() },
    },
    sensorHealth: { fanEverValid: false, envelopeEverValid: false, faultCycles: 0 },
    ota: { uploadInProgress: false, applyInProgress: false, versionTouched: false },
};

const $ = (id) => document.getElementById(id);

const refs = {
    connDot: $('connDot'), connLabel: $('connLabel'), statusMsg: $('statusMsg'),
    calGate: $('calGate'), calTitle: $('calTitle'), calDetail: $('calDetail'),
    calibrateBtn: $('calibrateBtn'), calProgress: $('calProgress'),
    calProgressBar: $('calProgressBar'), calProgressLabel: $('calProgressLabel'),
    calResult: $('calResult'),
    calRecal: $('calRecal'), calRecalStatus: $('calRecalStatus'),
    recalibrateBtn: $('recalibrateBtn'),
    calRecalProgress: $('calRecalProgress'), calRecalBar: $('calRecalBar'),
    calRecalLabel: $('calRecalLabel'),
    heroAch: $('heroAch'), heroAchInstant: $('heroAchInstant'),
    heroModeLabel: $('heroModeLabel'), heroPa: $('heroPa'), heroTarget: $('heroTarget'),
    modeCard: $('modeCard'),
    modeTabs: document.querySelectorAll('.mode-tab'),
    panelManual: $('panelManual'), panelSemi: $('panelSemi'), panelAuto: $('panelAuto'),
    pwmDisplay: $('pwmDisplay'), fanSlider: $('fanSlider'),
    quickBtns: document.querySelectorAll('[data-speed]'),
    pressureBtns: document.querySelectorAll('[data-pa]'),
    manualStopBtn: $('manualStopBtn'),
    semiStopBtn: $('semiStopBtn'),
    autoN50Btn: $('autoN50Btn'), autoN75Btn: $('autoN75Btn'),
    startTestBtn: $('startTestBtn'), stopTestBtn: $('stopTestBtn'),
    fanPaVal: $('fanPaVal'), fanTempVal: $('fanTempVal'), fanStatusPill: $('fanStatusPill'),
    bldgPaVal: $('bldgPaVal'), bldgTempVal: $('bldgTempVal'), bldgStatusPill: $('bldgStatusPill'),
    lineFreqVal: $('lineFreqVal'), sensorPwmVal: $('sensorPwmVal'),
    settingsBtn: $('settingsBtn'), closeDrawerBtn: $('closeDrawerBtn'),
    settingsDrawer: $('settingsDrawer'), drawerBackdrop: $('drawerBackdrop'),
    buildingVolume: $('buildingVolume'), fanApertureCm: $('fanApertureCm'),
    altitude: $('altitude'), fanCoefC: $('fanCoefC'), fanCoefN: $('fanCoefN'),
    anemoSpeed: $('anemoSpeed'), applyAnemoBtn: $('applyAnemoBtn'),
    apertureAreaLabel: $('apertureAreaLabel'),
    otaVersionInput: $('otaVersionInput'), otaFileInput: $('otaFileInput'),
    otaUploadBtn: $('otaUploadBtn'), otaApplyBtn: $('otaApplyBtn'),
    otaProgressBar: $('otaProgressBar'), otaProgressLabel: $('otaProgressLabel'),
    fwVersionLabel: $('fwVersionLabel'), fwVersion: $('fwVersion'),
    lastUpdate: $('lastUpdate'),
};

/* ── Utilities ── */

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const pad2 = (v) => String(v).padStart(2, '0');
const fmtTime = (d = new Date()) => `${pad2(d.getHours())}:${pad2(d.getMinutes())}:${pad2(d.getSeconds())}`;
const toNum = (v, fb = 0) => { const n = Number(v); return Number.isFinite(n) ? n : fb; };
const toBool = (v) => v === true || v === 1 || v === 'true' || v === '1';
const fmt = (v, u, d = 1) => Number.isFinite(v) ? `${v.toFixed(d)} ${u}` : `-- ${u}`;

function isCalibrated() {
    return state.baseline.done === true;
}

function setStatus(text) {
    if (refs.statusMsg) refs.statusMsg.textContent = text;
}

function stampUpdate() {
    if (refs.lastUpdate) refs.lastUpdate.textContent = `Last update: ${fmtTime()}`;
}

function setConn(kind, label) {
    if (!refs.connDot || !refs.connLabel) return;
    refs.connDot.classList.remove('ok', 'err');
    if (kind === 'ok') { refs.connDot.classList.add('ok'); refs.connLabel.textContent = label || 'Connected'; }
    else if (kind === 'err') { refs.connDot.classList.add('err'); refs.connLabel.textContent = label || 'Error'; }
    else { refs.connLabel.textContent = label || 'Offline'; }
}

/* ── OTA helpers ── */

function setOtaProgress(pct, label) {
    const v = clamp(toNum(pct, 0), 0, 100);
    if (refs.otaProgressBar) refs.otaProgressBar.value = v;
    if (refs.otaProgressLabel) refs.otaProgressLabel.textContent = label || `${Math.round(v)}%`;
}

function setOtaBusy(busy) {
    if (refs.otaUploadBtn) refs.otaUploadBtn.disabled = busy;
    if (refs.otaApplyBtn) refs.otaApplyBtn.disabled = busy;
}

function bytesToBase64(u8) {
    let bin = '';
    for (let i = 0; i < u8.length; i += 0x8000) {
        bin += String.fromCharCode(...u8.subarray(i, i + 0x8000));
    }
    return btoa(bin);
}

function createCrc32Table() {
    const t = new Uint32Array(256);
    for (let i = 0; i < 256; i++) {
        let v = i;
        for (let b = 0; b < 8; b++) v = (v & 1) ? (0xedb88320 ^ (v >>> 1)) : (v >>> 1);
        t[i] = v >>> 0;
    }
    return t;
}
const CRC32_TABLE = createCrc32Table();

function crc32(u8) {
    let c = 0xffffffff;
    for (let i = 0; i < u8.length; i++) c = CRC32_TABLE[(c ^ u8[i]) & 0xff] ^ (c >>> 8);
    return (c ^ 0xffffffff) >>> 0;
}

/* ── API helpers ── */

async function postJson(url, body) {
    const r = await fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
    const txt = await r.text();
    let parsed = null;
    if (txt) { try { parsed = JSON.parse(txt); } catch {} }
    if (!r.ok) {
        const reason = parsed?.reason || '';
        const err = new Error(reason ? `HTTP ${r.status} ${reason}` : `HTTP ${r.status}`);
        err.responsePayload = parsed;
        throw err;
    }
    return parsed;
}

async function sendCmd(endpoint, value) {
    await postJson(API[endpoint] || `/api/${endpoint}`, { value });
}

async function sendPwmCoalesced(value) {
    const v = clamp(Math.round(toNum(value, 0)), 0, 100);
    if (pwmRequestInFlight) { pendingPwmValue = v; return; }
    pwmRequestInFlight = true;
    try { await sendCmd('pwm', v); }
    finally {
        pwmRequestInFlight = false;
        if (pendingPwmValue !== null && pendingPwmValue !== v) {
            const next = pendingPwmValue; pendingPwmValue = null;
            await sendPwmCoalesced(next);
        } else { pendingPwmValue = null; }
    }
}

function requestPwm(value) {
    const v = clamp(Math.round(toNum(value, 0)), 0, 100);
    if (state.control.lastSetPwm === v) return;
    state.control.lastSetPwm = v;
    updatePwmDisplay(v);
    sendPwmCoalesced(v).then(() => setConn('ok', 'Connected')).catch(() => setConn('err', 'PWM error'));
}

/* ── Settings persistence ── */

function collectSettings() {
    return {
        buildingVolume: refs.buildingVolume?.value ?? '',
        fanApertureCm: refs.fanApertureCm?.value ?? '',
        altitude: refs.altitude?.value ?? String(DEFAULT_ALTITUDE_M),
        fanCoefC: refs.fanCoefC?.value ?? '', fanCoefN: refs.fanCoefN?.value ?? '',
        autoTestType: state.autoTestType,
    };
}

function saveSettings() {
    try { localStorage.setItem(SETTINGS_KEY, JSON.stringify(collectSettings())); } catch {}
}

function loadSettings() {
    try {
        const raw = localStorage.getItem(SETTINGS_KEY);
        if (!raw) { if (refs.altitude) refs.altitude.value = String(DEFAULT_ALTITUDE_M); return; }
        const s = JSON.parse(raw);
        if (refs.buildingVolume && s.buildingVolume !== undefined) refs.buildingVolume.value = String(s.buildingVolume);
        if (refs.fanApertureCm && s.fanApertureCm !== undefined) refs.fanApertureCm.value = String(s.fanApertureCm);
        if (refs.altitude) refs.altitude.value = s.altitude !== undefined ? String(s.altitude) : String(DEFAULT_ALTITUDE_M);
        if (refs.fanCoefC && s.fanCoefC !== undefined) refs.fanCoefC.value = String(s.fanCoefC);
        if (refs.fanCoefN && s.fanCoefN !== undefined) refs.fanCoefN.value = String(s.fanCoefN);
        if (s.autoTestType === 'n50' || s.autoTestType === 'n75') state.autoTestType = s.autoTestType;
    } catch {
        if (refs.altitude) refs.altitude.value = String(DEFAULT_ALTITUDE_M);
    }
}

/* ── Calibration ── */

function updateCalGate(calState, calPct, calFan, calEnv) {
    const calibrated = calState === 2;
    state.baseline.done = calibrated;
    if (calibrated) {
        state.baseline.fanOffset = calFan;
        state.baseline.envelopeOffset = calEnv;
        state.baseline.everCalibrated = true;
    }

    /* Recalibration path: if we already calibrated once, never re-lock */
    const isRecal = state.baseline.everCalibrated;
    if (refs.modeCard) refs.modeCard.classList.toggle('locked', !calibrated && !isRecal);

    /* ── Compact recal bar (shown after first calibration) ── */
    if (isRecal && refs.calRecal) {
        refs.calRecal.hidden = false;
        if (calState === 1) {
            /* Recalibrating: show inline progress */
            if (refs.calRecalStatus) refs.calRecalStatus.textContent = 'Recalibrating...';
            if (refs.calRecalProgress) refs.calRecalProgress.hidden = false;
            if (refs.calRecalBar) refs.calRecalBar.value = calPct;
            const secs = Math.round(calPct / 10);
            if (refs.calRecalLabel) refs.calRecalLabel.textContent = `${secs}s / 10s`;
            if (refs.recalibrateBtn) refs.recalibrateBtn.hidden = true;
        } else {
            /* Idle or done: show recalibrate button */
            if (refs.calRecalStatus) refs.calRecalStatus.textContent = '\u2713 Calibrated';
            if (refs.calRecalProgress) refs.calRecalProgress.hidden = true;
            if (refs.recalibrateBtn) { refs.recalibrateBtn.hidden = false; refs.recalibrateBtn.disabled = false; }
        }
    } else if (refs.calRecal) {
        refs.calRecal.hidden = true;
    }

    /* ── Full cal-gate (only shown for first-time calibration) ── */
    if (!refs.calGate) return;
    if (isRecal) return; /* skip full gate updates if already calibrated once */

    if (calState === 1) {
        refs.calGate.classList.remove('ok');
        if (refs.calTitle) refs.calTitle.textContent = 'Calibrating...';
        if (refs.calDetail) refs.calDetail.textContent = 'Sampling sensor zero offsets';
        if (refs.calibrateBtn) refs.calibrateBtn.hidden = true;
        if (refs.calProgress) refs.calProgress.hidden = false;
        if (refs.calProgressBar) refs.calProgressBar.value = calPct;
        const secs = Math.round(calPct / 10);
        if (refs.calProgressLabel) refs.calProgressLabel.textContent = `Sampling... ${secs}s / 10s`;
        if (refs.calResult) refs.calResult.hidden = true;
    } else if (calibrated) {
        refs.calGate.classList.add('ok');
        if (refs.calTitle) refs.calTitle.textContent = 'Sensors Calibrated';
        if (refs.calDetail) refs.calDetail.textContent = '';
        if (refs.calibrateBtn) refs.calibrateBtn.hidden = true;
        if (refs.calProgress) refs.calProgress.hidden = true;
        if (refs.calResult) {
            refs.calResult.hidden = false;
            refs.calResult.textContent = `Fan ${calFan.toFixed(2)} Pa \u00b7 Bldg ${calEnv.toFixed(2)} Pa`;
        }
        resetAch();
    } else {
        refs.calGate.classList.remove('ok');
        if (refs.calTitle) refs.calTitle.textContent = 'Zero Calibration Required';
        if (refs.calDetail) refs.calDetail.textContent = 'Ensure fan is off and no pressure across sensors';
        if (refs.calibrateBtn) { refs.calibrateBtn.hidden = false; refs.calibrateBtn.textContent = 'Calibrate Sensors'; refs.calibrateBtn.disabled = false; }
        if (refs.calProgress) refs.calProgress.hidden = true;
        if (refs.calResult) refs.calResult.hidden = true;
    }
}

/* ── Mode management ── */

function switchMode(mode) {
    if (state.activeMode === mode) return;
    if (state.control.active) stopControl(false);
    state.activeMode = mode;

    refs.modeTabs.forEach((tab) => tab.classList.toggle('active', tab.dataset.mode === mode));
    if (refs.panelManual) refs.panelManual.classList.toggle('hidden', mode !== 'manual');
    if (refs.panelSemi) refs.panelSemi.classList.toggle('hidden', mode !== 'semi');
    if (refs.panelAuto) refs.panelAuto.classList.toggle('hidden', mode !== 'auto');
    updateTargetDisplay();
}

function getActiveTarget() {
    if (state.control.active) return state.control.targetPa;
    if (state.activeMode === 'auto') return state.autoTestType === 'n75' ? 75 : 50;
    if (state.activeMode === 'semi' && state.semiTargetPa) return state.semiTargetPa;
    return 50;
}

function getAchLabel() {
    const t = getActiveTarget();
    if (t === 50) return 'n50';
    if (t === 75) return 'n75';
    return `${t} Pa`;
}

function updateTargetDisplay() {
    const t = getActiveTarget();
    if (refs.heroTarget) refs.heroTarget.textContent = `${t} Pa`;
    if (refs.heroModeLabel) refs.heroModeLabel.textContent = getAchLabel();
}

function updateAutoButtons() {
    if (refs.autoN50Btn) refs.autoN50Btn.classList.toggle('active', state.autoTestType === 'n50');
    if (refs.autoN75Btn) refs.autoN75Btn.classList.toggle('active', state.autoTestType === 'n75');
}

/* ── Display helpers ── */

function updatePwmDisplay(value) {
    const v = clamp(Math.round(toNum(value, 0)), 0, 100);
    if (refs.pwmDisplay) refs.pwmDisplay.textContent = String(v);
    if (refs.sensorPwmVal) refs.sensorPwmVal.textContent = `${v} %`;
    if (refs.fanSlider && document.activeElement !== refs.fanSlider) refs.fanSlider.value = String(v);
}

function updatePill(el, ok, hasReading) {
    if (!el) return;
    el.classList.remove('ok', 'err');
    if (!hasReading) { el.textContent = '--'; return; }
    el.textContent = ok ? 'OK' : 'ERR';
    el.classList.add(ok ? 'ok' : 'err');
}

function updateApertureLabel() {
    if (!refs.apertureAreaLabel) return;
    const d = clamp(toNum(refs.fanApertureCm?.value, FULL_APERTURE_DIAMETER_CM), 5, 60) / 100;
    refs.apertureAreaLabel.textContent = `${(Math.PI * (d / 2) ** 2).toFixed(4)} m²`;
}

/* ── Drawer ── */

function openDrawer() {
    if (refs.settingsDrawer) { refs.settingsDrawer.classList.add('open'); refs.settingsDrawer.setAttribute('aria-hidden', 'false'); }
    if (refs.drawerBackdrop) refs.drawerBackdrop.hidden = false;
    fetchOtaStatus();
}

function closeDrawer() {
    if (refs.settingsDrawer) { refs.settingsDrawer.classList.remove('open'); refs.settingsDrawer.setAttribute('aria-hidden', 'true'); }
    if (refs.drawerBackdrop) refs.drawerBackdrop.hidden = true;
}

/* ── Flow / ACH computation ── */

function getFlowInputs() {
    const vol = toNum(refs.buildingVolume?.value, 0);
    const C = toNum(refs.fanCoefC?.value, 0);
    const n = toNum(refs.fanCoefN?.value, 0);
    const alt = toNum(refs.altitude?.value, DEFAULT_ALTITUDE_M);
    const apCm = clamp(toNum(refs.fanApertureCm?.value, FULL_APERTURE_DIAMETER_CM), 5, 60);
    const area = Math.PI * ((apCm / 100) / 2) ** 2;
    const fullArea = Math.PI * ((FULL_APERTURE_DIAMETER_CM / 100) / 2) ** 2;
    return { vol, C, n, alt, area, scale: area / fullArea };
}

function airDensity(alt, tempC) {
    const a = clamp(toNum(alt, DEFAULT_ALTITUDE_M), 0, 6000);
    const pPa = 101325 * Math.pow(1 - 2.25577e-5 * a, 5.25588);
    const tK = (Number.isFinite(tempC) ? tempC : 20) + 273.15;
    return pPa / (287.05 * tK);
}

function computeAch(fanPa, bldgPa, bldgTempC) {
    const target = getActiveTarget();
    const f = getFlowInputs();
    if (!Number.isFinite(fanPa) || !Number.isFinite(bldgPa)) return { valid: false };
    if (f.vol <= 0 || f.C <= 0 || f.n <= 0) return { valid: false };
    const fanAbs = Math.abs(fanPa);
    const bldgAbs = Math.abs(bldgPa);
    if (fanAbs <= 0 || bldgAbs < MIN_BUILDING_PRESSURE_FOR_ACH) return { valid: false };

    const rho = airDensity(f.alt, bldgTempC);
    const df = Math.sqrt(SEA_LEVEL_AIR_DENSITY / rho);
    const qFan = f.C * Math.pow(fanAbs, f.n) * f.scale * df;
    const qRef = qFan * Math.pow(target / bldgAbs, 0.65);
    const instant = qRef / f.vol;
    if (!Number.isFinite(instant) || instant < 0) return { valid: false };

    const now = Date.now();
    state.achHistory.push({ t: now, v: instant });
    while (state.achHistory.length && now - state.achHistory[0].t > ACH_WINDOW_MS) state.achHistory.shift();
    const avg = state.achHistory.reduce((s, i) => s + i.v, 0) / Math.max(state.achHistory.length, 1);
    return { valid: true, instant, avg };
}

function resetAch() { state.achHistory.length = 0; }

function updateHero(fanPa, bldgPa, bldgTempC) {
    if (refs.heroPa) refs.heroPa.textContent = fmt(bldgPa, 'Pa', 1);
    const ach = computeAch(fanPa, bldgPa, bldgTempC);
    if (!refs.heroAch || !refs.heroAchInstant) return;
    if (!ach.valid) { refs.heroAch.textContent = '--'; refs.heroAchInstant.textContent = '--'; return; }
    refs.heroAch.textContent = ach.avg.toFixed(2);
    refs.heroAchInstant.textContent = ach.instant.toFixed(2);
}

/* ── Control loop ── */

function startControl(targetPa) {
    state.control.active = true;
    state.control.stage = 'ramp';
    state.control.targetPa = targetPa;
    state.control.lastSetPwm = null;
    state.control.pid = { integral: 0, lastError: targetPa, lastTime: Date.now() };
    updateTargetDisplay();
    setStatus(`Ramping to ${targetPa} Pa...`);
    sendCmd('led', 1).catch(() => {});
    sendCmd('relay', 1).catch(() => {});
    requestPwm(100);
}

function stopControl(stopFan) {
    state.control.active = false;
    state.control.stage = 'idle';
    state.control.lastSetPwm = null;
    state.control.pid = { integral: 0, lastError: 0, lastTime: Date.now() };

    refs.pressureBtns.forEach((b) => b.classList.remove('active'));
    if (refs.semiStopBtn) refs.semiStopBtn.disabled = true;
    state.semiTargetPa = null;

    if (stopFan) {
        requestPwm(0);
        sendCmd('relay', 0).catch(() => {});
        sendCmd('led', 0).catch(() => {});
    }
    updateTargetDisplay();
}

function processControl(bldgPa, sensorOk) {
    if (!state.control.active || !Number.isFinite(bldgPa) || !sensorOk) return;
    const measured = Math.abs(bldgPa);
    const target = state.control.targetPa;

    if (state.control.stage === 'ramp') {
        if (measured >= target * RAMP_RATIO) {
            state.control.stage = 'control';
            state.control.pid = { integral: 0, lastError: target - measured, lastTime: Date.now() };
            setStatus(`Holding at ${target} Pa`);
        } else { requestPwm(100); }
        return;
    }

    const now = Date.now();
    let dt = (now - state.control.pid.lastTime) / 1000;
    if (dt <= 0) dt = 0.1;
    state.control.pid.lastTime = now;

    const err = target - measured;
    if (Math.abs(err) <= PID.deadband) return;

    state.control.pid.integral = clamp(state.control.pid.integral + err * dt, -80, 80);
    const deriv = (err - state.control.pid.lastError) / dt;
    state.control.pid.lastError = err;

    const correction = PID.kp * err + PID.ki * state.control.pid.integral + PID.kd * deriv;
    const raw = clamp(PID.base + correction, 0, 100);
    const prev = state.control.lastSetPwm;
    const smoothed = Number.isFinite(prev) ? prev * 0.7 + raw * 0.3 : raw;
    requestPwm(smoothed);
}

/* ── Telemetry ── */

function extractTelemetry(data) {
    const rawFanPa = toNum(data.dp1_pressure ?? data.dp_pressure, NaN);
    const rawFanT = toNum(data.dp1_temperature ?? data.dp_temperature, NaN);
    const rawEnvPa = toNum(data.dp2_pressure, NaN);
    const rawEnvT = toNum(data.dp2_temperature, NaN);
    const fOff = Number.isFinite(state.baseline.fanOffset) ? state.baseline.fanOffset : 0;
    const eOff = Number.isFinite(state.baseline.envelopeOffset) ? state.baseline.envelopeOffset : 0;

    return {
        pwm: Number.isFinite(data.pwm) ? Number(data.pwm) : NaN,
        relay: toBool(data.relay),
        autoHold: toBool(data.led),
        freqHz: toNum(data.frequency, NaN),
        fanPa: Number.isFinite(rawFanPa) ? rawFanPa - fOff : NaN,
        fanT: rawFanT,
        fanOk: toBool(data.dp1_ok ?? data.dp_ok),
        envPa: Number.isFinite(rawEnvPa) ? rawEnvPa - eOff : NaN,
        envT: rawEnvT,
        envOk: toBool(data.dp2_ok),
        rawFanPa, rawEnvPa,
        fw: typeof data.fw === 'string' ? data.fw : null,
    };
}

function handleTelemetry(data, detail) {
    if (!data) return;
    latestTelemetry = data;
    const t = extractTelemetry(data);

    if (t.fanOk) state.sensorHealth.fanEverValid = true;
    if (t.envOk) state.sensorHealth.envelopeEverValid = true;

    if (Number.isFinite(t.pwm)) { updatePwmDisplay(t.pwm); state.lastManualSpeed = t.pwm; }

    if (refs.fanPaVal) refs.fanPaVal.textContent = fmt(t.fanPa, 'Pa', 1);
    if (refs.fanTempVal) refs.fanTempVal.textContent = fmt(t.fanT, '°C', 1);
    updatePill(refs.fanStatusPill, t.fanOk, Number.isFinite(t.rawFanPa));

    if (refs.bldgPaVal) refs.bldgPaVal.textContent = fmt(t.envPa, 'Pa', 1);
    if (refs.bldgTempVal) refs.bldgTempVal.textContent = fmt(t.envT, '°C', 1);
    updatePill(refs.bldgStatusPill, t.envOk, Number.isFinite(t.rawEnvPa));

    if (refs.lineFreqVal) refs.lineFreqVal.textContent = fmt(t.freqHz, 'Hz', 1);

    updateHero(t.fanPa, t.envPa, t.envT);
    processControl(t.envPa, t.envOk);

    /* Calibration state from firmware */
    const calState = toNum(data.cal, 0);
    const calPct = toNum(data.cal_pct, 0);
    const calFan = toNum(data.cal_fan, 0);
    const calEnv = toNum(data.cal_env, 0);
    updateCalGate(calState, calPct, calFan, calEnv);

    if (t.fw) {
        if (refs.fwVersion) refs.fwVersion.textContent = `Firmware: ${t.fw}`;
        if (refs.fwVersionLabel) refs.fwVersionLabel.textContent = t.fw;
        if (refs.otaVersionInput && !state.ota.versionTouched && !refs.otaVersionInput.value.trim()) {
            refs.otaVersionInput.value = t.fw;
        }
    }

    if (!t.fanOk || !t.envOk) state.sensorHealth.faultCycles++;
    else state.sensorHealth.faultCycles = 0;

    if (state.sensorHealth.faultCycles >= 3) {
        setConn('err', 'Sensor error');
        setStatus('ADP910 read error: check wiring');
    } else {
        setConn('ok', 'Connected');
    }
    stampUpdate();
}

/* ── SSE ── */

function closeSSE() {
    if (sseReconnectTimer) { clearTimeout(sseReconnectTimer); sseReconnectTimer = null; }
    if (eventSource) { eventSource.close(); eventSource = null; }
}

function scheduleReconnect() {
    if (sseReconnectTimer) return;
    sseReconnectTimer = setTimeout(() => {
        sseReconnectTimer = null;
        sseRetryDelayMs = Math.min(sseRetryDelayMs * 2, SSE_RETRY_MAX_MS);
        connectSSE();
    }, sseRetryDelayMs);
}

function connectSSE() {
    closeSSE();
    try { eventSource = new EventSource(API.events); }
    catch { setConn('err', 'SSE error'); scheduleReconnect(); return; }

    eventSource.onopen = () => {
        sseRetryDelayMs = SSE_RETRY_BASE_MS;
        hasReceivedSseEvent = false;
        setConn('ok', 'Connected');
    };

    eventSource.onmessage = (ev) => {
        try {
            handleTelemetry(JSON.parse(ev.data), 'SSE');
            hasReceivedSseEvent = true;
        } catch {}
    };

    eventSource.onerror = () => {
        closeSSE();
        setConn('err', hasReceivedSseEvent ? 'Reconnecting...' : 'Connecting...');
        hasReceivedSseEvent = false;
        scheduleReconnect();
    };
}

/* ── OTA ── */

function applyOtaStatus(p) {
    if (!p || typeof p !== 'object') return;
    if (typeof p.firmware_version === 'string') {
        if (refs.fwVersionLabel) refs.fwVersionLabel.textContent = p.firmware_version;
        if (refs.fwVersion) refs.fwVersion.textContent = `Firmware: ${p.firmware_version}`;
    }
    const s = String(p.state || 'idle');
    const pct = clamp(toNum(p.progress_percent, 0), 0, 100);
    const sv = typeof p.staged_version === 'string' ? p.staged_version : '';
    const le = typeof p.last_error === 'string' ? p.last_error : '';
    if (s === 'receiving') setOtaProgress(pct, `Uploading ${Math.round(pct)}%`);
    else if (s === 'ready') setOtaProgress(100, `Staged (${sv || 'unlabeled'})`);
    else if (s === 'applying') setOtaProgress(pct, 'Applying...');
    else if (s === 'error') setOtaProgress(pct, `Error: ${le || 'unknown'}`);
    else setOtaProgress(0, 'OTA idle');
}

async function fetchOtaStatus() {
    try {
        const r = await fetch(API.otaStatus, { cache: 'no-store' });
        if (r.ok) applyOtaStatus(await r.json());
    } catch {}
}

async function uploadOta() {
    if (state.ota.uploadInProgress || state.ota.applyInProgress) return;
    const file = refs.otaFileInput?.files?.[0];
    if (!file) { setStatus('Select a .bin file first'); return; }

    state.ota.uploadInProgress = true;
    setOtaBusy(true);
    try {
        setStatus('Uploading firmware...');
        setOtaProgress(0, 'Reading file...');
        const buf = new Uint8Array(await file.arrayBuffer());
        const crcVal = crc32(buf);
        const activeVer = refs.fwVersionLabel?.textContent?.trim() || '0.0.0-dev';
        const ver = (refs.otaVersionInput?.value || activeVer).trim() || activeVer;

        await postJson(API.otaBegin, { size: buf.length, crc32: crcVal, version: ver });
        let off = 0;
        while (off < buf.length) {
            const chunk = buf.subarray(off, off + OTA_CHUNK_SIZE);
            await postJson(API.otaChunk, { offset: off, data: bytesToBase64(chunk) });
            off += chunk.length;
            setOtaProgress((off * 100) / buf.length, `Uploading ${Math.round((off * 100) / buf.length)}%`);
        }
        await postJson(API.otaFinish, {});
        setStatus('Firmware uploaded. Press Apply to reboot.');
        setOtaProgress(100, `Uploaded (${ver})`);
        await fetchOtaStatus();
    } catch (e) {
        setStatus('OTA upload failed');
        setOtaProgress(0, 'Upload error');
        await fetchOtaStatus();
    } finally {
        state.ota.uploadInProgress = false;
        setOtaBusy(false);
    }
}

async function applyOta() {
    if (state.ota.uploadInProgress || state.ota.applyInProgress) return;
    state.ota.applyInProgress = true;
    setOtaBusy(true);
    setStatus('Applying firmware...');
    try {
        setOtaProgress(100, 'Applying and rebooting...');
        await postJson(API.otaApply, {});
        setConn('err', 'Rebooting...');
    } catch {
        setStatus('OTA apply failed');
    } finally {
        state.ota.applyInProgress = false;
        setOtaBusy(false);
        setTimeout(fetchOtaStatus, 5000);
    }
}

/* ── Anemometer calibration ── */

function applyAnemoCalibration() {
    const speed = toNum(refs.anemoSpeed?.value, NaN);
    if (!Number.isFinite(speed) || speed <= 0 || !latestTelemetry) return;
    const t = extractTelemetry(latestTelemetry);
    if (!Number.isFinite(t.fanPa) || t.fanPa <= 0) { setStatus('No fan ΔP for calibration'); return; }
    const f = getFlowInputs();
    if (f.n <= 0 || f.scale <= 0) return;
    const qMeasured = speed * f.area * 3600;
    const rho = airDensity(f.alt, t.envT);
    const df = Math.sqrt(SEA_LEVEL_AIR_DENSITY / rho);
    const newC = qMeasured / (Math.pow(Math.abs(t.fanPa), f.n) * f.scale * df);
    if (!Number.isFinite(newC) || newC <= 0) return;
    if (refs.fanCoefC) refs.fanCoefC.value = newC.toFixed(1);
    resetAch();
    setStatus(`C recalibrated: ${newC.toFixed(1)}`);
    if (latestTelemetry) handleTelemetry(latestTelemetry, 'C updated');
}

/* ── Debounce ── */

function debounce(fn, ms) {
    let id = null;
    return (...args) => { if (id) clearTimeout(id); id = setTimeout(() => fn(...args), ms); };
}

/* ── Event binding ── */

function bindEvents() {
    refs.modeTabs.forEach((tab) => {
        tab.addEventListener('click', () => switchMode(tab.dataset.mode));
    });

    refs.settingsBtn?.addEventListener('click', openDrawer);
    refs.closeDrawerBtn?.addEventListener('click', closeDrawer);
    refs.drawerBackdrop?.addEventListener('click', closeDrawer);

    const triggerCalibration = async (sourceBtn) => {
        stopControl(false);
        requestPwm(0);
        if (sourceBtn) sourceBtn.disabled = true;
        try {
            await postJson(API.calibrate, {});
            setStatus('Calibration started (10s sampling)');
        } catch (e) {
            if (sourceBtn) sourceBtn.disabled = false;
            setConn('err', 'Calibration failed');
            setStatus(`Calibration failed: ${e.message}`);
        }
    };

    refs.calibrateBtn?.addEventListener('click', () => triggerCalibration(refs.calibrateBtn));
    refs.recalibrateBtn?.addEventListener('click', () => triggerCalibration(refs.recalibrateBtn));

    /* Manual mode */
    const debouncedPwm = debounce((v) => {
        requestPwm(v);
        if (v > 0) sendCmd('relay', 1).catch(() => {});
        else sendCmd('relay', 0).catch(() => {});
    }, 140);

    refs.fanSlider?.addEventListener('input', (e) => {
        if (!isCalibrated()) return;
        const v = clamp(toNum(e.target.value, 0), 0, 100);
        if (state.control.active) stopControl(false);
        state.lastManualSpeed = v;
        updatePwmDisplay(v);
        debouncedPwm(v);
    });

    refs.quickBtns.forEach((btn) => {
        btn.addEventListener('click', () => {
            if (!isCalibrated()) { setStatus('Calibrate first'); return; }
            const v = clamp(toNum(btn.dataset.speed, 0), 0, 100);
            if (state.control.active) stopControl(false);
            state.lastManualSpeed = v;
            requestPwm(v);
            if (v > 0) sendCmd('relay', 1).catch(() => {});
            else sendCmd('relay', 0).catch(() => {});
            setStatus(`Power: ${v}%`);
        });
    });

    refs.manualStopBtn?.addEventListener('click', () => {
        if (state.control.active) stopControl(false);
        requestPwm(0);
        sendCmd('relay', 0).catch(() => {});
        sendCmd('led', 0).catch(() => {});
        setStatus('Fan stopped');
    });

    /* Semi-auto mode */
    refs.pressureBtns.forEach((btn) => {
        btn.addEventListener('click', () => {
            if (!isCalibrated()) { setStatus('Calibrate first'); return; }
            const pa = toNum(btn.dataset.pa, 50);
            state.semiTargetPa = pa;
            refs.pressureBtns.forEach((b) => b.classList.toggle('active', b === btn));
            if (refs.semiStopBtn) refs.semiStopBtn.disabled = false;
            startControl(pa);
        });
    });

    refs.semiStopBtn?.addEventListener('click', () => {
        stopControl(true);
        setStatus('Pressure hold stopped');
    });

    /* Auto mode */
    refs.autoN50Btn?.addEventListener('click', () => {
        state.autoTestType = 'n50';
        updateAutoButtons();
        updateTargetDisplay();
        saveSettings();
        resetAch();
        if (latestTelemetry) handleTelemetry(latestTelemetry, 'n50');
    });

    refs.autoN75Btn?.addEventListener('click', () => {
        state.autoTestType = 'n75';
        updateAutoButtons();
        updateTargetDisplay();
        saveSettings();
        resetAch();
        if (latestTelemetry) handleTelemetry(latestTelemetry, 'n75');
    });

    refs.startTestBtn?.addEventListener('click', () => {
        if (!isCalibrated()) { setStatus('Calibrate first'); return; }
        const target = state.autoTestType === 'n75' ? 75 : 50;
        startControl(target);
    });

    refs.stopTestBtn?.addEventListener('click', () => {
        stopControl(true);
        setStatus('Test stopped');
    });

    /* Settings fields */
    refs.applyAnemoBtn?.addEventListener('click', applyAnemoCalibration);
    refs.otaUploadBtn?.addEventListener('click', uploadOta);
    refs.otaApplyBtn?.addEventListener('click', applyOta);
    refs.otaFileInput?.addEventListener('change', () => {
        const f = refs.otaFileInput?.files?.[0];
        setOtaProgress(0, f ? `Ready: ${f.name}` : 'OTA idle');
    });
    refs.otaVersionInput?.addEventListener('input', () => { state.ota.versionTouched = true; });

    [refs.buildingVolume, refs.fanApertureCm, refs.altitude, refs.fanCoefC, refs.fanCoefN].forEach((input) => {
        input?.addEventListener('change', () => {
            saveSettings(); updateApertureLabel(); resetAch();
            if (latestTelemetry) handleTelemetry(latestTelemetry, 'settings');
        });
        input?.addEventListener('input', saveSettings);
    });
}

/* ── Bootstrap ── */

function bootstrap() {
    loadSettings();
    updateAutoButtons();
    updateTargetDisplay();
    updateApertureLabel();
    updateCalGate(0, 0, 0, 0);
    updatePwmDisplay(0);
    setOtaProgress(0, 'OTA idle');
    setStatus('Idle');
    setConn('idle', 'Connecting');
    bindEvents();
    connectSSE();
}

window.addEventListener('load', bootstrap);
window.addEventListener('beforeunload', closeSSE);

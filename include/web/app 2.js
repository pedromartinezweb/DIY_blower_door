let eventSource = null;
let sseReconnectTimer = null;
let sseRetryDelayMs = 1500;
let hasReceivedSseEvent = false;
let pwmRequestInFlight = false;
let pendingPwmValue = null;
let latestTelemetry = null;
let otaStatusRefreshTimer = null;

const SSE_RETRY_BASE_MS = 1500;
const SSE_RETRY_MAX_MS = 12000;
const ACH_WINDOW_MS = 5000;
const OTA_STATUS_REFRESH_INTERVAL_MS = 15000;
const MIN_BUILDING_PRESSURE_FOR_ACH = 9;
const FULL_APERTURE_DIAMETER_CM = 31;
const SEA_LEVEL_AIR_DENSITY = 1.225;
const DEFAULT_ALTITUDE_M = 650;
const SETTINGS_STORAGE_KEY = 'blower_ui_settings_v1';
const OTA_CHUNK_SIZE_BYTES = 768;
const API_ROUTES = Object.freeze({
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

const CONTROL_RAMP_THRESHOLD_RATIO = 0.97;
const CONTROL_PID = {
    kp: 1.15,
    ki: 0.2,
    kd: 0.06,
    deadbandPa: 0.8,
    basePwm: 62,
};

const state = {
    selectedMode: 'n50',
    achHistory: [],
    lastManualSpeed: 40,
    baseline: {
        fanOffset: null,
        envelopeOffset: null,
        capturedAt: null,
    },
    control: {
        active: false,
        stage: 'idle',
        targetPa: 50,
        lastSetPwm: null,
        pid: {
            integral: 0,
            lastError: 0,
            lastTime: Date.now(),
        },
    },
    sensorHealth: {
        fanEverValid: false,
        envelopeEverValid: false,
    },
    ota: {
        uploadInProgress: false,
        applyInProgress: false,
        versionTouched: false,
    },
};

const refs = {
    connectionDot: document.getElementById('connectionDot'),
    connectionLabel: document.getElementById('connectionLabel'),
    connectionDetail: document.getElementById('connectionDetail'),
    firmwareVersionTop: document.getElementById('firmwareVersionTop'),
    firmwareVersionFooter: document.getElementById('firmwareVersionFooter'),
    lastUpdate: document.getElementById('lastUpdate'),
    sessionState: document.getElementById('sessionState'),

    heroPressure: document.getElementById('heroPressure'),
    heroTarget: document.getElementById('heroTarget'),
    heroAch: document.getElementById('heroAch'),
    heroAchInstant: document.getElementById('heroAchInstant'),
    heroModeLabel: document.getElementById('heroModeLabel'),

    modeN50Btn: document.getElementById('modeN50Btn'),
    modeN75Btn: document.getElementById('modeN75Btn'),
    settingsToggleBtn: document.getElementById('settingsToggleBtn'),
    settingsCloseBtn: document.getElementById('settingsCloseBtn'),
    settingsPanel: document.getElementById('settingsPanel'),
    settingsBackdrop: document.getElementById('settingsBackdrop'),

    buildingVolume: document.getElementById('buildingVolume'),
    fanApertureCm: document.getElementById('fanApertureCm'),
    altitude: document.getElementById('altitude'),
    fanCoefC: document.getElementById('fanCoefC'),
    fanCoefN: document.getElementById('fanCoefN'),
    anemoSpeed: document.getElementById('anemoSpeed'),
    applyAnemoBtn: document.getElementById('applyAnemoBtn'),
    apertureAreaLabel: document.getElementById('apertureAreaLabel'),
    otaVersionInput: document.getElementById('otaVersionInput'),
    otaFileInput: document.getElementById('otaFileInput'),
    otaUploadBtn: document.getElementById('otaUploadBtn'),
    otaApplyBtn: document.getElementById('otaApplyBtn'),
    otaProgressBar: document.getElementById('otaProgressBar'),
    otaProgressLabel: document.getElementById('otaProgressLabel'),
    firmwareVersionLabel: document.getElementById('firmwareVersionLabel'),

    calibrateBtn: document.getElementById('calibrateBtn'),
    baselineStatus: document.getElementById('baselineStatus'),
    startTestBtn: document.getElementById('startTestBtn'),
    stopTestBtn: document.getElementById('stopTestBtn'),

    fanPowerToggle: document.getElementById('fanPowerToggle'),
    autoHoldToggle: document.getElementById('autoHoldToggle'),
    fanSpeed: document.getElementById('fanSpeed'),
    fanSpeedValue: document.getElementById('fanSpeedValue'),

    dpVentPressureValue: document.getElementById('dpVentPressureValue'),
    dpVentTemperatureValue: document.getElementById('dpVentTemperatureValue'),
    dpVentStatus: document.getElementById('dpVentStatus'),
    dpBuildingPressureValue: document.getElementById('dpBuildingPressureValue'),
    dpBuildingTemperatureValue: document.getElementById('dpBuildingTemperatureValue'),
    dpBuildingStatus: document.getElementById('dpBuildingStatus'),
    fanFrequencyValue: document.getElementById('fanFrequencyValue'),
    sensorPwmValue: document.getElementById('sensorPwmValue'),
    quickButtons: document.querySelectorAll('[data-speed]'),
};

const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
const pad2 = (value) => String(value).padStart(2, '0');

function formatTime(date = new Date()) {
    return `${pad2(date.getHours())}:${pad2(date.getMinutes())}:${pad2(date.getSeconds())}`;
}

function toFiniteNumber(value, fallback = 0) {
    const parsed = Number(value);
    return Number.isFinite(parsed) ? parsed : fallback;
}

function parseBooleanFlag(value) {
    if (typeof value === 'boolean') {
        return value;
    }
    if (typeof value === 'number') {
        return value !== 0;
    }
    if (typeof value === 'string') {
        return value === 'true' || value === '1';
    }
    return false;
}

function formatMetric(value, unit, decimals = 1) {
    if (!Number.isFinite(value)) {
        return `-- ${unit}`;
    }
    return `${value.toFixed(decimals)} ${unit}`;
}

function setOtaProgress(progressPercent, label) {
    const normalized = clamp(toFiniteNumber(progressPercent, 0), 0, 100);
    if (refs.otaProgressBar) {
        refs.otaProgressBar.value = normalized;
    }
    if (refs.otaProgressLabel) {
        refs.otaProgressLabel.textContent = label || `${Math.round(normalized)}%`;
    }
}

function setOtaControlsBusy(isBusy) {
    if (refs.otaUploadBtn) {
        refs.otaUploadBtn.disabled = isBusy;
    }
    if (refs.otaApplyBtn) {
        refs.otaApplyBtn.disabled = isBusy;
    }
}

function bytesToBase64(uint8Array) {
    let binary = '';
    const chunkSize = 0x8000;
    for (let index = 0; index < uint8Array.length; index += chunkSize) {
        const chunk = uint8Array.subarray(index, index + chunkSize);
        binary += String.fromCharCode(...chunk);
    }
    return btoa(binary);
}

function createCrc32Table() {
    const table = new Uint32Array(256);
    for (let index = 0; index < 256; index += 1) {
        let value = index;
        for (let bit = 0; bit < 8; bit += 1) {
            if ((value & 1) !== 0) {
                value = 0xedb88320 ^ (value >>> 1);
            } else {
                value >>>= 1;
            }
        }
        table[index] = value >>> 0;
    }
    return table;
}

const CRC32_TABLE = createCrc32Table();

function computeCrc32(uint8Array) {
    let crc = 0xffffffff;
    for (let index = 0; index < uint8Array.length; index += 1) {
        const byte = uint8Array[index];
        crc = CRC32_TABLE[(crc ^ byte) & 0xff] ^ (crc >>> 8);
    }
    return (crc ^ 0xffffffff) >>> 0;
}

function setConnectionState(kind, detail) {
    if (!refs.connectionDot || !refs.connectionLabel || !refs.connectionDetail) {
        return;
    }

    refs.connectionDot.classList.remove('connected', 'error');

    if (kind === 'connected') {
        refs.connectionDot.classList.add('connected');
        refs.connectionLabel.textContent = 'Connected';
    } else if (kind === 'error') {
        refs.connectionDot.classList.add('error');
        refs.connectionLabel.textContent = 'Error';
    } else {
        refs.connectionLabel.textContent = 'Offline';
    }

    refs.connectionDetail.textContent = detail || '';
}

function setSessionState(text) {
    if (refs.sessionState) {
        refs.sessionState.textContent = `Status: ${text}`;
    }
}

function stampLastUpdate() {
    if (refs.lastUpdate) {
        refs.lastUpdate.textContent = `Last update: ${formatTime()}`;
    }
}

function getSelectedTargetPressure() {
    return state.selectedMode === 'n75' ? 75 : 50;
}

function updateModeButtons() {
    if (refs.modeN50Btn) {
        refs.modeN50Btn.classList.toggle('active', state.selectedMode === 'n50');
    }
    if (refs.modeN75Btn) {
        refs.modeN75Btn.classList.toggle('active', state.selectedMode === 'n75');
    }

    const targetPressure = getSelectedTargetPressure();
    if (refs.heroTarget) {
        refs.heroTarget.textContent = `${targetPressure} Pa`;
    }
    if (refs.heroModeLabel) {
        refs.heroModeLabel.textContent = state.selectedMode;
    }
    if (refs.startTestBtn) {
        refs.startTestBtn.textContent = `Start ${state.selectedMode} test`;
    }
}

function updateStatusPill(statusElement, isOk, hasReading) {
    if (!statusElement) {
        return;
    }

    statusElement.classList.remove('ok', 'err');
    if (!hasReading) {
        statusElement.textContent = '--';
        return;
    }

    statusElement.textContent = isOk ? 'OK' : 'ERR';
    statusElement.classList.add(isOk ? 'ok' : 'err');
}

function updateApertureAreaLabel() {
    if (!refs.apertureAreaLabel) {
        return;
    }

    const diameterCm = clamp(toFiniteNumber(refs.fanApertureCm?.value, FULL_APERTURE_DIAMETER_CM), 5, 60);
    const diameterM = diameterCm / 100;
    const areaM2 = Math.PI * Math.pow(diameterM / 2, 2);
    refs.apertureAreaLabel.textContent = `${areaM2.toFixed(4)} m²`;
}

function updateBaselineBadge() {
    if (!refs.baselineStatus) {
        return;
    }

    const fanOffset = state.baseline.fanOffset;
    const envelopeOffset = state.baseline.envelopeOffset;
    refs.baselineStatus.classList.remove('ok');

    if (!Number.isFinite(fanOffset) || !Number.isFinite(envelopeOffset)) {
        refs.baselineStatus.textContent = 'Zero calibration missing';
        return;
    }

    const capturedTime = state.baseline.capturedAt ? formatTime(state.baseline.capturedAt) : '--';
    refs.baselineStatus.textContent = `Zero OK · Fan ${fanOffset.toFixed(1)} Pa · Bldg ${envelopeOffset.toFixed(1)} Pa · ${capturedTime}`;
    refs.baselineStatus.classList.add('ok');
}

function updateManualSpeedDisplay(value) {
    const clampedValue = clamp(Math.round(toFiniteNumber(value, 0)), 0, 100);
    if (refs.fanSpeedValue) {
        refs.fanSpeedValue.textContent = String(clampedValue);
    }
    if (refs.sensorPwmValue) {
        refs.sensorPwmValue.textContent = `${clampedValue} %`;
    }
    if (refs.fanSpeed && document.activeElement !== refs.fanSpeed) {
        refs.fanSpeed.value = String(clampedValue);
    }
}

function getAirDensity(altitudeMeters, temperatureC) {
    const clampedAltitude = clamp(toFiniteNumber(altitudeMeters, DEFAULT_ALTITUDE_M), 0, 6000);
    const pressurePa = 101325 * Math.pow(1 - 2.25577e-5 * clampedAltitude, 5.25588);
    const tempKelvin = (Number.isFinite(temperatureC) ? temperatureC : 20) + 273.15;
    return pressurePa / (287.05 * tempKelvin);
}

function getFlowCalibrationInputs() {
    const volume = toFiniteNumber(refs.buildingVolume?.value, 0);
    const coefficientC = toFiniteNumber(refs.fanCoefC?.value, 0);
    const exponentN = toFiniteNumber(refs.fanCoefN?.value, 0);
    const altitude = toFiniteNumber(refs.altitude?.value, DEFAULT_ALTITUDE_M);

    const apertureCm = clamp(toFiniteNumber(refs.fanApertureCm?.value, FULL_APERTURE_DIAMETER_CM), 5, 60);
    const apertureArea = Math.PI * Math.pow((apertureCm / 100) / 2, 2);
    const fullArea = Math.PI * Math.pow((FULL_APERTURE_DIAMETER_CM / 100) / 2, 2);
    const apertureScale = apertureArea / fullArea;

    return {
        volume,
        coefficientC,
        exponentN,
        altitude,
        apertureArea,
        apertureScale,
    };
}

function collectUiSettings() {
    return {
        buildingVolume: refs.buildingVolume?.value ?? '',
        fanApertureCm: refs.fanApertureCm?.value ?? '',
        altitude: refs.altitude?.value ?? String(DEFAULT_ALTITUDE_M),
        fanCoefC: refs.fanCoefC?.value ?? '',
        fanCoefN: refs.fanCoefN?.value ?? '',
        selectedMode: state.selectedMode,
    };
}

function saveUiSettings() {
    try {
        localStorage.setItem(SETTINGS_STORAGE_KEY, JSON.stringify(collectUiSettings()));
    } catch (error) {
        console.warn('Could not save local settings:', error);
    }
}

function applyUiSettings(saved) {
    if (!saved || typeof saved !== 'object') {
        return;
    }

    if (refs.buildingVolume && saved.buildingVolume !== undefined) {
        refs.buildingVolume.value = String(saved.buildingVolume);
    }
    if (refs.fanApertureCm && saved.fanApertureCm !== undefined) {
        refs.fanApertureCm.value = String(saved.fanApertureCm);
    }
    if (refs.altitude) {
        refs.altitude.value = saved.altitude !== undefined ? String(saved.altitude) : String(DEFAULT_ALTITUDE_M);
    }
    if (refs.fanCoefC && saved.fanCoefC !== undefined) {
        refs.fanCoefC.value = String(saved.fanCoefC);
    }
    if (refs.fanCoefN && saved.fanCoefN !== undefined) {
        refs.fanCoefN.value = String(saved.fanCoefN);
    }
    if (saved.selectedMode === 'n50' || saved.selectedMode === 'n75') {
        state.selectedMode = saved.selectedMode;
    }
}

function loadUiSettings() {
    try {
        const raw = localStorage.getItem(SETTINGS_STORAGE_KEY);
        if (!raw) {
            if (refs.altitude) {
                refs.altitude.value = String(DEFAULT_ALTITUDE_M);
            }
            return;
        }
        const parsed = JSON.parse(raw);
        applyUiSettings(parsed);
    } catch (error) {
        console.warn('Could not load local settings:', error);
        if (refs.altitude) {
            refs.altitude.value = String(DEFAULT_ALTITUDE_M);
        }
    }
}

function setSettingsPanelOpen(open) {
    if (!refs.settingsPanel) {
        return;
    }

    refs.settingsPanel.classList.toggle('open', open);
    refs.settingsPanel.setAttribute('aria-hidden', open ? 'false' : 'true');
    if (refs.settingsBackdrop) {
        refs.settingsBackdrop.hidden = !open;
    }
}

function computeAchForMode(fanPressurePa, buildingPressurePa, buildingTemperatureC) {
    const targetPressure = getSelectedTargetPressure();
    const flowInputs = getFlowCalibrationInputs();

    if (!Number.isFinite(fanPressurePa) || !Number.isFinite(buildingPressurePa)) {
        return { valid: false };
    }

    if (flowInputs.volume <= 0 || flowInputs.coefficientC <= 0 || flowInputs.exponentN <= 0) {
        return { valid: false };
    }

    const fanDpAbs = Math.abs(fanPressurePa);
    const buildingDpAbs = Math.abs(buildingPressurePa);
    if (fanDpAbs <= 0 || buildingDpAbs < MIN_BUILDING_PRESSURE_FOR_ACH) {
        return { valid: false };
    }

    const airDensity = getAirDensity(flowInputs.altitude, buildingTemperatureC);
    const densityFactor = Math.sqrt(SEA_LEVEL_AIR_DENSITY / airDensity);

    const qFanM3h = flowInputs.coefficientC
        * Math.pow(fanDpAbs, flowInputs.exponentN)
        * flowInputs.apertureScale
        * densityFactor;

    const qRefM3h = qFanM3h * Math.pow(targetPressure / buildingDpAbs, 0.65);
    const achInstant = qRefM3h / flowInputs.volume;

    if (!Number.isFinite(achInstant) || achInstant < 0) {
        return { valid: false };
    }

    const now = Date.now();
    state.achHistory.push({ timestamp: now, value: achInstant });
    while (state.achHistory.length > 0 && now - state.achHistory[0].timestamp > ACH_WINDOW_MS) {
        state.achHistory.shift();
    }

    const sum = state.achHistory.reduce((acc, item) => acc + item.value, 0);
    const average = sum / Math.max(state.achHistory.length, 1);

    return {
        valid: true,
        instant: achInstant,
        average,
    };
}

function updateHeroMetrics(fanPressurePa, buildingPressurePa, buildingTemperatureC) {
    if (refs.heroPressure) {
        refs.heroPressure.textContent = formatMetric(buildingPressurePa, 'Pa', 1);
    }

    const ach = computeAchForMode(fanPressurePa, buildingPressurePa, buildingTemperatureC);
    if (!refs.heroAch || !refs.heroAchInstant) {
        return;
    }

    if (!ach.valid) {
        refs.heroAch.textContent = '--';
        refs.heroAchInstant.textContent = '--';
        return;
    }

    refs.heroAch.textContent = ach.average.toFixed(2);
    refs.heroAchInstant.textContent = ach.instant.toFixed(2);
}

function resetAchHistory() {
    state.achHistory.length = 0;
}

function extractTelemetry(data) {
    const rawFanPressure = toFiniteNumber(
        data.dp1_pressure ?? data.dp_pressure ?? data.dpPressure,
        Number.NaN,
    );
    const rawFanTemperature = toFiniteNumber(
        data.dp1_temperature ?? data.dp_temperature ?? data.dpTemperature,
        Number.NaN,
    );

    const rawEnvelopePressure = toFiniteNumber(data.dp2_pressure, Number.NaN);
    const rawEnvelopeTemperature = toFiniteNumber(data.dp2_temperature, Number.NaN);

    const fanOffset = Number.isFinite(state.baseline.fanOffset) ? state.baseline.fanOffset : 0;
    const envelopeOffset = Number.isFinite(state.baseline.envelopeOffset) ? state.baseline.envelopeOffset : 0;

    const correctedFanPressure = Number.isFinite(rawFanPressure) ? rawFanPressure - fanOffset : Number.NaN;
    const correctedEnvelopePressure = Number.isFinite(rawEnvelopePressure)
        ? rawEnvelopePressure - envelopeOffset
        : Number.NaN;

    return {
        pwm: Number.isFinite(data.pwm) ? Number(data.pwm) : Number.NaN,
        relayEnabled: parseBooleanFlag(data.relay),
        autoHoldEnabled: parseBooleanFlag(data.led),
        frequencyHz: toFiniteNumber(data.frequency, Number.NaN),

        fanPressurePa: correctedFanPressure,
        fanTemperatureC: rawFanTemperature,
        fanOk: parseBooleanFlag(data.dp1_ok ?? data.dp_ok),

        envelopePressurePa: correctedEnvelopePressure,
        envelopeTemperatureC: rawEnvelopeTemperature,
        envelopeOk: parseBooleanFlag(data.dp2_ok),

        rawFanPressure,
        rawEnvelopePressure,
    };
}

async function postJson(url, payload) {
    const response = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload),
    });

    if (!response.ok) {
        throw new Error(`HTTP ${response.status}`);
    }

    const text = await response.text();
    if (!text) {
        return null;
    }

    try {
        return JSON.parse(text);
    } catch (_error) {
        return null;
    }
}

async function sendUpdate(endpoint, value) {
    const route = API_ROUTES[endpoint] || `/api/${endpoint}`;
    await postJson(route, { value });
}

async function sendPwmUpdateCoalesced(value) {
    const clampedValue = clamp(Math.round(toFiniteNumber(value, 0)), 0, 100);

    if (pwmRequestInFlight) {
        pendingPwmValue = clampedValue;
        return;
    }

    pwmRequestInFlight = true;
    try {
        await sendUpdate('pwm', clampedValue);
    } finally {
        pwmRequestInFlight = false;
        if (pendingPwmValue !== null && pendingPwmValue !== clampedValue) {
            const nextValue = pendingPwmValue;
            pendingPwmValue = null;
            await sendPwmUpdateCoalesced(nextValue);
        } else {
            pendingPwmValue = null;
        }
    }
}

function requestControlPwm(value) {
    const clampedValue = clamp(Math.round(toFiniteNumber(value, 0)), 0, 100);
    if (state.control.lastSetPwm === clampedValue) {
        return;
    }

    state.control.lastSetPwm = clampedValue;
    updateManualSpeedDisplay(clampedValue);
    sendPwmUpdateCoalesced(clampedValue)
        .then(() => {
            setConnectionState('connected', 'PWM command applied');
        })
        .catch((error) => {
            console.error('Error sending PWM:', error);
            setConnectionState('error', 'Failed to send PWM');
        });
}

function stopControlSession(stopFan) {
    state.control.active = false;
    state.control.stage = 'idle';
    state.control.lastSetPwm = null;
    state.control.pid.integral = 0;
    state.control.pid.lastError = 0;
    state.control.pid.lastTime = Date.now();

    if (stopFan) {
        requestControlPwm(0);
        sendUpdate('relay', 0).catch((error) => {
            console.error('Could not disable relay:', error);
        });
        if (refs.fanPowerToggle) {
            refs.fanPowerToggle.checked = false;
        }
    }
}

function startControlSession() {
    const targetPa = getSelectedTargetPressure();

    state.control.active = true;
    state.control.stage = 'ramp';
    state.control.targetPa = targetPa;
    state.control.lastSetPwm = null;
    state.control.pid.integral = 0;
    state.control.pid.lastError = targetPa;
    state.control.pid.lastTime = Date.now();

    setSessionState(`${state.selectedMode} test active · ramping to ${targetPa} Pa`);

    if (refs.autoHoldToggle && !refs.autoHoldToggle.checked) {
        refs.autoHoldToggle.checked = true;
        sendUpdate('led', 1).catch((error) => {
            console.error('Could not enable auto hold:', error);
        });
    }

    if (refs.fanPowerToggle && !refs.fanPowerToggle.checked) {
        refs.fanPowerToggle.checked = true;
    }
    sendUpdate('relay', 1).catch((error) => {
        console.error('Could not enable relay:', error);
    });

    requestControlPwm(100);
}

function processControlLoop(buildingPressurePa, sensorOk) {
    if (!state.control.active || !Number.isFinite(buildingPressurePa) || !sensorOk) {
        return;
    }

    const measuredPa = Math.abs(buildingPressurePa);
    const targetPa = state.control.targetPa;

    if (state.control.stage === 'ramp') {
        if (measuredPa >= targetPa * CONTROL_RAMP_THRESHOLD_RATIO) {
            state.control.stage = 'control';
            state.control.pid.integral = 0;
            state.control.pid.lastError = targetPa - measuredPa;
            state.control.pid.lastTime = Date.now();
            setSessionState(`${state.selectedMode} test active · fine control`);
        } else {
            requestControlPwm(100);
        }
        return;
    }

    const now = Date.now();
    let dt = (now - state.control.pid.lastTime) / 1000;
    if (dt <= 0) {
        dt = 0.1;
    }
    state.control.pid.lastTime = now;

    const error = targetPa - measuredPa;
    if (Math.abs(error) <= CONTROL_PID.deadbandPa) {
        return;
    }

    state.control.pid.integral = clamp(state.control.pid.integral + error * dt, -80, 80);
    const derivative = (error - state.control.pid.lastError) / dt;
    state.control.pid.lastError = error;

    const correction = CONTROL_PID.kp * error
        + CONTROL_PID.ki * state.control.pid.integral
        + CONTROL_PID.kd * derivative;

    const rawPwm = clamp(CONTROL_PID.basePwm + correction, 0, 100);
    const previous = state.control.lastSetPwm;
    const smoothedPwm = Number.isFinite(previous)
        ? (previous * 0.7) + (rawPwm * 0.3)
        : rawPwm;

    requestControlPwm(smoothedPwm);
}

function captureZeroCalibration() {
    if (!latestTelemetry) {
        setSessionState('no telemetry available for calibration');
        return;
    }

    const telemetry = extractTelemetry(latestTelemetry);

    if (!Number.isFinite(telemetry.rawFanPressure) || !Number.isFinite(telemetry.rawEnvelopePressure)) {
        setSessionState('no valid readings for calibration');
        return;
    }

    state.baseline.fanOffset = telemetry.rawFanPressure;
    state.baseline.envelopeOffset = telemetry.rawEnvelopePressure;
    state.baseline.capturedAt = new Date();
    updateBaselineBadge();
    resetAchHistory();

    setSessionState('zero calibration applied');

    handleTelemetryPayload(latestTelemetry, 'calibration applied');
}

function applyAnemometerCalibration() {
    const speedMs = toFiniteNumber(refs.anemoSpeed?.value, Number.NaN);
    if (!Number.isFinite(speedMs) || speedMs <= 0 || !latestTelemetry) {
        return;
    }

    const telemetry = extractTelemetry(latestTelemetry);
    if (!Number.isFinite(telemetry.fanPressurePa) || telemetry.fanPressurePa <= 0) {
        setSessionState('no fan ΔP available to recalibrate C');
        return;
    }

    const flowInputs = getFlowCalibrationInputs();
    if (flowInputs.exponentN <= 0 || flowInputs.apertureScale <= 0) {
        return;
    }

    const qMeasuredM3h = speedMs * flowInputs.apertureArea * 3600;
    const airDensity = getAirDensity(flowInputs.altitude, telemetry.envelopeTemperatureC);
    const densityFactor = Math.sqrt(SEA_LEVEL_AIR_DENSITY / airDensity);

    const suggestedC = qMeasuredM3h
        / (Math.pow(Math.abs(telemetry.fanPressurePa), flowInputs.exponentN)
            * flowInputs.apertureScale
            * densityFactor);

    if (!Number.isFinite(suggestedC) || suggestedC <= 0) {
        return;
    }

    if (refs.fanCoefC) {
        refs.fanCoefC.value = suggestedC.toFixed(1);
    }

    resetAchHistory();
    setSessionState(`C recalibrated: ${suggestedC.toFixed(1)}`);

    handleTelemetryPayload(latestTelemetry, 'C coefficient recalibrated');
}

function handleTelemetryPayload(data, sourceDetail) {
    if (!data) {
        return;
    }

    latestTelemetry = data;
    const telemetry = extractTelemetry(data);

    if (telemetry.fanOk) {
        state.sensorHealth.fanEverValid = true;
    }
    if (telemetry.envelopeOk) {
        state.sensorHealth.envelopeEverValid = true;
    }

    if (Number.isFinite(telemetry.pwm)) {
        updateManualSpeedDisplay(telemetry.pwm);
        state.lastManualSpeed = telemetry.pwm;
    }

    if (refs.fanPowerToggle) {
        refs.fanPowerToggle.checked = telemetry.relayEnabled || (Number.isFinite(telemetry.pwm) && telemetry.pwm > 0);
    }

    if (refs.autoHoldToggle) {
        refs.autoHoldToggle.checked = telemetry.autoHoldEnabled;
    }

    if (refs.dpVentPressureValue) {
        refs.dpVentPressureValue.textContent = formatMetric(telemetry.fanPressurePa, 'Pa', 1);
    }
    if (refs.dpVentTemperatureValue) {
        refs.dpVentTemperatureValue.textContent = formatMetric(telemetry.fanTemperatureC, '°C', 1);
    }
    updateStatusPill(
        refs.dpVentStatus,
        telemetry.fanOk,
        telemetry.fanOk || state.sensorHealth.fanEverValid,
    );

    if (refs.dpBuildingPressureValue) {
        refs.dpBuildingPressureValue.textContent = formatMetric(telemetry.envelopePressurePa, 'Pa', 1);
    }
    if (refs.dpBuildingTemperatureValue) {
        refs.dpBuildingTemperatureValue.textContent = formatMetric(telemetry.envelopeTemperatureC, '°C', 1);
    }
    updateStatusPill(
        refs.dpBuildingStatus,
        telemetry.envelopeOk,
        telemetry.envelopeOk || state.sensorHealth.envelopeEverValid,
    );

    if (refs.fanFrequencyValue) {
        refs.fanFrequencyValue.textContent = formatMetric(telemetry.frequencyHz, 'Hz', 1);
    }

    updateHeroMetrics(
        telemetry.fanPressurePa,
        telemetry.envelopePressurePa,
        telemetry.envelopeTemperatureC,
    );

    processControlLoop(telemetry.envelopePressurePa, telemetry.envelopeOk);

    setConnectionState('connected', sourceDetail || 'real-time telemetry');
    stampLastUpdate();
}

function closeEventStream() {
    if (sseReconnectTimer) {
        clearTimeout(sseReconnectTimer);
        sseReconnectTimer = null;
    }

    if (eventSource) {
        eventSource.close();
        eventSource = null;
    }
}

function scheduleSseReconnect() {
    if (sseReconnectTimer) {
        return;
    }

    sseReconnectTimer = setTimeout(() => {
        sseReconnectTimer = null;
        sseRetryDelayMs = Math.min(sseRetryDelayMs * 2, SSE_RETRY_MAX_MS);
        connectEventStream();
    }, sseRetryDelayMs);
}

function connectEventStream() {
    closeEventStream();

    try {
        eventSource = new EventSource(API_ROUTES.events);
    } catch (error) {
        console.error('Could not open SSE:', error);
        setConnectionState('error', 'error opening SSE, retrying');
        scheduleSseReconnect();
        return;
    }

    eventSource.onopen = () => {
        sseRetryDelayMs = SSE_RETRY_BASE_MS;
        hasReceivedSseEvent = false;
        setConnectionState('connected', 'SSE connected');
    };

    eventSource.onmessage = (event) => {
        try {
            const payload = JSON.parse(event.data);
            handleTelemetryPayload(payload, 'SSE active');
            hasReceivedSseEvent = true;
        } catch (error) {
            console.error('Invalid SSE payload:', error);
        }
    };

    eventSource.onerror = (error) => {
        const transientStartupError = !hasReceivedSseEvent && sseRetryDelayMs === SSE_RETRY_BASE_MS;
        if (!transientStartupError) {
            console.error('SSE error:', error);
        }
        closeEventStream();
        setConnectionState(
            'error',
            hasReceivedSseEvent
                ? 'SSE interrupted, reconnecting'
                : 'SSE unavailable, retrying',
        );
        hasReceivedSseEvent = false;
        scheduleSseReconnect();
    };
}

function updateOtaStatusView(payload) {
    if (!payload || typeof payload !== 'object') {
        return;
    }

    if (typeof payload.firmware_version === 'string') {
        if (refs.firmwareVersionLabel) {
            refs.firmwareVersionLabel.textContent = payload.firmware_version;
        }
        if (refs.firmwareVersionTop) {
            refs.firmwareVersionTop.textContent = payload.firmware_version;
        }
        if (refs.firmwareVersionFooter) {
            refs.firmwareVersionFooter.textContent = `Firmware: ${payload.firmware_version}`;
        }
        if (refs.otaVersionInput && !state.ota.versionTouched && refs.otaVersionInput.value.trim() === '') {
            refs.otaVersionInput.value = payload.firmware_version;
        }
    }

    const stateLabel = String(payload.state || 'idle');
    const progress = clamp(toFiniteNumber(payload.progress_percent, 0), 0, 100);
    const stagedVersion = typeof payload.staged_version === 'string' ? payload.staged_version : '';
    const lastError = typeof payload.last_error === 'string' ? payload.last_error : '';

    if (stateLabel === 'receiving') {
        setOtaProgress(progress, `Uploading firmware ${Math.round(progress)}%`);
    } else if (stateLabel === 'ready') {
        setOtaProgress(100, `Firmware staged (${stagedVersion || 'no label'})`);
    } else if (stateLabel === 'applying') {
        setOtaProgress(progress, 'Applying firmware and rebooting...');
    } else if (stateLabel === 'error') {
        setOtaProgress(progress, `OTA error: ${lastError || 'unknown'}`);
    } else {
        setOtaProgress(0, 'OTA idle');
    }
}

async function refreshOtaStatus() {
    try {
        const response = await fetch(API_ROUTES.otaStatus, { cache: 'no-store' });
        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }
        const payload = await response.json();
        updateOtaStatusView(payload);
    } catch (error) {
        const isNetworkIssue = error instanceof TypeError;
        if (!isNetworkIssue) {
            console.error('Error in /api/ota/status:', error);
        }
    }
}

async function uploadFirmwareOta() {
    if (state.ota.uploadInProgress || state.ota.applyInProgress) {
        return;
    }

    const file = refs.otaFileInput?.files?.[0] || null;
    if (!file) {
        setSessionState('select a .bin before uploading OTA');
        return;
    }

    state.ota.uploadInProgress = true;
    setOtaControlsBusy(true);

    try {
        setSessionState('starting OTA transfer');
        setOtaProgress(0, 'Reading firmware...');

        const arrayBuffer = await file.arrayBuffer();
        const firmwareBytes = new Uint8Array(arrayBuffer);
        const crc32 = computeCrc32(firmwareBytes);
        const activeVersion =
            refs.firmwareVersionLabel?.textContent?.trim()
            || refs.firmwareVersionTop?.textContent?.trim()
            || '0.0.0-dev';
        const versionLabelRaw = refs.otaVersionInput?.value || activeVersion;
        const versionLabel = versionLabelRaw.trim() || activeVersion;

        await postJson(API_ROUTES.otaBegin, {
            size: firmwareBytes.length,
            crc32,
            version: versionLabel,
        });

        let offset = 0;
        while (offset < firmwareBytes.length) {
            const chunk = firmwareBytes.subarray(offset, offset + OTA_CHUNK_SIZE_BYTES);
            const chunkB64 = bytesToBase64(chunk);
            await postJson(API_ROUTES.otaChunk, {
                offset,
                data: chunkB64,
            });

            offset += chunk.length;
            const progress = (offset * 100) / firmwareBytes.length;
            setOtaProgress(progress, `Uploading firmware ${Math.round(progress)}%`);
        }

        await postJson(API_ROUTES.otaFinish, {});
        setSessionState('OTA uploaded. Press apply to reboot.');
        setOtaProgress(100, `OTA firmware uploaded (${versionLabel})`);
        await refreshOtaStatus();
    } catch (error) {
        console.error('Error during OTA upload:', error);
        setSessionState('OTA upload failed');
        setOtaProgress(0, 'OTA upload error');
        await refreshOtaStatus();
    } finally {
        state.ota.uploadInProgress = false;
        setOtaControlsBusy(false);
    }
}

async function applyFirmwareOta() {
    if (state.ota.uploadInProgress || state.ota.applyInProgress) {
        return;
    }

    state.ota.applyInProgress = true;
    setOtaControlsBusy(true);
    setSessionState('applying OTA firmware');

    try {
        setOtaProgress(100, 'Applying firmware and rebooting...');
        await postJson(API_ROUTES.otaApply, {});
        setConnectionState('error', 'device rebooting due to OTA');
    } catch (error) {
        console.error('Error during OTA apply:', error);
        setSessionState('OTA apply failed');
    } finally {
        state.ota.applyInProgress = false;
        setOtaControlsBusy(false);
        setTimeout(() => {
            refreshOtaStatus();
        }, 5000);
    }
}

function debounce(fn, delayMs) {
    let timerId = null;
    return (...args) => {
        if (timerId) {
            clearTimeout(timerId);
        }
        timerId = setTimeout(() => fn(...args), delayMs);
    };
}

function bindEvents() {
    refs.modeN50Btn?.addEventListener('click', () => {
        state.selectedMode = 'n50';
        state.control.targetPa = getSelectedTargetPressure();
        updateModeButtons();
        saveUiSettings();
        resetAchHistory();
        if (latestTelemetry) {
            handleTelemetryPayload(latestTelemetry, 'n50 mode selected');
        }
    });

    refs.modeN75Btn?.addEventListener('click', () => {
        state.selectedMode = 'n75';
        state.control.targetPa = getSelectedTargetPressure();
        updateModeButtons();
        saveUiSettings();
        resetAchHistory();
        if (latestTelemetry) {
            handleTelemetryPayload(latestTelemetry, 'n75 mode selected');
        }
    });

    refs.settingsToggleBtn?.addEventListener('click', () => {
        setSettingsPanelOpen(true);
    });

    refs.settingsCloseBtn?.addEventListener('click', () => {
        setSettingsPanelOpen(false);
    });

    refs.settingsBackdrop?.addEventListener('click', () => {
        setSettingsPanelOpen(false);
    });

    refs.calibrateBtn?.addEventListener('click', async () => {
        stopControlSession(false);
        requestControlPwm(0);

        try {
            await postJson(API_ROUTES.calibrate, {});
            captureZeroCalibration();
        } catch (error) {
            console.error('Could not calibrate on firmware:', error);
            setConnectionState('error', 'failed to calibrate ADP910');
        }
    });

    refs.startTestBtn?.addEventListener('click', () => {
        startControlSession();
    });

    refs.stopTestBtn?.addEventListener('click', () => {
        stopControlSession(true);
        setSessionState('test stopped');
    });

    refs.fanPowerToggle?.addEventListener('change', async (event) => {
        const shouldEnable = event.target.checked;
        if (!shouldEnable) {
            stopControlSession(true);
            setSessionState('fan stopped');
            return;
        }

        try {
            await sendUpdate('relay', 1);
            const target = state.lastManualSpeed > 0 ? state.lastManualSpeed : 40;
            requestControlPwm(target);
            setSessionState('fan enabled');
        } catch (error) {
            console.error('Could not enable relay:', error);
            event.target.checked = false;
            setConnectionState('error', 'could not enable fan');
        }
    });

    refs.autoHoldToggle?.addEventListener('change', async (event) => {
        const enabled = event.target.checked;
        try {
            await sendUpdate('led', enabled ? 1 : 0);
            setSessionState(enabled ? 'auto hold enabled' : 'auto hold disabled');
        } catch (error) {
            console.error('Could not update auto hold:', error);
            event.target.checked = !enabled;
            setConnectionState('error', 'failed to change auto hold');
        }
    });

    const sendManualPwm = debounce((value) => {
        requestControlPwm(value);
    }, 140);

    refs.fanSpeed?.addEventListener('input', (event) => {
        const value = clamp(toFiniteNumber(event.target.value, 0), 0, 100);
        stopControlSession(false);
        state.lastManualSpeed = value;
        updateManualSpeedDisplay(value);
        sendManualPwm(value);
    });

    refs.quickButtons.forEach((button) => {
        button.addEventListener('click', () => {
            const value = clamp(toFiniteNumber(button.dataset.speed, 0), 0, 100);
            stopControlSession(false);
            state.lastManualSpeed = value;
            requestControlPwm(value);
            setSessionState(`manual power set to ${value}%`);
        });
    });

    refs.applyAnemoBtn?.addEventListener('click', () => {
        applyAnemometerCalibration();
    });

    refs.otaUploadBtn?.addEventListener('click', () => {
        uploadFirmwareOta();
    });

    refs.otaApplyBtn?.addEventListener('click', () => {
        applyFirmwareOta();
    });

    refs.otaFileInput?.addEventListener('change', () => {
        const file = refs.otaFileInput?.files?.[0] || null;
        if (!file) {
            setOtaProgress(0, 'OTA idle');
            return;
        }
        setOtaProgress(0, `File ready: ${file.name}`);
    });

    refs.otaVersionInput?.addEventListener('input', () => {
        state.ota.versionTouched = true;
    });

    [
        refs.buildingVolume,
        refs.fanApertureCm,
        refs.altitude,
        refs.fanCoefC,
        refs.fanCoefN,
    ].forEach((input) => {
        input?.addEventListener('change', () => {
            saveUiSettings();
            updateApertureAreaLabel();
            resetAchHistory();
            if (latestTelemetry) {
                handleTelemetryPayload(latestTelemetry, 'parameters updated');
            }
        });
        input?.addEventListener('input', saveUiSettings);
    });
}

function bootstrap() {
    loadUiSettings();
    updateModeButtons();
    updateApertureAreaLabel();
    updateBaselineBadge();
    updateManualSpeedDisplay(0);
    setOtaProgress(0, 'OTA idle');
    setSessionState('idle');
    setConnectionState('idle', 'waiting for telemetry');

    bindEvents();
    connectEventStream();
    refreshOtaStatus();
    otaStatusRefreshTimer = setInterval(() => {
        refreshOtaStatus();
    }, OTA_STATUS_REFRESH_INTERVAL_MS);
}

window.addEventListener('load', bootstrap);

window.addEventListener('beforeunload', () => {
    closeEventStream();
    if (otaStatusRefreshTimer) {
        clearInterval(otaStatusRefreshTimer);
        otaStatusRefreshTimer = null;
    }
});

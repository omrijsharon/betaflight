const state = {
  session: null,
  system: null,
  config: null,
  pending: null,
  latestSessionId: null,
  downloaded: null,
  sessions: [],
  localSessions: [],
  selectedAnalysisSessionIds: new Set(),
  recordingActive: false,
  plotData: null,
  selectedPlotFields: new Set(),
  plotSessionId: null,
  codex: {
    messages: [],
    itemIndex: new Map(),
    rawEvents: [],
    status: 'idle',
    tokenUsage: null,
    rateLimits: null
  }
};

const DEFAULT_PLOT_FIELDS = [
  'status.altitude_cm',
  'status.baro_altitude_cm',
  'status.velocity_cms',
  'status.position_rate_cms',
  'status.accel_world_z_cms2',
  'status.accel_bias_cms2',
  'status.innovation_cm'
];

const PLOT_COLORS = ['#24589f', '#c62828', '#087f5b', '#9c27b0', '#ef6c00', '#00838f', '#5d4037', '#607d8b'];

function el(id) {
  return document.getElementById(id);
}

async function api(path, options = {}) {
  const res = await fetch(path, options);
  const text = await res.text();
  const data = text ? JSON.parse(text) : {};
  if (!res.ok) throw new Error(data.error || text);
  return data;
}

async function post(path, body) {
  return api(path, {
    method: 'POST',
    headers: {'content-type': 'application/json'},
    body: JSON.stringify(body || {})
  });
}

function showTab(name) {
  document.querySelectorAll('.tab').forEach((btn) => btn.classList.toggle('active', btn.dataset.tab === name));
  document.querySelectorAll('.panel').forEach((panel) => panel.classList.toggle('active', panel.id === name));
}

function statusCard(label, value) {
  return `<div class="status-card"><strong>${label}</strong><span>${value}</span></div>`;
}

function boolText(value) {
  return value ? 'OK' : 'No';
}

function escapeHtml(value) {
  return String(value ?? '').replace(/[&<>"']/g, (char) => ({
    '&': '&amp;',
    '<': '&lt;',
    '>': '&gt;',
    '"': '&quot;',
    "'": '&#39;'
  }[char]));
}

function sessionIdOf(session) {
  return String(session.session_id || session.sessionId || '');
}

function durationSecondsOf(session) {
  const value = session.duration_s ?? session.durationS ?? session.duration_seconds;
  const number = Number(value);
  return Number.isFinite(number) && number >= 0 ? number : null;
}

function formatDuration(session) {
  const durationS = durationSecondsOf(session);
  if (durationS === null) {
    return 'Length: unknown';
  }
  const total = Math.round(durationS);
  const seconds = total % 60;
  const minutes = Math.floor(total / 60) % 60;
  const hours = Math.floor(total / 3600);
  if (hours > 0) {
    return `Length: ${hours}:${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;
  }
  return `Length: ${minutes}:${String(seconds).padStart(2, '0')}`;
}

function plotLabel(field) {
  return field.label + (field.unit ? ` (${field.unit})` : '');
}

function renderCodexChat() {
  const chat = el('codexChat');
  if (!chat) return;
  const tokenUsage = state.codex.tokenUsage?.total;
  const rateLimit = state.codex.rateLimits?.primary;
  const meta = [
    `Status: ${state.codex.status}`,
    tokenUsage ? `Tokens: ${tokenUsage.totalTokens || 0}` : null,
    rateLimit ? `Rate: ${rateLimit.usedPercent || 0}%` : null
  ].filter(Boolean).join(' | ');
  const messages = state.codex.messages.length
    ? state.codex.messages.map((message) => `
      <div class="chat-message ${message.role}">
        <div class="chat-meta">${escapeHtml(message.label)}</div>
        <div class="chat-text">${escapeHtml(message.text || '...')}</div>
      </div>
    `).join('')
    : '<p class="muted">No Codex messages yet</p>';
  chat.innerHTML = `<div class="chat-status">${escapeHtml(meta)}</div>${messages}`;
  chat.scrollTop = chat.scrollHeight;
}

function appendCodexMessage(role, label, text) {
  state.codex.messages.push({role, label, text});
  renderCodexChat();
}

function resetCodexChat(sessionIds) {
  state.codex.messages = [];
  state.codex.itemIndex = new Map();
  state.codex.rawEvents = [];
  state.codex.status = 'running';
  state.codex.tokenUsage = null;
  state.codex.rateLimits = null;
  appendCodexMessage('user', 'Analysis request', `Analyze ${sessionIds.join(', ')}`);
  el('codexRawView').textContent = '';
}

function updateCodexAgentMessage(itemId, text, completed) {
  if (!itemId) return;
  let index = state.codex.itemIndex.get(itemId);
  if (index === undefined) {
    index = state.codex.messages.length;
    state.codex.itemIndex.set(itemId, index);
    state.codex.messages.push({role: 'assistant', label: 'Codex', text: ''});
  }
  state.codex.messages[index].text = text;
  if (completed) {
    state.codex.messages[index].completed = true;
  }
  renderCodexChat();
}

function handleCodexNotification(message) {
  state.codex.rawEvents.push(message);
  if (state.codex.rawEvents.length > 200) {
    state.codex.rawEvents.shift();
  }
  const rawView = el('codexRawView');
  if (rawView) {
    rawView.textContent = state.codex.rawEvents.map((event) => JSON.stringify(event)).join('\n');
    rawView.scrollTop = rawView.scrollHeight;
  }

  const params = message.params || {};
  if (message.method === 'item/agentMessage/delta') {
    const itemId = params.itemId;
    const index = state.codex.itemIndex.get(itemId);
    const existing = index === undefined ? '' : state.codex.messages[index].text;
    updateCodexAgentMessage(itemId, existing + (params.delta || ''), false);
    return;
  }
  if (message.method === 'item/completed' && params.item?.type === 'agentMessage') {
    updateCodexAgentMessage(params.item.id, params.item.text || '', true);
    return;
  }
  if (message.method === 'thread/status/changed') {
    state.codex.status = params.status?.type || 'unknown';
    renderCodexChat();
    return;
  }
  if (message.method === 'thread/tokenUsage/updated') {
    state.codex.tokenUsage = params.tokenUsage;
    renderCodexChat();
    return;
  }
  if (message.method === 'account/rateLimits/updated') {
    state.codex.rateLimits = params.rateLimits;
    renderCodexChat();
    return;
  }
  if (message.method === 'turn/completed') {
    state.codex.status = 'completed';
    const durationMs = params.turn?.durationMs;
    appendCodexMessage('system', 'Turn completed', durationMs ? `${(durationMs / 1000).toFixed(1)}s` : 'Done');
    return;
  }
  if (message.method === 'turn/failed' || message.method === 'turn/error') {
    state.codex.status = 'failed';
    appendCodexMessage('system error', 'Codex error', JSON.stringify(params));
  }
}

function renderStatus() {
  const system = state.system || {};
  const fc = system.fc && system.fc.status ? system.fc.status : system.fc;
  const recording = system.pi?.health?.recording || system.pi?.health?.recording || {};
  state.recordingActive = Boolean(recording && recording.active);
  updateRecordingButton(state.recordingActive);
  el('statusGrid').innerHTML = [
    statusCard('Pi', boolText(system.pi && system.pi.ok)),
    statusCard('FC', boolText(system.fc && system.fc.ok !== false)),
    statusCard('Armed', boolText(fc && fc.armed)),
    statusCard('Recording', boolText(recording && recording.active)),
    statusCard('Codex', boolText(system.codex && system.codex.ok)),
    statusCard('Pending', boolText(state.pending && state.pending.exists))
  ].join('');
}

function updateRecordingButton(active) {
  const button = el('recordingToggle');
  if (!button) return;
  button.classList.toggle('recording', active);
  button.title = active ? 'Stop recording' : 'Start recording';
  button.setAttribute('aria-label', active ? 'Stop recording' : 'Start recording');
  button.innerHTML = active
    ? '<span class="record-glyph square" aria-hidden="true"></span><span class="sr-only">Stop recording</span>'
    : '<span class="record-glyph circle" aria-hidden="true"></span><span class="sr-only">Start recording</span>';
}

function renderPending() {
  const pending = state.pending?.recommendation;
  if (!pending || !Array.isArray(pending.parameters) || pending.parameters.length === 0) {
    el('pendingView').innerHTML = '<p>No pending suggestion</p>';
    return;
  }
  el('pendingView').innerHTML = pending.parameters.map((param) => `
    <div class="diff-item">
      <strong>${param.name}</strong>
      <div>${param.old_value} -> ${param.new_value} ${param.unit || ''}</div>
      <p>${param.reason || ''}</p>
    </div>
  `).join('');
}

function renderSessions() {
  const view = el('sessionsView');
  if (!view) return;
  if (!state.sessions.length) {
    view.innerHTML = '<p>No sessions found on Pi</p>';
    return;
  }
  view.innerHTML = state.sessions.map((session) => {
    const id = sessionIdOf(session);
    const created = session.created_utc || '';
    const duration = formatDuration(session);
    return `
      <div class="session-item">
        <div class="session-main">
          <strong>${escapeHtml(id)}</strong>
          <div class="session-meta">${escapeHtml(created)} | ${escapeHtml(duration)}</div>
        </div>
        <div class="session-actions">
          <button data-session-id="${escapeHtml(id)}" class="download-one">Download</button>
          <button data-session-id="${escapeHtml(id)}" class="delete-one danger">Delete Pi</button>
        </div>
      </div>
    `;
  }).join('');
  document.querySelectorAll('.download-one').forEach((button) => {
    button.onclick = async () => {
      el('sessionIdInput').value = button.dataset.sessionId;
      await downloadSession();
    };
  });
  document.querySelectorAll('.delete-one').forEach((button) => {
    button.onclick = async () => {
      await deleteSession(button.dataset.sessionId);
    };
  });
}

function renderLocalSessions() {
  const localView = el('localSessionsView');
  const analysisView = el('analysisSessionsView');
  if (!localView || !analysisView) return;

  const body = state.localSessions.length ? state.localSessions.map((session) => {
    const id = sessionIdOf(session);
    const downloaded = session.downloaded_utc || '';
    const bytes = Number(session.telemetry_bytes || 0);
    const checked = state.selectedAnalysisSessionIds.has(id) ? 'checked' : '';
    const rec = session.has_recommendation ? `Recommendation: ${session.recommendation_parameters || 0}` : 'No recommendation';
    const duration = formatDuration(session);
    return `
      <div class="session-item">
        <label class="session-check">
          <input type="checkbox" class="analysis-select" data-session-id="${escapeHtml(id)}" ${checked}>
          <span>
            <strong>${escapeHtml(id)}</strong>
            <span class="session-meta">${escapeHtml(downloaded)} | ${escapeHtml(duration)} | ${bytes} telemetry bytes | ${escapeHtml(rec)}</span>
          </span>
        </label>
        <div class="session-actions">
          <button data-session-id="${escapeHtml(id)}" class="plot-one">Plot</button>
          <button data-session-id="${escapeHtml(id)}" class="delete-local-one danger">Delete Local</button>
        </div>
      </div>
    `;
  }).join('') : '<p>No local downloads found</p>';

  localView.innerHTML = body;
  analysisView.innerHTML = state.localSessions.length ? body : '<p>Download at least one Pi session before analysis</p>';

  document.querySelectorAll('.analysis-select').forEach((checkbox) => {
    checkbox.onchange = () => {
      if (checkbox.checked) {
        state.selectedAnalysisSessionIds.add(checkbox.dataset.sessionId);
      } else {
        state.selectedAnalysisSessionIds.delete(checkbox.dataset.sessionId);
      }
      syncAnalysisCheckboxes();
    };
  });
  document.querySelectorAll('.delete-local-one').forEach((button) => {
    button.onclick = async () => {
      await deleteLocalSession(button.dataset.sessionId);
    };
  });
  document.querySelectorAll('.plot-one').forEach((button) => {
    button.onclick = async () => {
      await plotSession(button.dataset.sessionId);
    };
  });
  syncAnalysisCheckboxes();
  renderPlotSessionOptions();
}

function syncAnalysisCheckboxes() {
  document.querySelectorAll('.analysis-select').forEach((checkbox) => {
    checkbox.checked = state.selectedAnalysisSessionIds.has(checkbox.dataset.sessionId);
  });
}

function renderPlotSessionOptions() {
  const select = el('plotSessionSelect');
  if (!select) return;
  const current = state.plotSessionId || select.value;
  select.innerHTML = state.localSessions.map((session) => {
    const id = sessionIdOf(session);
    return `<option value="${escapeHtml(id)}">${escapeHtml(id)} - ${escapeHtml(formatDuration(session))}</option>`;
  }).join('');
  if (current && state.localSessions.some((session) => sessionIdOf(session) === current)) {
    select.value = current;
  } else if (state.localSessions.length) {
    select.value = sessionIdOf(state.localSessions[0]);
  }
}

function defaultPlotFields(fields) {
  const names = fields.map((field) => field.name);
  const preferred = DEFAULT_PLOT_FIELDS.filter((field) => names.includes(field));
  return preferred.length ? preferred : names.slice(0, 4);
}

function renderPlotFields() {
  const container = el('plotFields');
  if (!container) return;
  const fields = state.plotData?.fields || [];
  if (!fields.length) {
    container.innerHTML = '<p>No numeric fields found</p>';
    return;
  }
  container.innerHTML = fields.map((field, index) => {
    const checked = state.selectedPlotFields.has(field.name) ? 'checked' : '';
    const color = PLOT_COLORS[index % PLOT_COLORS.length];
    return `
      <label class="field-chip">
        <input type="checkbox" class="plot-field" data-field="${escapeHtml(field.name)}" ${checked}>
        <span class="legend-swatch" style="background:${color}"></span>
        <span>${escapeHtml(plotLabel(field))}</span>
      </label>
    `;
  }).join('');
  document.querySelectorAll('.plot-field').forEach((checkbox) => {
    checkbox.onchange = () => {
      if (checkbox.checked) {
        state.selectedPlotFields.add(checkbox.dataset.field);
      } else {
        state.selectedPlotFields.delete(checkbox.dataset.field);
      }
      drawPlot();
    };
  });
}

function drawPlot() {
  const canvas = el('plotCanvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const rect = canvas.getBoundingClientRect();
  const cssWidth = Math.max(320, Math.floor(rect.width || 1100));
  const cssHeight = Math.max(260, Math.floor(rect.height || 520));
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.floor(cssWidth * dpr);
  canvas.height = Math.floor(cssHeight * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, cssWidth, cssHeight);
  ctx.fillStyle = '#ffffff';
  ctx.fillRect(0, 0, cssWidth, cssHeight);

  const samples = state.plotData?.samples || [];
  const fields = (state.plotData?.fields || []).filter((field) => state.selectedPlotFields.has(field.name));
  const margin = { left: 58, right: 20, top: 20, bottom: 42 };
  const width = cssWidth - margin.left - margin.right;
  const height = cssHeight - margin.top - margin.bottom;

  ctx.strokeStyle = '#d7dde7';
  ctx.lineWidth = 1;
  for (let i = 0; i <= 5; i += 1) {
    const x = margin.left + (width * i / 5);
    const y = margin.top + (height * i / 5);
    ctx.beginPath();
    ctx.moveTo(x, margin.top);
    ctx.lineTo(x, margin.top + height);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(margin.left, y);
    ctx.lineTo(margin.left + width, y);
    ctx.stroke();
  }

  ctx.strokeStyle = '#5a6678';
  ctx.beginPath();
  ctx.moveTo(margin.left, margin.top);
  ctx.lineTo(margin.left, margin.top + height);
  ctx.lineTo(margin.left + width, margin.top + height);
  ctx.stroke();

  if (!samples.length || !fields.length) {
    ctx.fillStyle = '#5a6678';
    ctx.font = '14px sans-serif';
    ctx.fillText('No plot data selected', margin.left + 12, margin.top + 28);
    return;
  }

  const maxT = Math.max(...samples.map((sample) => Number(sample.t_s) || 0), 1);
  let minY = Infinity;
  let maxY = -Infinity;
  fields.forEach((field) => {
    samples.forEach((sample) => {
      const value = Number(sample[field.name]);
      if (Number.isFinite(value)) {
        minY = Math.min(minY, value);
        maxY = Math.max(maxY, value);
      }
    });
  });
  if (!Number.isFinite(minY) || !Number.isFinite(maxY)) {
    minY = 0;
    maxY = 1;
  }
  if (minY === maxY) {
    minY -= 1;
    maxY += 1;
  }
  const pad = (maxY - minY) * 0.08;
  minY -= pad;
  maxY += pad;

  ctx.fillStyle = '#151b25';
  ctx.font = '12px sans-serif';
  ctx.fillText(`${maxY.toFixed(1)}`, 8, margin.top + 4);
  ctx.fillText(`${minY.toFixed(1)}`, 8, margin.top + height);
  ctx.fillText('0s', margin.left, cssHeight - 14);
  ctx.fillText(`${maxT.toFixed(1)}s`, margin.left + width - 42, cssHeight - 14);

  fields.forEach((field, index) => {
    ctx.strokeStyle = PLOT_COLORS[index % PLOT_COLORS.length];
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    let started = false;
    samples.forEach((sample) => {
      const value = Number(sample[field.name]);
      if (!Number.isFinite(value)) {
        started = false;
        return;
      }
      const x = margin.left + (Number(sample.t_s) || 0) / maxT * width;
      const y = margin.top + (1 - ((value - minY) / (maxY - minY))) * height;
      if (!started) {
        ctx.moveTo(x, y);
        started = true;
      } else {
        ctx.lineTo(x, y);
      }
    });
    ctx.stroke();
  });
}

async function plotSession(sessionId) {
  if (!sessionId) return;
  state.plotSessionId = sessionId;
  showTab('plot');
  renderPlotSessionOptions();
  el('plotSessionSelect').value = sessionId;
  await loadPlotData();
}

async function loadPlotData() {
  const sessionId = el('plotSessionSelect').value || state.plotSessionId;
  if (!sessionId) return;
  state.plotSessionId = sessionId;
  state.plotData = await api(`/api/local-sessions/${encodeURIComponent(sessionId)}/plot-data`);
  const fieldNames = new Set((state.plotData.fields || []).map((field) => field.name));
  const selected = Array.from(state.selectedPlotFields).filter((field) => fieldNames.has(field));
  state.selectedPlotFields = new Set(selected.length ? selected : defaultPlotFields(state.plotData.fields || []));
  el('plotSummary').innerHTML = [
    `<strong>${escapeHtml(state.plotData.session_id)}</strong>`,
    `${Number(state.plotData.sample_count || 0)} samples`,
    `${formatDuration({ duration_s: state.plotData.duration_s })}`
  ].join(' | ');
  el('plotView').textContent = JSON.stringify({
    session_id: state.plotData.session_id,
    sample_count: state.plotData.sample_count,
    duration_s: state.plotData.duration_s,
    fields: state.plotData.fields
  }, null, 2);
  renderPlotFields();
  drawPlot();
}

async function refresh() {
  try {
    state.session = await api('/api/session');
    el('sessionLine').textContent = state.session.authenticated ? 'Paired session active' : 'Open /operator to pair this browser';
    if (!state.session.authenticated) return;
    state.system = await api('/api/system/status');
    state.pending = await api('/api/recommendations/pending');
    renderStatus();
    renderPending();
    const rec = state.system.pi?.health?.recording;
    if (rec?.session_id) {
      state.latestSessionId = rec.session_id;
      el('sessionIdInput').value = rec.session_id;
    }
    if (rec?.active) {
      showTab('recording');
    }
  } catch (error) {
    el('sessionLine').textContent = String(error);
  }
}

async function loadConfig() {
  state.config = await api('/api/altitude/config');
  el('configView').textContent = JSON.stringify(state.config, null, 2);
}

async function applyRecommendation() {
  const pending = state.pending?.recommendation;
  if (!pending || !Array.isArray(pending.parameters)) return;
  const patch = {};
  pending.parameters.forEach((param) => {
    patch[param.name] = param.new_value;
  });
  state.config = await post('/api/altitude/config/apply', patch);
  await post('/api/recommendations/pending/consume');
  await loadConfig();
  await refresh();
}

async function startRecording() {
  el('recordingView').textContent = JSON.stringify(await post('/api/recording/start'), null, 2);
  state.recordingActive = true;
  updateRecordingButton(true);
  showTab('recording');
  await refresh();
}

async function stopRecording() {
  const result = await post('/api/recording/stop');
  el('recordingView').textContent = JSON.stringify(result, null, 2);
  if (result.session_id) {
    state.latestSessionId = result.session_id;
    el('sessionIdInput').value = result.session_id;
    showTab('download');
  }
  state.recordingActive = false;
  updateRecordingButton(false);
  await refresh();
  await refreshSessions();
  await refreshLocalSessions();
}

async function toggleRecording() {
  if (state.recordingActive) {
    await stopRecording();
  } else {
    await startRecording();
  }
}

async function downloadSession() {
  const sessionId = el('sessionIdInput').value || state.latestSessionId;
  if (!sessionId) return;
  const result = await post(`/api/sessions/${encodeURIComponent(sessionId)}/download`);
  state.downloaded = result;
  state.selectedAnalysisSessionIds.add(result.sessionId || sessionId);
  el('downloadView').textContent = JSON.stringify(result, null, 2);
  showTab('download');
  await refreshLocalSessions();
}

async function refreshSessions() {
  state.sessions = await api('/api/sessions');
  renderSessions();
}

async function refreshAllSessions() {
  await refreshSessions();
  await refreshLocalSessions();
}

async function refreshLocalSessions() {
  state.localSessions = await api('/api/local-sessions');
  const validIds = new Set(state.localSessions.map(sessionIdOf));
  state.selectedAnalysisSessionIds.forEach((id) => {
    if (!validIds.has(id)) {
      state.selectedAnalysisSessionIds.delete(id);
    }
  });
  renderLocalSessions();
}

async function downloadAllSessions() {
  const result = await post('/api/sessions/download-all');
  (result.sessions || []).forEach((session) => {
    const id = session.sessionId || session.session_id;
    if (id) state.selectedAnalysisSessionIds.add(id);
  });
  el('downloadView').textContent = JSON.stringify(result, null, 2);
  showTab('download');
  await refreshSessions();
  await refreshLocalSessions();
}

async function deleteSession(sessionId) {
  if (!sessionId) return;
  await api(`/api/sessions/${encodeURIComponent(sessionId)}`, {method: 'DELETE'});
  state.sessions = state.sessions.filter((session) => (session.session_id || session.sessionId) !== sessionId);
  renderSessions();
  el('downloadView').textContent = JSON.stringify({deleted: true, session_id: sessionId}, null, 2);
}

async function deleteLocalSession(sessionId) {
  if (!sessionId) return;
  await api(`/api/local-sessions/${encodeURIComponent(sessionId)}`, {method: 'DELETE'});
  state.selectedAnalysisSessionIds.delete(sessionId);
  state.localSessions = state.localSessions.filter((session) => sessionIdOf(session) !== sessionId);
  renderLocalSessions();
  el('downloadView').textContent = JSON.stringify({deleted_local: true, session_id: sessionId}, null, 2);
}

async function runAnalysis() {
  const sessionIds = Array.from(state.selectedAnalysisSessionIds).filter(Boolean);
  if (!sessionIds.length) {
    el('recommendationView').textContent = JSON.stringify({error: 'Select one or more local sessions to analyze'}, null, 2);
    showTab('analysis');
    return;
  }
  resetCodexChat(sessionIds);
  el('recommendationView').textContent = JSON.stringify({running: true, session_ids: sessionIds}, null, 2);
  showTab('analysis');
  const result = await post('/api/analysis/analyze', {session_ids: sessionIds});
  el('recommendationView').textContent = JSON.stringify(result.recommendation, null, 2);
  await refreshLocalSessions();
}

async function sendRecommendation() {
  el('recommendationView').textContent = JSON.stringify(await post('/api/recommendations/current/send-to-pi'), null, 2);
  await refresh();
}

document.querySelectorAll('.tab').forEach((btn) => btn.onclick = () => showTab(btn.dataset.tab));
el('loadConfig').onclick = loadConfig;
el('applyRecommendation').onclick = applyRecommendation;
el('saveConfig').onclick = async () => { el('configView').textContent = JSON.stringify(await post('/api/altitude/config/save'), null, 2); };
el('rebootFc').onclick = async () => { el('configView').textContent = JSON.stringify(await post('/api/altitude/fc/reboot'), null, 2); };
el('recordingToggle').onclick = toggleRecording;
el('downloadSession').onclick = downloadSession;
el('refreshSessions').onclick = refreshAllSessions;
el('downloadAllSessions').onclick = downloadAllSessions;
el('refreshAnalysisSessions').onclick = refreshLocalSessions;
el('refreshPlotSessions').onclick = refreshLocalSessions;
el('loadPlot').onclick = loadPlotData;
el('runAnalysis').onclick = runAnalysis;
el('sendRecommendation').onclick = sendRecommendation;
window.addEventListener('resize', () => drawPlot());

const ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
ws.onmessage = (event) => {
  const msg = JSON.parse(event.data);
  if (msg.type === 'codexNotification') {
    handleCodexNotification(msg.message);
  }
};

setInterval(refresh, 2000);
refresh().then(async () => {
  await loadConfig();
  await refreshAllSessions();
}).catch(() => undefined);

import http, { IncomingMessage, ServerResponse } from "node:http";
import { createHash } from "node:crypto";
import fs from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { WebSocketServer, WebSocket } from "ws";
import qrcode from "qrcode";

import { buildAnalysisPrompt, buildMultiSessionAnalysisPrompt, emptyRecommendation, runLocalAnalysis } from "./analysis.js";
import { CodexAppServerClient } from "./codexClient.js";
import { loadConfig, TuningConfig } from "./config.js";
import { clearCookie, cookie, parseCookies, readJson, sameOriginAllowed, sendJson, sendText } from "./httpUtil.js";
import { PiClient } from "./piClient.js";
import { PairingStore, readSignedSession, SessionRecord, SessionStore, signSession } from "./state.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const publicDir = path.resolve(__dirname, "..", "..", "public");
const OPERATOR_COOKIE = "althold_operator";
const SESSION_COOKIE = "althold_session";

interface PersistentState {
  threadId?: string;
  latestSessionId?: string;
  latestRecommendationPath?: string;
  latestAnalysisSessionIds?: string[];
}

interface ServerContext {
  config: TuningConfig;
  pairings: PairingStore;
  sessions: SessionStore;
  pi: PiClient;
  codex: CodexAppServerClient;
  state: PersistentState;
  clients: Set<WebSocket>;
}

async function readPersistentState(config: TuningConfig): Promise<PersistentState> {
  try {
    return JSON.parse(await fs.readFile(config.statePath, "utf8")) as PersistentState;
  } catch {
    return {};
  }
}

async function writePersistentState(config: TuningConfig, state: PersistentState): Promise<void> {
  await fs.mkdir(path.dirname(config.statePath), { recursive: true });
  await fs.writeFile(config.statePath, JSON.stringify(state, null, 2) + "\n", "utf8");
}

function requireSafeSessionId(sessionId: string): string {
  if (!/^[A-Za-z0-9_.-]+$/.test(sessionId) || sessionId === "." || sessionId === "..") {
    throw new Error("invalid session id");
  }
  return sessionId;
}

function containedPath(baseDir: string, targetPath: string): string {
  const base = path.resolve(baseDir);
  const target = path.resolve(targetPath);
  if (target !== base && !target.startsWith(base + path.sep)) {
    throw new Error("path escapes local log directory");
  }
  return target;
}

function utcStamp(): string {
  return new Date().toISOString().replace(/[-:.]/g, "").replace("T", "T").slice(0, 15) + "Z";
}

async function readJsonFile(filePath: string): Promise<Record<string, unknown> | null> {
  try {
    return JSON.parse(await fs.readFile(filePath, "utf8")) as Record<string, unknown>;
  } catch {
    return null;
  }
}

function isValidRecommendation(value: unknown): value is Record<string, unknown> {
  if (!value || typeof value !== "object" || Array.isArray(value)) {
    return false;
  }
  const record = value as Record<string, unknown>;
  return record.schema === "althold-recommendation-v1"
    && typeof record.session_id === "string"
    && Array.isArray(record.parameters);
}

async function readValidRecommendation(filePath: string): Promise<Record<string, unknown> | null> {
  const value = await readJsonFile(filePath);
  return isValidRecommendation(value) ? value : null;
}

function numberValue(value: unknown): number | null {
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function extractHostTimeNs(line: string): bigint | null {
  const match = /"host_time_ns"\s*:\s*(\d+)/.exec(line);
  return match ? BigInt(match[1]) : null;
}

function diffSeconds(startNs: bigint, endNs: bigint): number | null {
  if (endNs < startNs) {
    return null;
  }
  return Number(endNs - startNs) / 1_000_000_000;
}

async function durationFromEvents(filePath: string): Promise<number | null> {
  try {
    const text = await fs.readFile(filePath, "utf8");
    let startNs: bigint | null = null;
    let endNs: bigint | null = null;
    for (const line of text.split(/\r?\n/)) {
      if (!line) {
        continue;
      }
      const hostTimeNs = extractHostTimeNs(line);
      if (hostTimeNs === null) {
        continue;
      }
      const event = /"event"\s*:\s*"([^"]+)"/.exec(line)?.[1];
      if (event === "start" && startNs === null) {
        startNs = hostTimeNs;
      }
      if (event === "stop" || event === "stop_error" || event === "disarmed") {
        endNs = hostTimeNs;
      }
    }
    return startNs !== null && endNs !== null ? diffSeconds(startNs, endNs) : null;
  } catch {
    return null;
  }
}

async function durationFromTelemetry(filePath: string): Promise<number | null> {
  try {
    const text = await fs.readFile(filePath, "utf8");
    let startNs: bigint | null = null;
    let endNs: bigint | null = null;
    for (const line of text.split(/\r?\n/)) {
      if (!line) {
        continue;
      }
      const hostTimeNs = extractHostTimeNs(line);
      if (hostTimeNs === null) {
        continue;
      }
      if (startNs === null) {
        startNs = hostTimeNs;
      }
      endNs = hostTimeNs;
    }
    return startNs !== null && endNs !== null ? diffSeconds(startNs, endNs) : null;
  } catch {
    return null;
  }
}

async function sessionDurationS(sessionDir: string, manifest: Record<string, unknown> | null): Promise<number | null> {
  const manifestDuration = numberValue(manifest?.duration_s ?? manifest?.duration_seconds);
  if (manifestDuration !== null) {
    return manifestDuration;
  }
  return await durationFromEvents(path.join(sessionDir, "events.jsonl"))
    ?? await durationFromTelemetry(path.join(sessionDir, "telemetry.jsonl"));
}

function flattenNumericFields(prefix: string, value: unknown, output: Record<string, number>): void {
  if (typeof value === "number" && Number.isFinite(value)) {
    output[prefix] = value;
    return;
  }
  if (!value || typeof value !== "object" || Array.isArray(value)) {
    return;
  }
  for (const [key, child] of Object.entries(value as Record<string, unknown>)) {
    const nextPrefix = prefix ? `${prefix}.${key}` : key;
    flattenNumericFields(nextPrefix, child, output);
  }
}

function displayFieldName(field: string): string {
  return field.startsWith("status.") ? field.slice("status.".length) : field;
}

function unitForField(field: string): string {
  const name = displayFieldName(field);
  if (name.endsWith("_cm") || name === "altitude_cm" || name === "baro_altitude_cm" || name === "innovation_cm" || name === "gate_cm" || name === "baro_offset_cm") {
    return "cm";
  }
  if (name.endsWith("_cms") || name === "velocity_cms" || name === "position_rate_cms") {
    return "cm/s";
  }
  if (name.endsWith("_cms2") || name === "accel_world_z_cms2" || name === "accel_bias_cms2") {
    return "cm/s^2";
  }
  if (name.endsWith("_ms") || name === "baro_age_ms" || name === "configured_delay_ms") {
    return "ms";
  }
  if (name.endsWith("_us") || name === "timestamp_us") {
    return "us";
  }
  if (name.endsWith("_cm2") || name === "r_eff_cm2" || name === "s_cm2") {
    return "cm^2";
  }
  return "";
}

async function readPlotData(localLogDir: string, sessionId: string): Promise<Record<string, unknown>> {
  const safeId = requireSafeSessionId(sessionId);
  const sessionDir = containedPath(localLogDir, path.join(localLogDir, safeId));
  const telemetryPath = containedPath(sessionDir, path.join(sessionDir, "telemetry.jsonl"));
  const manifest = await readJsonFile(path.join(sessionDir, "manifest.json"));
  const lines = await fs.readFile(telemetryPath, "utf8").then((text) => text.split(/\r?\n/)).catch(() => []);
  const fields = new Set<string>();
  const samples: Array<Record<string, number | string | null>> = [];
  let startNs: bigint | null = null;
  let lastNs: bigint | null = null;

  for (const line of lines) {
    if (!line) {
      continue;
    }
    const hostTimeNs = extractHostTimeNs(line);
    if (hostTimeNs === null) {
      continue;
    }
    if (startNs === null) {
      startNs = hostTimeNs;
    }
    lastNs = hostTimeNs;
    let parsed: Record<string, unknown>;
    try {
      parsed = JSON.parse(line) as Record<string, unknown>;
    } catch {
      continue;
    }
    const numeric: Record<string, number> = {};
    flattenNumericFields("", parsed, numeric);
    delete numeric.host_time_ns;
    for (const field of Object.keys(numeric)) {
      fields.add(field);
    }
    samples.push({
      t_s: diffSeconds(startNs, hostTimeNs) ?? 0,
      host_time_ns: hostTimeNs.toString(),
      ...numeric
    });
  }

  const sortedFields = Array.from(fields).sort((a, b) => displayFieldName(a).localeCompare(displayFieldName(b)));
  return {
    session_id: safeId,
    duration_s: startNs !== null && lastNs !== null ? diffSeconds(startNs, lastNs) : await sessionDurationS(sessionDir, manifest),
    sample_count: samples.length,
    fields: sortedFields.map((field) => ({ name: field, label: displayFieldName(field), unit: unitForField(field) })),
    samples
  };
}

async function listLocalSessions(localLogDir: string): Promise<Array<Record<string, unknown>>> {
  await fs.mkdir(localLogDir, { recursive: true });
  const entries = await fs.readdir(localLogDir, { withFileTypes: true });
  const sessions: Array<Record<string, unknown>> = [];
  for (const entry of entries) {
    if (!entry.isDirectory() || entry.name === "analysis_runs") {
      continue;
    }
    const sessionId = entry.name;
    if (!/^[A-Za-z0-9_.-]+$/.test(sessionId)) {
      continue;
    }
    const sessionDir = containedPath(localLogDir, path.join(localLogDir, sessionId));
    const manifest = await readJsonFile(path.join(sessionDir, "manifest.json"));
    const recommendation = await readJsonFile(path.join(sessionDir, "recommendation.json"));
    const stat = await fs.stat(sessionDir);
    const telemetryPath = path.join(sessionDir, "telemetry.jsonl");
    const eventsPath = path.join(sessionDir, "events.jsonl");
    const telemetryBytes = await fs.stat(telemetryPath).then((value) => value.size).catch(() => 0);
    const eventsBytes = await fs.stat(eventsPath).then((value) => value.size).catch(() => 0);
    const durationS = await sessionDurationS(sessionDir, manifest);
    sessions.push({
      session_id: sessionId,
      path: sessionDir,
      downloaded_utc: stat.mtime.toISOString(),
      created_utc: manifest?.created_utc ?? manifest?.start_utc ?? null,
      duration_s: durationS === null ? null : Math.round(durationS * 1000) / 1000,
      telemetry_bytes: telemetryBytes,
      events_bytes: eventsBytes,
      has_manifest: Boolean(manifest),
      has_recommendation: Boolean(recommendation),
      recommendation_parameters: Array.isArray(recommendation?.parameters) ? recommendation.parameters.length : 0
    });
  }
  sessions.sort((a, b) => String(b.downloaded_utc).localeCompare(String(a.downloaded_utc)));
  return sessions;
}

async function deleteLocalSession(localLogDir: string, sessionId: string): Promise<Record<string, unknown>> {
  const safeId = requireSafeSessionId(sessionId);
  const sessionDir = containedPath(localLogDir, path.join(localLogDir, safeId));
  const zipPath = containedPath(localLogDir, path.join(localLogDir, `${safeId}.zip`));
  await fs.rm(sessionDir, { recursive: true, force: true });
  await fs.rm(zipPath, { force: true });
  return { deleted: true, session_id: safeId, path: sessionDir };
}

function isOperator(req: IncomingMessage, config: TuningConfig): boolean {
  const cookies = parseCookies(req);
  return cookies[OPERATOR_COOKIE] === operatorCookieValue(config.operatorSecret) && Boolean(config.operatorSecret);
}

function operatorCookieValue(secret: string): string {
  return createHash("sha256").update(`operator:${secret}`).digest("hex");
}

function sessionFor(req: IncomingMessage, ctx: ServerContext): SessionRecord | null {
  return readSignedSession(parseCookies(req)[SESSION_COOKIE], ctx.config.sessionSecret, ctx.sessions);
}

function requireSession(req: IncomingMessage, res: ServerResponse, ctx: ServerContext): SessionRecord | null {
  const session = sessionFor(req, ctx);
  if (!session) {
    sendJson(res, 401, { error: "authentication required" });
    return null;
  }
  return session;
}

function requireOperator(req: IncomingMessage, res: ServerResponse, ctx: ServerContext): boolean {
  if (isOperator(req, ctx.config)) {
    return true;
  }
  sendJson(res, 401, { error: "operator login required" });
  return false;
}

function requireSameOrigin(req: IncomingMessage, res: ServerResponse, ctx: ServerContext): boolean {
  if (sameOriginAllowed(req, ctx.config.publicBaseUrl)) {
    return true;
  }
  sendJson(res, 403, { error: "same-origin check failed" });
  return false;
}

function broadcast(ctx: ServerContext, event: Record<string, unknown>): void {
  const raw = JSON.stringify(event);
  for (const client of ctx.clients) {
    if (client.readyState === WebSocket.OPEN) {
      client.send(raw);
    }
  }
}

async function serveStatic(req: IncomingMessage, res: ServerResponse, pathname: string): Promise<boolean> {
  const fileName = pathname === "/" ? "index.html" : pathname.slice(1);
  const filePath = path.resolve(publicDir, fileName);
  if (!filePath.startsWith(publicDir)) {
    return false;
  }
  try {
    const body = await fs.readFile(filePath);
    const ext = path.extname(filePath);
    const contentType = ext === ".html" ? "text/html; charset=utf-8"
      : ext === ".js" ? "application/javascript; charset=utf-8"
        : ext === ".css" ? "text/css; charset=utf-8"
          : "application/octet-stream";
    res.writeHead(200, { "content-type": contentType, "content-length": body.length });
    res.end(body);
    return true;
  } catch {
    return false;
  }
}

async function systemStatus(ctx: ServerContext): Promise<Record<string, unknown>> {
  const checks: Record<string, unknown> = {
    backend: { ok: true, port: ctx.config.port, publicBaseUrl: ctx.config.publicBaseUrl },
    codex: { ok: ctx.codex.connected(), url: ctx.config.codexAppServerUrl },
    pi: { ok: false },
    fc: { ok: false },
    pendingRecommendation: null
  };
  try {
    checks.pi = { ok: true, health: await ctx.pi.health() };
  } catch (error) {
    checks.pi = { ok: false, error: error instanceof Error ? error.message : String(error) };
  }
  try {
    checks.fc = await ctx.pi.fcStatus();
  } catch (error) {
    checks.fc = { ok: false, error: error instanceof Error ? error.message : String(error) };
  }
  try {
    checks.pendingRecommendation = await ctx.pi.pendingRecommendation();
  } catch {
    checks.pendingRecommendation = null;
  }
  return checks;
}

async function selectRuntime(ctx: ServerContext): Promise<{ model: string; effort: string; serviceTier?: string }> {
  await ctx.codex.connect();
  const models = await ctx.codex.listModels();
  const model = models.find((entry) => {
    const id = String(entry.id ?? entry.model ?? entry.slug ?? "");
    const label = String(entry.displayName ?? entry.name ?? "");
    return id.includes(ctx.config.modelName) || label.toLowerCase().includes(ctx.config.modelName.toLowerCase());
  });
  if (!model) {
    throw new Error(`Codex model ${ctx.config.modelName} is not available`);
  }
  const modelId = String(model.id ?? model.model ?? model.slug);
  const legacyLevels = Array.isArray(model.supportedReasoningLevels) ? model.supportedReasoningLevels as Record<string, unknown>[] : [];
  const effortLevels = Array.isArray(model.supportedReasoningEfforts) ? model.supportedReasoningEfforts as Record<string, unknown>[] : [];
  const levels = [...legacyLevels, ...effortLevels];
  const requestedReasoning = ctx.config.reasoningLabel.toLowerCase();
  const reasoning = levels.find((level) => {
    const label = String(level.label ?? level.name ?? level.description ?? "");
    const value = String(level.value ?? level.reasoningEffort ?? "");
    return label.toLowerCase() === requestedReasoning
      || value.toLowerCase() === requestedReasoning
      || (requestedReasoning === "extra high" && value.toLowerCase() === "xhigh");
  });
  if (!reasoning) {
    throw new Error(`Reasoning option ${ctx.config.reasoningLabel} is not available for ${modelId}`);
  }
  const additionalSpeedTiers = Array.isArray(model.additionalSpeedTiers) ? model.additionalSpeedTiers : [];
  if (ctx.config.speed === "fast" && !additionalSpeedTiers.includes("fast")) {
    throw new Error(`Fast speed is not available for ${modelId}`);
  }
  return {
    model: modelId,
    effort: String(reasoning.value ?? reasoning.reasoningEffort ?? reasoning.label),
    serviceTier: ctx.config.speed === "fast" ? "fast" : undefined
  };
}

async function ensureThread(ctx: ServerContext): Promise<string> {
  const runtime = await selectRuntime(ctx);
  if (ctx.state.threadId) {
    await ctx.codex.resumeThread(ctx.state.threadId);
    return ctx.state.threadId;
  }
  const threadId = await ctx.codex.startThread({
    cwd: ctx.config.repoRoot,
    model: runtime.model,
    effort: runtime.effort,
    serviceTier: runtime.serviceTier,
    approvalPolicy: "never",
    sandboxPolicy: { type: "workspaceWrite", writableRoots: [ctx.config.repoRoot] },
    collaborationMode: "default"
  });
  ctx.state.threadId = threadId;
  await writePersistentState(ctx.config, ctx.state);
  return threadId;
}

function createCodexCompletionWaiter(ctx: ServerContext, recommendationPath?: string): {
  promise: Promise<"turn" | "recommendation">;
  cancel: () => void;
} {
  let settled = false;
  let timeout: NodeJS.Timeout | undefined;
  let interval: NodeJS.Timeout | undefined;
  let checkingRecommendation = false;
  let onNotification: ((msg: Record<string, unknown>) => void) | null = null;

  const cleanup = () => {
    if (onNotification) {
      ctx.codex.off("notification", onNotification);
      onNotification = null;
    }
    if (timeout) {
      clearTimeout(timeout);
      timeout = undefined;
    }
    if (interval) {
      clearInterval(interval);
      interval = undefined;
    }
  };

  let rejectPromise: (error: Error) => void = () => undefined;
  const promise = new Promise<"turn" | "recommendation">((resolve, reject) => {
    rejectPromise = reject;
    const resolveOnce = (reason: "turn" | "recommendation") => {
      if (settled) {
        return;
      }
      settled = true;
      cleanup();
      resolve(reason);
    };
    const rejectOnce = (error: Error) => {
      if (settled) {
        return;
      }
      settled = true;
      cleanup();
      reject(error);
    };

    const checkRecommendation = async () => {
      if (!recommendationPath || settled || checkingRecommendation) {
        return;
      }
      checkingRecommendation = true;
      try {
        if (await readValidRecommendation(recommendationPath)) {
          resolveOnce("recommendation");
        }
      } catch {
        // Ignore partial writes while Codex is still producing the file.
      } finally {
        checkingRecommendation = false;
      }
    };

    onNotification = (msg: Record<string, unknown>) => {
      broadcast(ctx, { type: "codexNotification", message: msg });
      if (msg.method === "turn/completed") {
        resolveOnce("turn");
      } else if (msg.method === "turn/failed" || msg.method === "turn/error") {
        rejectOnce(new Error("Codex turn failed"));
      }
    };
    ctx.codex.on("notification", onNotification);

    if (recommendationPath) {
      interval = setInterval(checkRecommendation, 1000);
      interval.unref();
      setTimeout(checkRecommendation, 250).unref();
    }

    timeout = setTimeout(() => {
      rejectOnce(new Error("Codex analysis timed out"));
    }, 10 * 60 * 1000);
    timeout.unref();
  });

  return {
    promise,
    cancel: () => {
      if (!settled) {
        settled = true;
        cleanup();
        rejectPromise(new Error("Codex wait cancelled"));
      }
    }
  };
}

async function runCodexPrompt(ctx: ServerContext, threadId: string, prompt: string, recommendationPath?: string): Promise<void> {
  const runtime = await selectRuntime(ctx);
  const waiter = createCodexCompletionWaiter(ctx, recommendationPath);
  try {
    await ctx.codex.startTurn({
      threadId,
      input: [{ type: "text", text: prompt }],
      model: runtime.model,
      effort: runtime.effort,
      serviceTier: runtime.serviceTier,
      approvalPolicy: "never",
      sandboxPolicy: { type: "workspaceWrite", writableRoots: [ctx.config.repoRoot] }
    });
    await waiter.promise;
  } catch (error) {
    waiter.cancel();
    try {
      await waiter.promise;
    } catch {
      // Preserve the original startTurn/wait failure.
    }
    throw error;
  }
}

async function readOrCreateRecommendation(recommendationPath: string, sessionId: string, threadId: string): Promise<Record<string, unknown>> {
  let recommendation = await readValidRecommendation(recommendationPath);
  if (!recommendation) {
    recommendation = emptyRecommendation(sessionId, threadId) as unknown as Record<string, unknown>;
  }
  if (!recommendation.source_thread_id) {
    recommendation.source_thread_id = threadId;
  }
  await fs.writeFile(recommendationPath, JSON.stringify(recommendation, null, 2) + "\n", "utf8");
  return recommendation;
}

async function analyzeSessions(ctx: ServerContext, sessionIds: string[]): Promise<Record<string, unknown>> {
  const safeSessionIds = Array.from(new Set(sessionIds.map(requireSafeSessionId)));
  if (safeSessionIds.length === 0) {
    throw new Error("at least one session id is required");
  }

  const currentConfig = await ctx.pi.getConfig();
  if (safeSessionIds.length === 1) {
    const sessionId = safeSessionIds[0];
    const sessionDir = containedPath(ctx.config.localLogDir, path.join(ctx.config.localLogDir, sessionId));
    const outputs = await runLocalAnalysis(ctx.config.repoRoot, sessionDir);
    const recommendationPath = containedPath(sessionDir, path.join(sessionDir, "recommendation.json"));
    await fs.rm(recommendationPath, { force: true });
    const threadId = await ensureThread(ctx);
    const prompt = buildAnalysisPrompt({
      sessionId,
      sessionDir,
      summaryPath: outputs.summaryPath,
      csvPath: outputs.csvPath,
      currentConfig,
      recommendationPath,
      sourceThreadId: threadId
    });
    await runCodexPrompt(ctx, threadId, prompt, recommendationPath);
    const recommendation = await readOrCreateRecommendation(recommendationPath, sessionId, threadId);
    ctx.state.latestSessionId = sessionId;
    ctx.state.latestRecommendationPath = recommendationPath;
    ctx.state.latestAnalysisSessionIds = [sessionId];
    await writePersistentState(ctx.config, ctx.state);
    return { threadId, sessionId, sessionIds: [sessionId], outputs, recommendation };
  }

  const runId = `analysis_${utcStamp()}`;
  const runDir = containedPath(ctx.config.localLogDir, path.join(ctx.config.localLogDir, "analysis_runs", runId));
  await fs.mkdir(runDir, { recursive: true });
  const sessions = [];
  for (const sessionId of safeSessionIds) {
    const sessionDir = containedPath(ctx.config.localLogDir, path.join(ctx.config.localLogDir, sessionId));
    const outputs = await runLocalAnalysis(ctx.config.repoRoot, sessionDir);
    sessions.push({ sessionId, sessionDir, ...outputs });
  }
  const recommendationPath = containedPath(runDir, path.join(runDir, "recommendation.json"));
  await fs.rm(recommendationPath, { force: true });
  const threadId = await ensureThread(ctx);
  const prompt = buildMultiSessionAnalysisPrompt({
    runId,
    runDir,
    sessions,
    currentConfig,
    recommendationPath,
    sourceThreadId: threadId
  });
  await runCodexPrompt(ctx, threadId, prompt, recommendationPath);
  const recommendation = {
    ...await readOrCreateRecommendation(recommendationPath, runId, threadId),
    source_sessions: safeSessionIds
  };
  await fs.writeFile(recommendationPath, JSON.stringify(recommendation, null, 2) + "\n", "utf8");
  ctx.state.latestSessionId = runId;
  ctx.state.latestRecommendationPath = recommendationPath;
  ctx.state.latestAnalysisSessionIds = safeSessionIds;
  await writePersistentState(ctx.config, ctx.state);
  return { threadId, sessionId: runId, sessionIds: safeSessionIds, analysisRunId: runId, runDir, sessions, recommendation };
}

async function handleApi(req: IncomingMessage, res: ServerResponse, ctx: ServerContext, url: URL): Promise<boolean> {
  const pathName = url.pathname;
  if (pathName === "/api/operator/login" && req.method === "POST") {
    if (!requireSameOrigin(req, res, ctx)) return true;
    const body = await readJson<{ secret?: string }>(req);
    if (body.secret && body.secret === ctx.config.operatorSecret) {
      sendJson(res, 200, { ok: true }, { "set-cookie": cookie(OPERATOR_COOKIE, operatorCookieValue(ctx.config.operatorSecret), ctx.config.publicBaseUrl) });
    } else {
      sendJson(res, 401, { error: "invalid operator secret" });
    }
    return true;
  }
  if (pathName === "/api/operator/logout" && req.method === "POST") {
    if (!requireSameOrigin(req, res, ctx)) return true;
    sendJson(res, 200, { ok: true }, { "set-cookie": clearCookie(OPERATOR_COOKIE, ctx.config.publicBaseUrl) });
    return true;
  }
  if (pathName === "/api/pairing/start" && req.method === "POST") {
    if (!requireSameOrigin(req, res, ctx) || !requireOperator(req, res, ctx)) return true;
    const pairing = ctx.pairings.create(ctx.config.pairingTtlMs);
    const pairUrl = `${ctx.config.publicBaseUrl}/pair?token=${encodeURIComponent(pairing.token)}`;
    const qrSvg = await qrcode.toString(pairUrl, { type: "svg", margin: 1 });
    sendJson(res, 200, { pairUrl, qrSvg, expiresAt: pairing.expiresAt });
    return true;
  }
  if (pathName === "/api/session" && req.method === "GET") {
    const session = sessionFor(req, ctx);
    sendJson(res, 200, { authenticated: Boolean(session), session });
    return true;
  }
  if (pathName === "/api/logout" && req.method === "POST") {
    if (!requireSameOrigin(req, res, ctx)) return true;
    const session = sessionFor(req, ctx);
    if (session) {
      ctx.sessions.revoke(session.sessionId);
    }
    sendJson(res, 200, { ok: true }, { "set-cookie": clearCookie(SESSION_COOKIE, ctx.config.publicBaseUrl) });
    return true;
  }
  if (pathName === "/api/session/revoke-current" && req.method === "POST") {
    if (!requireSameOrigin(req, res, ctx)) return true;
    const session = sessionFor(req, ctx);
    if (session) {
      ctx.sessions.revoke(session.sessionId);
    }
    sendJson(res, 200, { ok: true }, { "set-cookie": clearCookie(SESSION_COOKIE, ctx.config.publicBaseUrl) });
    return true;
  }
  if (pathName === "/api/operator/session/revoke" && req.method === "POST") {
    if (!requireSameOrigin(req, res, ctx) || !requireOperator(req, res, ctx)) return true;
    const body = await readJson<{ sessionId?: string }>(req);
    if (!body.sessionId) {
      sendJson(res, 400, { error: "sessionId is required" });
      return true;
    }
    sendJson(res, 200, { revoked: ctx.sessions.revoke(body.sessionId) });
    return true;
  }
  if (pathName === "/api/operator/session/revoke-all" && req.method === "POST") {
    if (!requireSameOrigin(req, res, ctx) || !requireOperator(req, res, ctx)) return true;
    ctx.sessions.revokeAll();
    sendJson(res, 200, { ok: true });
    return true;
  }
  if (pathName === "/api/system/status" && req.method === "GET") {
    if (!requireSession(req, res, ctx)) return true;
    sendJson(res, 200, await systemStatus(ctx));
    return true;
  }
  if (pathName === "/api/pi/status" && req.method === "GET") {
    if (!requireSession(req, res, ctx)) return true;
    sendJson(res, 200, await ctx.pi.health());
    return true;
  }
  if (pathName === "/api/fc/status" && req.method === "GET") {
    if (!requireSession(req, res, ctx)) return true;
    sendJson(res, 200, await ctx.pi.fcStatus());
    return true;
  }
  if (pathName === "/api/altitude/config" && req.method === "GET") {
    if (!requireSession(req, res, ctx)) return true;
    sendJson(res, 200, await ctx.pi.getConfig());
    return true;
  }
  if (pathName === "/api/altitude/config/apply" && req.method === "POST") {
    if (!requireSameOrigin(req, res, ctx) || !requireSession(req, res, ctx)) return true;
    sendJson(res, 200, await ctx.pi.applyConfig(await readJson<Record<string, unknown>>(req)));
    return true;
  }
  if (pathName === "/api/altitude/config/save" && req.method === "POST") {
    if (!requireSameOrigin(req, res, ctx) || !requireSession(req, res, ctx)) return true;
    sendJson(res, 200, await ctx.pi.saveConfig());
    return true;
  }
  if (pathName === "/api/altitude/fc/reboot" && req.method === "POST") {
    if (!requireSameOrigin(req, res, ctx) || !requireSession(req, res, ctx)) return true;
    sendJson(res, 200, await ctx.pi.rebootFc());
    return true;
  }
  if (pathName === "/api/recording/status" && req.method === "GET") {
    if (!requireSession(req, res, ctx)) return true;
    sendJson(res, 200, await ctx.pi.recordingStatus());
    return true;
  }
  if (pathName === "/api/recording/start" && req.method === "POST") {
    if (!requireSameOrigin(req, res, ctx) || !requireSession(req, res, ctx)) return true;
    sendJson(res, 200, await ctx.pi.startRecording());
    return true;
  }
  if (pathName === "/api/recording/stop" && req.method === "POST") {
    if (!requireSameOrigin(req, res, ctx) || !requireSession(req, res, ctx)) return true;
    sendJson(res, 200, await ctx.pi.stopRecording());
    return true;
  }
  if (pathName === "/api/sessions" && req.method === "GET") {
    if (!requireSession(req, res, ctx)) return true;
    sendJson(res, 200, await ctx.pi.listSessions());
    return true;
  }
  if (pathName === "/api/local-sessions" && req.method === "GET") {
    if (!requireSession(req, res, ctx)) return true;
    sendJson(res, 200, await listLocalSessions(ctx.config.localLogDir));
    return true;
  }
  if (pathName.startsWith("/api/local-sessions/") && req.method === "DELETE") {
    if (!requireSameOrigin(req, res, ctx) || !requireSession(req, res, ctx)) return true;
    const sessionId = decodeURIComponent(pathName.split("/")[3] ?? "");
    sendJson(res, 200, await deleteLocalSession(ctx.config.localLogDir, sessionId));
    return true;
  }
  if (pathName.startsWith("/api/local-sessions/") && pathName.endsWith("/plot-data") && req.method === "GET") {
    if (!requireSession(req, res, ctx)) return true;
    const sessionId = decodeURIComponent(pathName.split("/")[3] ?? "");
    sendJson(res, 200, await readPlotData(ctx.config.localLogDir, sessionId));
    return true;
  }
  if (pathName.startsWith("/api/sessions/") && pathName.endsWith("/download") && req.method === "POST") {
    if (!requireSameOrigin(req, res, ctx) || !requireSession(req, res, ctx)) return true;
    const sessionId = decodeURIComponent(pathName.split("/")[3] ?? "");
    const result = await ctx.pi.downloadSession(sessionId);
    ctx.state.latestSessionId = sessionId;
    await writePersistentState(ctx.config, ctx.state);
    sendJson(res, 200, result);
    return true;
  }
  if (pathName.startsWith("/api/sessions/") && req.method === "DELETE") {
    if (!requireSameOrigin(req, res, ctx) || !requireSession(req, res, ctx)) return true;
    const sessionId = decodeURIComponent(pathName.split("/")[3] ?? "");
    sendJson(res, 200, await ctx.pi.deleteSession(sessionId));
    return true;
  }
  if (pathName === "/api/sessions/download-all" && req.method === "POST") {
    if (!requireSameOrigin(req, res, ctx) || !requireSession(req, res, ctx)) return true;
    const results = await ctx.pi.downloadAllSessions();
    if (results.length > 0) {
      ctx.state.latestSessionId = results[0].sessionId;
      await writePersistentState(ctx.config, ctx.state);
    }
    sendJson(res, 200, { count: results.length, sessions: results });
    return true;
  }
  if (pathName.startsWith("/api/sessions/") && pathName.endsWith("/analyze") && req.method === "POST") {
    if (!requireSameOrigin(req, res, ctx) || !requireSession(req, res, ctx)) return true;
    const sessionId = decodeURIComponent(pathName.split("/")[3] ?? "");
    sendJson(res, 200, await analyzeSessions(ctx, [sessionId]));
    return true;
  }
  if (pathName === "/api/analysis/analyze" && req.method === "POST") {
    if (!requireSameOrigin(req, res, ctx) || !requireSession(req, res, ctx)) return true;
    const body = await readJson<{ session_ids?: string[]; sessionIds?: string[] }>(req);
    const sessionIds = body.session_ids ?? body.sessionIds ?? [];
    sendJson(res, 200, await analyzeSessions(ctx, sessionIds));
    return true;
  }
  if (pathName === "/api/recommendations/current" && req.method === "GET") {
    if (!requireSession(req, res, ctx)) return true;
    const recommendationPath = ctx.state.latestRecommendationPath
      ?? (ctx.state.latestSessionId ? path.join(ctx.config.localLogDir, ctx.state.latestSessionId, "recommendation.json") : null);
    if (!recommendationPath) {
      sendJson(res, 200, { exists: false, recommendation: null });
      return true;
    }
    try {
      sendJson(res, 200, {
        exists: true,
        sessionIds: ctx.state.latestAnalysisSessionIds ?? (ctx.state.latestSessionId ? [ctx.state.latestSessionId] : []),
        recommendation: JSON.parse(await fs.readFile(recommendationPath, "utf8"))
      });
    } catch {
      sendJson(res, 200, { exists: false, recommendation: null });
    }
    return true;
  }
  if (pathName === "/api/recommendations/pending" && req.method === "GET") {
    if (!requireSession(req, res, ctx)) return true;
    sendJson(res, 200, await ctx.pi.pendingRecommendation());
    return true;
  }
  if (pathName === "/api/recommendations/pending/consume" && req.method === "POST") {
    if (!requireSameOrigin(req, res, ctx) || !requireSession(req, res, ctx)) return true;
    sendJson(res, 200, await ctx.pi.consumeRecommendation());
    return true;
  }
  if (pathName === "/api/recommendations/current/send-to-pi" && req.method === "POST") {
    if (!requireSameOrigin(req, res, ctx) || !requireSession(req, res, ctx)) return true;
    const recommendationPath = ctx.state.latestRecommendationPath
      ?? (ctx.state.latestSessionId ? path.join(ctx.config.localLogDir, ctx.state.latestSessionId, "recommendation.json") : null);
    if (!recommendationPath) {
      sendJson(res, 404, { error: "no current recommendation" });
      return true;
    }
    const recommendation = JSON.parse(await fs.readFile(recommendationPath, "utf8"));
    sendJson(res, 200, await ctx.pi.sendRecommendation(recommendation));
    return true;
  }
  return false;
}

export async function createTuningServer(config: TuningConfig = loadConfig()): Promise<http.Server> {
  const ctx: ServerContext = {
    config,
    pairings: new PairingStore(),
    sessions: new SessionStore(),
    pi: new PiClient({ baseUrl: config.piBaseUrl, localLogDir: config.localLogDir }),
    codex: new CodexAppServerClient(config.codexAppServerUrl),
    state: await readPersistentState(config),
    clients: new Set()
  };
  ctx.codex.on("notification", (message) => broadcast(ctx, { type: "codexNotification", message }));
  setInterval(() => ctx.pairings.cleanup(), 30_000).unref();

  const server = http.createServer(async (req, res) => {
    try {
      const url = new URL(req.url ?? "/", config.publicBaseUrl);
      if (url.pathname === "/operator") {
        return void (await serveStatic(req, res, "/operator.html"));
      }
      if (url.pathname === "/pair" && req.method === "GET") {
        const token = url.searchParams.get("token") ?? "";
        const pairing = ctx.pairings.consume(token);
        if (!pairing) {
          return sendText(res, 400, "Invalid or expired pairing token");
        }
        const session = ctx.sessions.create(pairing.pairingId, config.sessionTtlMs, {
          ipAddress: req.socket.remoteAddress,
          userAgent: req.headers["user-agent"]
        });
        res.writeHead(302, {
          "location": "/",
          "set-cookie": cookie(SESSION_COOKIE, signSession(session.sessionId, config.sessionSecret), config.publicBaseUrl)
        });
        return res.end();
      }
      if (url.pathname.startsWith("/api/")) {
        const handled = await handleApi(req, res, ctx, url);
        if (!handled) {
          sendJson(res, 404, { error: "not found" });
        }
        return;
      }
      if (await serveStatic(req, res, url.pathname)) {
        return;
      }
      sendJson(res, 404, { error: "not found" });
    } catch (error) {
      sendJson(res, 500, { error: error instanceof Error ? error.message : String(error) });
    }
  });

  const wss = new WebSocketServer({ noServer: true });
  server.on("upgrade", (req, socket, head) => {
    const url = new URL(req.url ?? "/", config.publicBaseUrl);
    if (url.pathname !== "/ws" || !sessionFor(req, ctx)) {
      socket.destroy();
      return;
    }
    wss.handleUpgrade(req, socket, head, (ws) => {
      ctx.clients.add(ws);
      ws.on("close", () => ctx.clients.delete(ws));
      ws.send(JSON.stringify({ type: "session", authenticated: true }));
    });
  });

  server.on("close", () => {
    ctx.codex.close();
    for (const client of ctx.clients) {
      client.terminate();
    }
    wss.close();
  });

  return server;
}

export async function main(): Promise<void> {
  const config = loadConfig();
  const server = await createTuningServer(config);
  server.listen(config.port, config.host, () => {
    console.log(`Altitude tuning app listening on http://${config.host}:${config.port}`);
  });
}

if (process.argv[1] === fileURLToPath(import.meta.url)) {
  main().catch((error) => {
    console.error(error);
    process.exit(1);
  });
}

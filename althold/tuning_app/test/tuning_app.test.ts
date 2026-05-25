import test from "node:test";
import assert from "node:assert/strict";
import http from "node:http";
import fs from "node:fs/promises";
import os from "node:os";
import path from "node:path";
import AdmZip from "adm-zip";
import { WebSocketServer } from "ws";

import { createTuningServer } from "../src/server.js";
import { TuningConfig } from "../src/config.js";

function baseConfig(tmp: string, piBaseUrl: string, codexUrl = "ws://127.0.0.1:1"): TuningConfig {
  return {
    host: "127.0.0.1",
    port: 0,
    publicBaseUrl: "http://127.0.0.1:0",
    piBaseUrl,
    codexAppServerUrl: codexUrl,
    repoRoot: path.resolve("..", ".."),
    localLogDir: path.join(tmp, "logs"),
    sessionSecret: "test-session-secret",
    operatorSecret: "test-operator-secret",
    pairingTtlMs: 60_000,
    sessionTtlMs: 60_000,
    sessionRequestsPerMinute: 60,
    pairingRequestsPerHour: 20,
    modelName: "gpt-5.5",
    reasoningLabel: "Extra High",
    speed: "fast",
    statePath: path.join(tmp, "state.json")
  };
}

async function listen(server: http.Server): Promise<string> {
  await new Promise<void>((resolve) => server.listen(0, "127.0.0.1", resolve));
  const address = server.address();
  assert(address && typeof address === "object");
  return `http://127.0.0.1:${address.port}`;
}

async function close(server: http.Server): Promise<void> {
  await new Promise<void>((resolve, reject) => server.close((error) => error ? reject(error) : resolve()));
}

function cookieValue(header: string | null): string {
  assert(header);
  return header.split(";")[0];
}

async function authenticatedBase(config: TuningConfig): Promise<{ server: http.Server; base: string; sessionCookie: string }> {
  const server = await createTuningServer(config);
  const base = await listen(server);
  config.publicBaseUrl = base;

  const login = await fetch(`${base}/api/operator/login`, {
    method: "POST",
    headers: { "content-type": "application/json", "origin": base },
    body: JSON.stringify({ secret: config.operatorSecret })
  });
  assert.equal(login.status, 200);
  const operatorCookie = cookieValue(login.headers.get("set-cookie"));

  const pair = await fetch(`${base}/api/pairing/start`, {
    method: "POST",
    headers: { "origin": base, "cookie": operatorCookie }
  });
  assert.equal(pair.status, 200);
  const pairData = await pair.json() as { pairUrl: string };
  const paired = await fetch(pairData.pairUrl, { redirect: "manual" });
  assert.equal(paired.status, 302);
  return { server, base, sessionCookie: cookieValue(paired.headers.get("set-cookie")) };
}

function createFakePi(): Promise<{ server: http.Server; base: string; calls: string[] }> {
  const calls: string[] = [];
  const server = http.createServer((req, res) => {
    const url = new URL(req.url ?? "/", "http://127.0.0.1");
    calls.push(`${req.method} ${url.pathname}`);
    const send = (status: number, data: unknown) => {
      const body = JSON.stringify(data);
      res.writeHead(status, { "content-type": "application/json", "content-length": Buffer.byteLength(body) });
      res.end(body);
    };
    if (url.pathname === "/healthz") return send(200, { ok: true, recording: { active: false, session_id: null } });
    if (url.pathname === "/api/fc/status") return send(200, { ok: true, status: { armed: false } });
    if (url.pathname === "/api/altitude/config") {
      if (req.method === "GET") return send(200, { baro_noise_cm: 150 });
      return send(200, { baro_noise_cm: 180 });
    }
    if (url.pathname === "/api/altitude/config/save") return send(200, { saved: true });
    if (url.pathname === "/api/recording/status") return send(200, { recording: { active: false, session_id: "s1" } });
    if (url.pathname === "/api/recommendations/pending") return send(200, { exists: false, recommendation: null });
    if (url.pathname === "/api/recommendations/pending/consume") return send(200, { exists: true });
    if (url.pathname === "/api/altitude/sessions") return send(200, [
      { session_id: "s1", created_utc: "20260525T000000Z", duration_s: 9.4 },
      { session_id: "s2", created_utc: "20260525T000100Z" }
    ]);
    if (url.pathname === "/api/altitude/sessions/s1/download") {
      const zip = new AdmZip();
      zip.addFile("s1/manifest.json", Buffer.from(JSON.stringify({ duration_s: 9.4 })));
      zip.addFile("s1/events.jsonl", Buffer.from(""));
      zip.addFile("s1/telemetry.jsonl", Buffer.from([
        JSON.stringify({ host_time_ns: 1000000000, status: { altitude_cm: 10, velocity_cms: 1 } }),
        JSON.stringify({ host_time_ns: 2000000000, status: { altitude_cm: 12, velocity_cms: 2 } })
      ].join("\n")));
      const body = zip.toBuffer();
      res.writeHead(200, { "content-type": "application/zip", "content-length": body.length });
      return res.end(body);
    }
    if (url.pathname === "/api/altitude/sessions/s2/download") {
      const zip = new AdmZip();
      zip.addFile("s2/manifest.json", Buffer.from("{}"));
      zip.addFile("s2/events.jsonl", Buffer.from([
        JSON.stringify({ host_time_ns: 1000000000, event: "start" }),
        JSON.stringify({ host_time_ns: 4240000000, event: "stop" })
      ].join("\n")));
      zip.addFile("s2/telemetry.jsonl", Buffer.from(""));
      const body = zip.toBuffer();
      res.writeHead(200, { "content-type": "application/zip", "content-length": body.length });
      return res.end(body);
    }
    if (url.pathname === "/api/altitude/sessions/s1" && req.method === "DELETE") return send(200, { deleted: true, session_id: "s1" });
    return send(404, { error: "not found" });
  });
  return new Promise((resolve) => {
    server.listen(0, "127.0.0.1", () => {
      const address = server.address();
      assert(address && typeof address === "object");
      resolve({ server, base: `http://127.0.0.1:${address.port}`, calls });
    });
  });
}

test("operator login and pairing create an authenticated session", async () => {
  const tmp = await fs.mkdtemp(path.join(os.tmpdir(), "althold-tuning-"));
  const pi = await createFakePi();
  const config = baseConfig(tmp, pi.base);
  const { server, base, sessionCookie } = await authenticatedBase(config);
  try {
    const session = await fetch(`${base}/api/session`, { headers: { cookie: sessionCookie } });
    assert.equal(session.status, 200);
    assert.equal((await session.json() as { authenticated: boolean }).authenticated, true);
    const revoked = await fetch(`${base}/api/session/revoke-current`, { method: "POST", headers: { cookie: sessionCookie, origin: base } });
    assert.equal(revoked.status, 200);
    const after = await fetch(`${base}/api/session`, { headers: { cookie: sessionCookie } });
    assert.equal((await after.json() as { authenticated: boolean }).authenticated, false);
  } finally {
    await close(server);
    await close(pi.server);
  }
});

test("laptop backend proxies Pi config and downloads sessions", async () => {
  const tmp = await fs.mkdtemp(path.join(os.tmpdir(), "althold-tuning-"));
  const pi = await createFakePi();
  const config = baseConfig(tmp, pi.base);
  const { server, base, sessionCookie } = await authenticatedBase(config);
  try {
    const headers = { cookie: sessionCookie, origin: base, "content-type": "application/json" };
    const cfg = await fetch(`${base}/api/altitude/config`, { headers: { cookie: sessionCookie } });
    assert.equal((await cfg.json() as { baro_noise_cm: number }).baro_noise_cm, 150);
    const apply = await fetch(`${base}/api/altitude/config/apply`, { method: "POST", headers, body: JSON.stringify({ baro_noise_cm: 180 }) });
    assert.equal(apply.status, 200);
    const download = await fetch(`${base}/api/sessions/s1/download`, { method: "POST", headers });
    assert.equal(download.status, 200);
    assert.equal(await fs.stat(path.join(tmp, "logs", "s1", "manifest.json")).then(() => true), true);
    const list = await fetch(`${base}/api/sessions`, { headers: { cookie: sessionCookie } });
    const piSessions = await list.json() as Array<{ duration_s?: number }>;
    assert.equal(piSessions.length, 2);
    assert.equal(piSessions[0].duration_s, 9.4);
    const all = await fetch(`${base}/api/sessions/download-all`, { method: "POST", headers });
    assert.equal((await all.json() as { count: number }).count, 2);
    assert.equal(await fs.stat(path.join(tmp, "logs", "s2", "manifest.json")).then(() => true), true);
    const localList = await fetch(`${base}/api/local-sessions`, { headers: { cookie: sessionCookie } });
    const localSessions = await localList.json() as Array<{ session_id: string; duration_s: number }>;
    assert.deepEqual(localSessions.map((session) => session.session_id).sort(), ["s1", "s2"]);
    assert.equal(localSessions.find((session) => session.session_id === "s1")?.duration_s, 9.4);
    assert.equal(localSessions.find((session) => session.session_id === "s2")?.duration_s, 3.24);
    const plot = await fetch(`${base}/api/local-sessions/s1/plot-data`, { headers: { cookie: sessionCookie } });
    const plotData = await plot.json() as { sample_count: number; fields: Array<{ name: string }>; samples: Array<Record<string, number>> };
    assert.equal(plotData.sample_count, 2);
    assert.equal(plotData.fields.some((field) => field.name === "status.altitude_cm"), true);
    assert.equal(plotData.samples[1]["status.altitude_cm"], 12);
    const deletedLocal = await fetch(`${base}/api/local-sessions/s1`, { method: "DELETE", headers });
    assert.equal((await deletedLocal.json() as { deleted: boolean }).deleted, true);
    await assert.rejects(fs.stat(path.join(tmp, "logs", "s1", "manifest.json")));
    const deleted = await fetch(`${base}/api/sessions/s1`, { method: "DELETE", headers });
    assert.equal((await deleted.json() as { deleted: boolean }).deleted, true);
  } finally {
    await close(server);
    await close(pi.server);
  }
});

test("Codex runtime validation blocks missing model", async () => {
  const tmp = await fs.mkdtemp(path.join(os.tmpdir(), "althold-tuning-"));
  const pi = await createFakePi();
  await fs.mkdir(path.join(tmp, "logs", "s1"), { recursive: true });
  await fs.writeFile(path.join(tmp, "logs", "s1", "telemetry.jsonl"), "");
  await fs.writeFile(path.join(tmp, "logs", "s1", "events.jsonl"), "");

  const wss = new WebSocketServer({ port: 0, host: "127.0.0.1" });
  await new Promise<void>((resolve) => wss.once("listening", resolve));
  const address = wss.address();
  assert(address && typeof address === "object");
  wss.on("connection", (ws) => {
    ws.on("message", (raw) => {
      const msg = JSON.parse(raw.toString());
      if (msg.id && msg.method === "initialize") ws.send(JSON.stringify({ jsonrpc: "2.0", id: msg.id, result: {} }));
      if (msg.id && msg.method === "model/list") ws.send(JSON.stringify({ jsonrpc: "2.0", id: msg.id, result: { data: [] } }));
    });
  });

  const config = baseConfig(tmp, pi.base, `ws://127.0.0.1:${address.port}`);
  const { server, base, sessionCookie } = await authenticatedBase(config);
  try {
    const response = await fetch(`${base}/api/analysis/analyze`, {
      method: "POST",
      headers: { cookie: sessionCookie, origin: base, "content-type": "application/json" },
      body: JSON.stringify({ session_ids: ["s1"] })
    });
    assert.equal(response.status, 500);
    assert.match((await response.json() as { error: string }).error, /not available/);
  } finally {
    await close(server);
    await close(pi.server);
    wss.close();
  }
});

test("Codex runtime validation accepts current app-server model metadata shape", async () => {
  const tmp = await fs.mkdtemp(path.join(os.tmpdir(), "althold-tuning-"));
  const pi = await createFakePi();
  await fs.mkdir(path.join(tmp, "logs", "s1"), { recursive: true });
  await fs.writeFile(path.join(tmp, "logs", "s1", "telemetry.jsonl"), [
    JSON.stringify({ host_time_ns: 1000000000, status: { altitude_cm: 10 } }),
    JSON.stringify({ host_time_ns: 2000000000, status: { altitude_cm: 11 } })
  ].join("\n"));
  await fs.writeFile(path.join(tmp, "logs", "s1", "events.jsonl"), "");

  const wss = new WebSocketServer({ port: 0, host: "127.0.0.1" });
  await new Promise<void>((resolve) => wss.once("listening", resolve));
  const address = wss.address();
  assert(address && typeof address === "object");
  wss.on("connection", (ws) => {
    ws.on("message", async (raw) => {
      const msg = JSON.parse(raw.toString());
      if (msg.id && msg.method === "initialize") ws.send(JSON.stringify({ jsonrpc: "2.0", id: msg.id, result: {} }));
      if (msg.id && msg.method === "model/list") {
        ws.send(JSON.stringify({
          jsonrpc: "2.0",
          id: msg.id,
          result: {
            data: [{
              id: "gpt-5.5",
              displayName: "GPT-5.5",
              supportedReasoningEfforts: [{ reasoningEffort: "xhigh", description: "Extra high reasoning depth" }],
              additionalSpeedTiers: ["fast"]
            }]
          }
        }));
      }
      if (msg.id && msg.method === "thread/start") ws.send(JSON.stringify({ jsonrpc: "2.0", id: msg.id, result: { threadId: "thread-1" } }));
      if (msg.id && msg.method === "turn/start") {
        const recommendation = {
          schema: "althold-recommendation-v1",
          session_id: "s1",
          created_utc: new Date().toISOString(),
          source_thread_id: "thread-1",
          parameters: []
        };
        await fs.writeFile(path.join(tmp, "logs", "s1", "recommendation.json"), JSON.stringify(recommendation), "utf8");
        ws.send(JSON.stringify({ jsonrpc: "2.0", id: msg.id, result: {} }));
      }
    });
  });

  const config = baseConfig(tmp, pi.base, `ws://127.0.0.1:${address.port}`);
  const { server, base, sessionCookie } = await authenticatedBase(config);
  try {
    const response = await fetch(`${base}/api/analysis/analyze`, {
      method: "POST",
      headers: { cookie: sessionCookie, origin: base, "content-type": "application/json" },
      body: JSON.stringify({ session_ids: ["s1"] })
    });
    assert.equal(response.status, 200);
    assert.equal((await response.json() as { threadId: string }).threadId, "thread-1");
  } finally {
    await close(server);
    await close(pi.server);
    wss.close();
  }
});

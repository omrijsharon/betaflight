import fs from "node:fs/promises";
import path from "node:path";
import { pipeline } from "node:stream/promises";
import { createWriteStream } from "node:fs";
import { Readable } from "node:stream";
import AdmZip from "adm-zip";

export interface PiClientOptions {
  baseUrl: string;
  localLogDir: string;
}

export class PiClient {
  readonly baseUrl: string;
  readonly localLogDir: string;

  constructor(options: PiClientOptions) {
    this.baseUrl = options.baseUrl.replace(/\/+$/, "");
    this.localLogDir = options.localLogDir;
  }

  async requestJson<T = unknown>(method: string, route: string, body?: unknown): Promise<T> {
    const response = await fetch(`${this.baseUrl}${route}`, {
      method,
      headers: body === undefined ? undefined : { "content-type": "application/json" },
      body: body === undefined ? undefined : JSON.stringify(body)
    });
    const text = await response.text();
    const parsed = text ? JSON.parse(text) : {};
    if (!response.ok) {
      const message = typeof parsed?.error === "string" ? parsed.error : response.statusText;
      throw new Error(message);
    }
    return parsed as T;
  }

  health(): Promise<unknown> {
    return this.requestJson("GET", "/healthz");
  }

  fcStatus(): Promise<unknown> {
    return this.requestJson("GET", "/api/fc/status");
  }

  altitudeStatus(): Promise<unknown> {
    return this.requestJson("GET", "/api/altitude/status");
  }

  getConfig(): Promise<Record<string, unknown>> {
    return this.requestJson("GET", "/api/altitude/config");
  }

  applyConfig(config: Record<string, unknown>): Promise<Record<string, unknown>> {
    return this.requestJson("POST", "/api/altitude/config", config);
  }

  saveConfig(): Promise<unknown> {
    return this.requestJson("POST", "/api/altitude/config/save");
  }

  rebootFc(): Promise<unknown> {
    return this.requestJson("POST", "/api/altitude/fc/reboot");
  }

  recordingStatus(): Promise<unknown> {
    return this.requestJson("GET", "/api/recording/status");
  }

  listSessions(): Promise<Array<Record<string, unknown>>> {
    return this.requestJson("GET", "/api/altitude/sessions");
  }

  startRecording(): Promise<unknown> {
    return this.requestJson("POST", "/api/altitude/logging/start");
  }

  stopRecording(): Promise<unknown> {
    return this.requestJson("POST", "/api/altitude/logging/stop");
  }

  pendingRecommendation(): Promise<unknown> {
    return this.requestJson("GET", "/api/recommendations/pending");
  }

  sendRecommendation(recommendation: Record<string, unknown>): Promise<unknown> {
    return this.requestJson("POST", "/api/recommendations/pending", recommendation);
  }

  consumeRecommendation(): Promise<unknown> {
    return this.requestJson("POST", "/api/recommendations/pending/consume");
  }

  async downloadSession(sessionId: string): Promise<{ sessionId: string; zipPath: string; extractDir: string }> {
    const response = await fetch(`${this.baseUrl}/api/altitude/sessions/${encodeURIComponent(sessionId)}/download`);
    if (!response.ok || !response.body) {
      throw new Error(`download failed: ${response.status} ${response.statusText}`);
    }
    await fs.mkdir(this.localLogDir, { recursive: true });
    const zipPath = path.join(this.localLogDir, `${sessionId}.zip`);
    const extractBase = this.localLogDir;
    const extractDir = path.join(extractBase, sessionId);
    await pipeline(Readable.fromWeb(response.body as never), createWriteStream(zipPath));
    await fs.rm(extractDir, { recursive: true, force: true });
    new AdmZip(zipPath).extractAllTo(extractBase, true);
    return { sessionId, zipPath, extractDir };
  }

  async downloadAllSessions(): Promise<Array<{ sessionId: string; zipPath: string; extractDir: string }>> {
    const sessions = await this.listSessions();
    const results = [];
    for (const session of sessions) {
      const sessionId = String(session.session_id ?? "");
      if (sessionId) {
        results.push(await this.downloadSession(sessionId));
      }
    }
    return results;
  }

  async deleteSession(sessionId: string): Promise<unknown> {
    return this.requestJson("DELETE", `/api/altitude/sessions/${encodeURIComponent(sessionId)}`);
  }
}

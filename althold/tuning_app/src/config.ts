import { randomBytes } from "node:crypto";
import path from "node:path";

export interface TuningConfig {
  host: string;
  port: number;
  publicBaseUrl: string;
  piBaseUrl: string;
  codexAppServerUrl: string;
  repoRoot: string;
  localLogDir: string;
  sessionSecret: string;
  operatorSecret: string;
  pairingTtlMs: number;
  sessionTtlMs: number;
  sessionRequestsPerMinute: number;
  pairingRequestsPerHour: number;
  modelName: string;
  reasoningLabel: string;
  speed: "standard" | "fast";
  statePath: string;
}

function envNumber(name: string, fallback: number): number {
  const raw = process.env[name];
  if (!raw) {
    return fallback;
  }
  const value = Number(raw);
  return Number.isFinite(value) ? value : fallback;
}

function isLocalBaseUrl(value: string): boolean {
  try {
    const url = new URL(value);
    return ["localhost", "127.0.0.1", "::1"].includes(url.hostname);
  } catch {
    return false;
  }
}

export function loadConfig(): TuningConfig {
  const host = (process.env.ALTHOLD_TUNING_HOST ?? "127.0.0.1").trim() || "127.0.0.1";
  const port = envNumber("ALTHOLD_TUNING_PORT", 8790);
  const publicBaseUrl = (process.env.ALTHOLD_TUNING_BASE_URL ?? `http://${host}:${port}`).trim();
  const repoRoot = (process.env.ALTHOLD_REPO_ROOT ?? path.resolve(process.cwd(), "..", "..")).trim();
  const operatorSecret = (process.env.ALTHOLD_TUNING_OPERATOR_SECRET ?? "").trim();
  const explicitSessionSecret = (process.env.ALTHOLD_TUNING_SESSION_SECRET ?? "").trim();
  const sessionSecret = explicitSessionSecret || randomBytes(24).toString("base64url");

  if (!isLocalBaseUrl(publicBaseUrl)) {
    if (!operatorSecret) {
      throw new Error("ALTHOLD_TUNING_OPERATOR_SECRET is required for a public base URL");
    }
    if (!explicitSessionSecret) {
      throw new Error("ALTHOLD_TUNING_SESSION_SECRET is required for a public base URL");
    }
  }

  return {
    host,
    port,
    publicBaseUrl,
    piBaseUrl: (process.env.ALTHOLD_PI_BASE_URL ?? "http://omrijsharon.local:8080").trim(),
    codexAppServerUrl: (process.env.ALTHOLD_CODEX_APP_SERVER_URL ?? "ws://127.0.0.1:4500").trim(),
    repoRoot,
    localLogDir: path.resolve(repoRoot, process.env.ALTHOLD_LOCAL_LOG_DIR ?? "althold/logs"),
    sessionSecret,
    operatorSecret,
    pairingTtlMs: envNumber("ALTHOLD_TUNING_PAIRING_TTL_MS", 60_000),
    sessionTtlMs: envNumber("ALTHOLD_TUNING_SESSION_TTL_MS", 30 * 24 * 60 * 60 * 1000),
    sessionRequestsPerMinute: envNumber("ALTHOLD_TUNING_SESSION_REQ_PER_MIN", 60),
    pairingRequestsPerHour: envNumber("ALTHOLD_TUNING_PAIRING_REQ_PER_HOUR", 20),
    modelName: (process.env.ALTHOLD_CODEX_MODEL ?? "gpt-5.5").trim(),
    reasoningLabel: (process.env.ALTHOLD_CODEX_REASONING_LABEL ?? "Extra High").trim(),
    speed: process.env.ALTHOLD_CODEX_SPEED === "standard" ? "standard" : "fast",
    statePath: path.resolve(repoRoot, process.env.ALTHOLD_TUNING_STATE_PATH ?? "althold/tuning_app/.state/state.json")
  };
}

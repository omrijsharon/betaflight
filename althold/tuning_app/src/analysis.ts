import { spawn } from "node:child_process";
import fs from "node:fs/promises";
import path from "node:path";

export interface RecommendationParameter {
  name: string;
  old_value: number | string;
  new_value: number | string;
  unit: string;
  reason: string;
  confidence: "low" | "medium" | "high";
}

export interface Recommendation {
  schema: "althold-recommendation-v1";
  session_id: string;
  created_utc: string;
  source_thread_id: string | null;
  parameters: RecommendationParameter[];
}

export async function runLocalAnalysis(repoRoot: string, sessionDir: string): Promise<{ outputDir: string; summaryPath: string; csvPath: string }> {
  const outputDir = path.join(sessionDir, "analysis");
  await fs.mkdir(outputDir, { recursive: true });
  const script = path.join(repoRoot, "althold", "analysis", "analyze_altitude_log.py");
  await new Promise<void>((resolve, reject) => {
    const child = spawn("python", [script, sessionDir, "--output-dir", outputDir], { cwd: repoRoot, stdio: "pipe" });
    let stderr = "";
    child.stderr.on("data", (chunk) => {
      stderr += chunk.toString();
    });
    child.on("error", reject);
    child.on("close", (code) => {
      if (code === 0) {
        resolve();
      } else {
        reject(new Error(stderr || `analysis exited with ${code}`));
      }
    });
  });
  return {
    outputDir,
    summaryPath: path.join(outputDir, "summary.json"),
    csvPath: path.join(outputDir, "summary.csv")
  };
}

export function buildAnalysisPrompt(input: {
  sessionId: string;
  sessionDir: string;
  summaryPath: string;
  csvPath: string;
  currentConfig: Record<string, unknown>;
  recommendationPath: string;
  sourceThreadId: string;
}): string {
  return [
    "Analyze this altitude estimator flight-log session and recommend EKF tuning changes.",
    "",
    `Session id: ${input.sessionId}`,
    `Session directory: ${input.sessionDir}`,
    `Summary JSON: ${input.summaryPath}`,
    `Summary CSV: ${input.csvPath}`,
    `Recommendation JSON path to create/overwrite: ${input.recommendationPath}`,
    `Codex tuning thread id to write as source_thread_id: ${input.sourceThreadId}`,
    "",
    "Current altitude estimator config:",
    JSON.stringify(input.currentConfig, null, 2),
    "",
    "Use the local AGENTS.md instructions. You may inspect raw logs and improve local analysis tools only if the existing tools are insufficient for this session.",
    "For this UI-triggered tuning run, do not run broad test suites and do not attempt cleanup outside the session directory.",
    "Create/overwrite recommendation.json at the exact path above, then stop.",
    "Return only a brief explanation after writing the JSON.",
    "The JSON schema must be althold-recommendation-v1 with parameters containing name, old_value, new_value, unit, reason, and confidence."
  ].join("\n");
}

export function buildMultiSessionAnalysisPrompt(input: {
  runId: string;
  runDir: string;
  sessions: Array<{ sessionId: string; sessionDir: string; summaryPath: string; csvPath: string }>;
  currentConfig: Record<string, unknown>;
  recommendationPath: string;
  sourceThreadId: string;
}): string {
  const sessionLines = input.sessions.flatMap((session, index) => [
    `Session ${index + 1}: ${session.sessionId}`,
    `  Directory: ${session.sessionDir}`,
    `  Summary JSON: ${session.summaryPath}`,
    `  Summary CSV: ${session.csvPath}`
  ]);
  return [
    "Analyze these altitude estimator flight-log sessions together and recommend EKF tuning changes.",
    "",
    `Analysis run id: ${input.runId}`,
    `Analysis run directory: ${input.runDir}`,
    `Recommendation JSON path to create/overwrite: ${input.recommendationPath}`,
    `Codex tuning thread id to write as source_thread_id: ${input.sourceThreadId}`,
    "",
    ...sessionLines,
    "",
    "Current altitude estimator config:",
    JSON.stringify(input.currentConfig, null, 2),
    "",
    "Use the local AGENTS.md instructions. You may inspect raw logs and improve local analysis tools only if the existing tools are insufficient for this analysis run.",
    "For this UI-triggered tuning run, do not run broad test suites and do not attempt cleanup outside the analysis run directory.",
    "Create/overwrite recommendation.json at the exact path above, then stop.",
    "Return only a brief explanation after writing the JSON.",
    "The JSON schema must be althold-recommendation-v1 with parameters containing name, old_value, new_value, unit, reason, and confidence.",
    "Set session_id to the analysis run id, and include a source_sessions array with the analyzed session ids."
  ].join("\n");
}

export function emptyRecommendation(sessionId: string, threadId: string | null): Recommendation {
  return {
    schema: "althold-recommendation-v1",
    session_id: sessionId,
    created_utc: new Date().toISOString(),
    source_thread_id: threadId,
    parameters: []
  };
}

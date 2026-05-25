import { IncomingMessage, ServerResponse } from "node:http";

export function readBody(req: IncomingMessage): Promise<Buffer> {
  return new Promise((resolve, reject) => {
    const chunks: Buffer[] = [];
    req.on("data", (chunk) => chunks.push(Buffer.from(chunk)));
    req.on("end", () => resolve(Buffer.concat(chunks)));
    req.on("error", reject);
  });
}

export async function readJson<T = unknown>(req: IncomingMessage): Promise<T> {
  const raw = await readBody(req);
  if (!raw.length) {
    return {} as T;
  }
  return JSON.parse(raw.toString("utf8")) as T;
}

export function sendJson(res: ServerResponse, status: number, data: unknown, headers: Record<string, string> = {}): void {
  const body = JSON.stringify(data);
  res.writeHead(status, {
    "content-type": "application/json; charset=utf-8",
    "content-length": Buffer.byteLength(body).toString(),
    ...headers
  });
  res.end(body);
}

export function sendText(res: ServerResponse, status: number, body: string, contentType = "text/plain; charset=utf-8", headers: Record<string, string> = {}): void {
  res.writeHead(status, {
    "content-type": contentType,
    "content-length": Buffer.byteLength(body).toString(),
    ...headers
  });
  res.end(body);
}

export function parseCookies(req: IncomingMessage): Record<string, string> {
  const out: Record<string, string> = {};
  const header = req.headers.cookie;
  if (!header) {
    return out;
  }
  for (const part of header.split(";")) {
    const index = part.indexOf("=");
    if (index > 0) {
      out[part.slice(0, index).trim()] = decodeURIComponent(part.slice(index + 1).trim());
    }
  }
  return out;
}

export function cookie(name: string, value: string, baseUrl: string, maxAgeSeconds?: number): string {
  const secure = baseUrl.startsWith("https://") ? "; Secure" : "";
  const maxAge = typeof maxAgeSeconds === "number" ? `; Max-Age=${maxAgeSeconds}` : "";
  return `${name}=${encodeURIComponent(value)}; HttpOnly; Path=/; SameSite=Lax${secure}${maxAge}`;
}

export function clearCookie(name: string, baseUrl: string): string {
  return cookie(name, "", baseUrl, 0);
}

export function sameOriginAllowed(req: IncomingMessage, publicBaseUrl: string): boolean {
  const method = (req.method ?? "GET").toUpperCase();
  if (!["POST", "PUT", "PATCH", "DELETE"].includes(method)) {
    return true;
  }
  const origin = req.headers.origin || req.headers.referer;
  if (!origin || typeof origin !== "string") {
    return false;
  }
  try {
    return new URL(origin).origin === new URL(publicBaseUrl).origin;
  } catch {
    return false;
  }
}

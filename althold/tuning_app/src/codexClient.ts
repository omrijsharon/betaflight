import { EventEmitter } from "node:events";
import WebSocket from "ws";

interface PendingCall {
  resolve: (value: unknown) => void;
  reject: (error: Error) => void;
  timeout: NodeJS.Timeout;
}

export class CodexAppServerClient extends EventEmitter {
  private ws: WebSocket | null = null;
  private nextId = 1;
  private pending = new Map<number, PendingCall>();
  private initialized = false;

  constructor(private readonly url: string) {
    super();
  }

  connected(): boolean {
    return Boolean(this.ws && this.ws.readyState === WebSocket.OPEN && this.initialized);
  }

  close(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
    this.initialized = false;
    for (const [id, pending] of this.pending.entries()) {
      clearTimeout(pending.timeout);
      pending.reject(new Error("app-server connection closed"));
      this.pending.delete(id);
    }
  }

  async connect(): Promise<void> {
    if (this.connected()) {
      return;
    }
    this.ws = new WebSocket(this.url);
    await new Promise<void>((resolve, reject) => {
      const ws = this.ws;
      if (!ws) {
        reject(new Error("WebSocket was not created"));
        return;
      }
      ws.once("open", resolve);
      ws.once("error", reject);
      ws.on("message", (data) => this.onMessage(data.toString()));
      ws.on("close", () => {
        this.initialized = false;
        this.emit("close");
      });
    });
    await this.call("initialize", {
      clientInfo: { name: "althold-tuning-app", title: "Altitude Tuning App", version: "0.1.0" },
      capabilities: { experimentalApi: true }
    });
    this.notify("notifications/initialized", {});
    this.initialized = true;
  }

  async call<T = unknown>(method: string, params: Record<string, unknown> = {}, timeoutMs = 30_000): Promise<T> {
    await this.ensureSocket();
    const id = this.nextId++;
    const ws = this.ws;
    if (!ws) {
      throw new Error("app-server not connected");
    }
    const payload = { jsonrpc: "2.0", id, method, params };
    const promise = new Promise<T>((resolve, reject) => {
      const timeout = setTimeout(() => {
        this.pending.delete(id);
        reject(new Error(`app-server ${method} timed out`));
      }, timeoutMs);
      this.pending.set(id, {
        resolve: (value) => resolve(value as T),
        reject,
        timeout
      });
    });
    ws.send(JSON.stringify(payload));
    return promise;
  }

  notify(method: string, params: Record<string, unknown>): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error("app-server not connected");
    }
    this.ws.send(JSON.stringify({ jsonrpc: "2.0", method, params }));
  }

  async listModels(): Promise<Record<string, unknown>[]> {
    const result = await this.call<{ data?: Record<string, unknown>[] }>("model/list", {});
    return Array.isArray(result.data) ? result.data : [];
  }

  async startThread(params: Record<string, unknown>): Promise<string> {
    const result = await this.call<Record<string, unknown>>("thread/start", params);
    const threadId = String(result.threadId ?? (result.thread as Record<string, unknown> | undefined)?.id ?? result.id ?? "");
    if (!threadId) {
      throw new Error("thread/start did not return a thread id");
    }
    return threadId;
  }

  async resumeThread(threadId: string): Promise<void> {
    await this.call("thread/resume", { threadId });
  }

  async startTurn(params: Record<string, unknown>): Promise<void> {
    await this.call("turn/start", params, 120_000);
  }

  private async ensureSocket(): Promise<void> {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      throw new Error("app-server not connected");
    }
  }

  private onMessage(raw: string): void {
    let msg: Record<string, unknown>;
    try {
      msg = JSON.parse(raw) as Record<string, unknown>;
    } catch {
      return;
    }
    if (typeof msg.id === "number") {
      const pending = this.pending.get(msg.id);
      if (!pending) {
        return;
      }
      clearTimeout(pending.timeout);
      this.pending.delete(msg.id);
      if (msg.error) {
        const error = msg.error as { message?: string };
        pending.reject(new Error(error.message || "app-server error"));
      } else {
        pending.resolve(msg.result);
      }
      return;
    }
    this.emit("notification", msg);
  }
}

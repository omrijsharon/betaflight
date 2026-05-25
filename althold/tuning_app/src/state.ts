import { createHash, randomUUID } from "node:crypto";

export interface PairingRecord {
  pairingId: string;
  token: string;
  createdAt: number;
  expiresAt: number;
  usedAt?: number;
}

export interface SessionRecord {
  sessionId: string;
  pairingId: string;
  createdAt: number;
  expiresAt: number;
  ipAddress?: string;
  userAgent?: string;
}

export class PairingStore {
  private readonly pairings = new Map<string, PairingRecord>();

  create(ttlMs: number): PairingRecord {
    const pairing = {
      pairingId: randomUUID(),
      token: randomUUID(),
      createdAt: Date.now(),
      expiresAt: Date.now() + ttlMs
    };
    this.pairings.set(pairing.token, pairing);
    return pairing;
  }

  consume(token: string): PairingRecord | null {
    const pairing = this.pairings.get(token);
    if (!pairing || pairing.usedAt || pairing.expiresAt <= Date.now()) {
      return null;
    }
    pairing.usedAt = Date.now();
    return pairing;
  }

  cleanup(): void {
    const now = Date.now();
    for (const [token, pairing] of this.pairings.entries()) {
      if (pairing.usedAt || pairing.expiresAt <= now) {
        this.pairings.delete(token);
      }
    }
  }
}

export class SessionStore {
  private readonly sessions = new Map<string, SessionRecord>();

  create(pairingId: string, ttlMs: number, metadata: Partial<SessionRecord>): SessionRecord {
    const session = {
      sessionId: randomUUID(),
      pairingId,
      createdAt: Date.now(),
      expiresAt: Date.now() + ttlMs,
      ipAddress: metadata.ipAddress,
      userAgent: metadata.userAgent
    };
    this.sessions.set(session.sessionId, session);
    return session;
  }

  get(sessionId: string): SessionRecord | null {
    const session = this.sessions.get(sessionId);
    if (!session || session.expiresAt <= Date.now()) {
      if (session) {
        this.sessions.delete(sessionId);
      }
      return null;
    }
    return session;
  }

  revoke(sessionId: string): boolean {
    return this.sessions.delete(sessionId);
  }

  revokeAll(): void {
    this.sessions.clear();
  }
}

export function signSession(sessionId: string, secret: string): string {
  const signature = createHash("sha256").update(`${sessionId}:${secret}`).digest("hex");
  return `${sessionId}.${signature}`;
}

export function readSignedSession(value: string | undefined, secret: string, store: SessionStore): SessionRecord | null {
  if (!value) {
    return null;
  }
  const dot = value.indexOf(".");
  if (dot <= 0) {
    return null;
  }
  const sessionId = value.slice(0, dot);
  if (signSession(sessionId, secret) !== value) {
    return null;
  }
  return store.get(sessionId);
}

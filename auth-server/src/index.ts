
import { createServer } from "http";
import { auth } from "better-auth";

/**
 * Railway injects env vars automatically.
 * DO NOT use dotenv here.
 */
const PORT = Number(process.env.PORT || 3000);
const HOST = process.env.HOST || "0.0.0.0";

const AUTH_SECRET = process.env.AUTH_SECRET;
const AUTH_BASE_URL = process.env.AUTH_BASE_URL;
const DATABASE_URL = process.env.DATABASE_URL;

if (!AUTH_SECRET || !AUTH_BASE_URL || !DATABASE_URL) {
  throw new Error("Missing required environment variables");
}

const authHandler = auth({
  baseURL: AUTH_BASE_URL,
  secret: AUTH_SECRET,
  database: {
    url: DATABASE_URL
  }
});

const server = createServer(async (req, res) => {
  if (!req.url) {
    res.statusCode = 400;
    res.end("Bad Request");
    return;
  }

  if (req.url.startsWith("/auth")) {
    await authHandler(req, res);
    return;
  }

  res.statusCode = 200;
  res.end("Auth server running");
});

server.listen(PORT, HOST, () => {
  console.log(`Auth server running on ${HOST}:${PORT}`);
});

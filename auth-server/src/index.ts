import express from "express";
import { betterAuth } from "better-auth";

const app = express();
app.use(express.json());

const auth = betterAuth({
  database: {
    type: "postgres",
    url: process.env.DATABASE_URL!
  },
  secret: process.env.AUTH_SECRET!,
  baseURL: process.env.AUTH_BASE_URL!
});

/**
 * Minimal HTTP bridge (NO @better-auth/node)
 */
app.post("/auth/*", async (req, res) => {
  const response = await auth.fetch(req);
  res.status(response.status);
  response.headers.forEach((v, k) => res.setHeader(k, v));
  res.send(await response.text());
});

const port = process.env.PORT || 3000;
app.listen(port, () => {
  console.log(`Auth server running on port ${port}`);
});

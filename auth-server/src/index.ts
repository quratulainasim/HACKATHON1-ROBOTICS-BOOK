import express from "express";
import { betterAuth } from "better-auth";

const app = express();

app.use(express.json());

const auth = betterAuth({
  app: {
    name: "Auth Server",
    baseURL: process.env.AUTH_BASE_URL, // ✅ correct casing
  },
  database: {
    url: process.env.DATABASE_URL!,
    type: "postgres",
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },
  secret: process.env.AUTH_SECRET!,
});

// ✅ USE DIRECTLY AS MIDDLEWARE
app.use("/auth", auth);

app.get("/health", (_req, res) => {
  res.json({ ok: true });
});

const port = Number(process.env.PORT) || 3000;
app.listen(port, "0.0.0.0", () => {
  console.log(`Auth server running on port ${port}`);
});

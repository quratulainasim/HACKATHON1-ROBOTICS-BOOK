import express from "express";
import { betterAuth } from "better-auth";
import { toNodeHandler } from "better-auth/node";

const app = express();

// Create auth instance
const auth = betterAuth({
  secret: process.env.AUTH_SECRET!,
  baseURL: process.env.AUTH_BASE_URL!, // ← اب baseURL (capital URL)
  // یہاں database, emailAndPassword, socialProviders وغیرہ add کرو اگر ضرورت ہو
});

// Better Auth handler کو exact path پر mount کرو
app.use("/api/auth", toNodeHandler(auth));

// IMPORTANT: express.json() کو Better Auth handler کے بعد لگاؤ
app.use(express.json());

// Health check
app.get("/", (_req, res) => {
  res.send("Auth server running");
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
});

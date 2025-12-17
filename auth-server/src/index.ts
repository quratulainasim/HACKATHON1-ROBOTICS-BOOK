import express from "express";
import { betterAuth } from "better-auth";
import { toNodeHandler } from "better-auth/node";

const app = express();

// Better Auth instance
const auth = betterAuth({
  secret: process.env.AUTH_SECRET!,
  baseURL: process.env.AUTH_BASE_URL!, // ← یہی صحیح ہے: baseURL (capital U اور L)

  // یہاں database adapter add کرو (example کے لیے comment کیا ہوا ہے)
  // database: drizzleAdapter(db, { provider: "pg" }),

  // email/password یا social providers enable کرو اگر چاہیے
  // emailAndPassword: { enabled: true },
});

// All auth routes کو handle کرو (Express v4 کے لیے یہ syntax)
app.all("/api/auth/*", toNodeHandler(auth));

// IMPORTANT: express.json() کو auth handler کے بعد رکھو
app.use(express.json());

// Simple health check
app.get("/", (_req, res) => {
  res.send("Auth server running 🚀");
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
});

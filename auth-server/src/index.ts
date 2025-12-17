import express from "express";
import { betterAuth } from "better-auth";
import { toNodeHandler } from "better-auth/node";

const app = express();

const auth = betterAuth({
  secret: process.env.AUTH_SECRET!,
  baseURL: process.env.AUTH_BASE_URL!,

  // ← یہ add کرو: email/password enable
  emailAndPassword: {
    enabled: true,
    // optional: اگر email verification چاہیے تو
    // requireEmailVerification: true,
    // disableSignUp: false, // default false ہے
  },

  // اگر database connect کرنا ہے تو یہاں add کرو (ابھی کے لیے optional)
  // database: drizzleAdapter(db, { provider: "pg" }),
});

// All auth routes
app.all("/api/auth/*", toNodeHandler(auth));

app.use(express.json());

app.get("/", (_req, res) => {
  res.send("Auth server running 🚀<br><br>اب sign-up endpoints available ہیں!");
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
});

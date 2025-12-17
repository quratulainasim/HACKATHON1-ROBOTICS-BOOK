import express from "express";
import cors from "cors"; // ← Added
import { betterAuth } from "better-auth";
import { toNodeHandler } from "better-auth/node";

const app = express();

// ← CORS middleware (add this BEFORE the auth handler)
app.use(
  cors({
    origin: "https://hackathon-1-robotics-book-n1zvtvbcd.vercel.app", // Your exact Vercel domain
    credentials: true, // Required for sessions/cookies
    methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allowedHeaders: ["Content-Type", "Authorization"],
  })
);

// Better Auth instance
const auth = betterAuth({
  secret: process.env.AUTH_SECRET!,
  baseURL: process.env.AUTH_BASE_URL!,

  emailAndPassword: {
    enabled: true,
    // requireEmailVerification: true, // Uncomment if you want email verification later
  },

  // database: drizzleAdapter(db, { provider: "pg" }), // Add later when ready
});

// All auth routes – must come AFTER CORS
app.all("/api/auth/*", toNodeHandler(auth));

// express.json() after auth handler (important!)
app.use(express.json());

// Health check
app.get("/", (_req, res) => {
  res.send("Auth server running 🚀<br><br>Sign-up endpoints are now available and CORS enabled!");
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
});

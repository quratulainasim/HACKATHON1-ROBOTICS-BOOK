import express from "express";
import cors from "cors";
import dotenv from "dotenv";
import { betterAuth } from "better-auth";
import { toNodeHandler } from "better-auth/node";  // Critical for Express

dotenv.config();

const app = express();
const PORT = process.env.PORT || 3001;

// FIXED CORS for your Vercel frontend
app.use(cors({
  origin: [
    "http://localhost:3000",
    "https://hackathon-1-robotics-book.vercel.app",
    "https://hackathon-1-robotics-book-oono8cq32.vercel.app",
  ],
  credentials: true,
  methods: ["GET", "POST", "PUT", "DELETE", "OPTIONS"],
  allowedHeaders: ["Content-Type", "Authorization"],
}));

// Handle preflight
app.options("*", cors());

// DO NOT put express.json() before Better Auth handler
// Better Auth needs raw body for some requests

// Initialize Better Auth
const auth = betterAuth({
  app: {
    name: "Physical AI Robotics Auth",
    baseURL: process.env.AUTH_BASE_URL || `http://localhost:${PORT}`,
    siteURL: process.env.FRONTEND_URL || "https://hackathon-1-robotics-book.vercel.app",
  },
  database: {
    url: process.env.DATABASE_URL,
    type: "postgres",
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },
  secret: process.env.AUTH_SECRET || "fallback-secret-for-dev",
});

// Mount Better Auth with node adapter
app.all("/api/auth/*", toNodeHandler(auth));  // This handles all Better Auth routes + CORS internally

// express.json() AFTER Better Auth handler
app.use(express.json());

// Health check
app.get("/health", (req, res) => {
  res.json({ status: "OK", timestamp: new Date().toISOString() });
});

app.listen(PORT, () => {
  console.log(`Auth server running on port ${PORT}`);
  console.log(`Frontend allowed: your Vercel URL + localhost`);
});

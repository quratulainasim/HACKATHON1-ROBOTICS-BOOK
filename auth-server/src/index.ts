import express from "express";
import { betterAuth } from "better-auth";
import { toNodeHandler } from "better-auth/node";

const app = express();

// Create auth instance
const auth = betterAuth({
  secret: process.env.AUTH_SECRET!,
  baseURL: process.env.AUTH_BASE_URL!, // e.g., "https://your-app.up.railway.app"
  // basePath: "/api/auth", // Optional: uncomment if you want to make it configurable
  // Add database, emailAndPassword, socialProviders, etc. here
});

// Mount Better Auth on /api/auth/*
app.all("/api/auth/*", toNodeHandler(auth));

// Place express.json() AFTER the auth handler (important!)
app.use(express.json());

// Health check route
app.get("/", (_req, res) => {
  res.send("Auth server running");
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
});

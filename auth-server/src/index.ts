import express from "express";
import { betterAuth } from "better-auth";
import { toNodeHandler } from "better-auth/node";

const app = express();

// Create auth instance – note the lowercase 'u' in baseUrl
const auth = betterAuth({
  secret: process.env.AUTH_SECRET!,
  baseUrl: process.env.AUTH_BASE_URL!, // ← fixed: baseUrl (not baseURL)
  // Add your database, emailAndPassword, socialProviders, etc. here if needed
});

// Recommended mounting: use app.use with exact base path
// This avoids wildcard issues in Express v4 and matches official examples
app.use("/api/auth", toNodeHandler(auth));

// Place express.json() AFTER the auth handler (critical!)
app.use(express.json());

// Health check route
app.get("/", (_req, res) => {
  res.send("Auth server running");
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
});

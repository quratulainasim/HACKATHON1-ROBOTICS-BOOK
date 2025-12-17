import express from "express";
import { betterAuth } from "better-auth";

const app = express();
app.use(express.json());

// ✅ create auth instance
const auth = betterAuth({
  secret: process.env.AUTH_SECRET!,
  baseURL: process.env.AUTH_BASE_URL!,
});

// ✅ mount routes CORRECTLY
app.post("/auth/*", async (req, res) => {
  return auth.handler(req, res);
});

app.get("/", (_req, res) => {
  res.send("Auth server running");
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
});

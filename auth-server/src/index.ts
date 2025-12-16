import express from "express";
import cors from "cors";
import { betterAuth } from "better-auth"; // <-- use betterAuth, not BetterAuth

const app = express();
app.use(cors());
app.use(express.json());

const auth = betterAuth({
  apiKey: process.env.BETTER_AUTH_API_KEY!, // Railway secret
  databaseUrl: process.env.DATABASE_URL!    // PostgreSQL URL from Railway
});

// Example route
app.get("/", async (req, res) => {
  res.send("Server is running!");
});

// Example: create user route
app.post("/signup", async (req, res) => {
  const { email, password } = req.body;
  try {
    const user = await auth.createUser({ email, password });
    res.json(user);
  } catch (err) {
    res.status(400).json({ error: err.message });
  }
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
});

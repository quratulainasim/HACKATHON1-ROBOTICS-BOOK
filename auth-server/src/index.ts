import express from "express";
import BetterAuth from "better-auth";
import dotenv from "dotenv";

dotenv.config();

const app = express();
app.use(express.json());

const auth = new BetterAuth({
  apiKey: process.env.BETTER_AUTH_API_KEY,
});

// Signup endpoint
app.post("/signup", async (req, res) => {
  try {
    const { email, password } = req.body;
    const user = await auth.signup({ email, password });
    res.json({ success: true, user });
  } catch (err: any) {
    res.status(400).json({ success: false, error: err.message });
  }
});

// Login endpoint
app.post("/login", async (req, res) => {
  try {
    const { email, password } = req.body;
    const token = await auth.login({ email, password });
    res.json({ success: true, token });
  } catch (err: any) {
    res.status(400).json({ success: false, error: err.message });
  }
});

// Start server
const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
});

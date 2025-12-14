import express from "express";
import cors from "cors";
import dotenv from "dotenv";
import authRoutes from './auth-routes';

dotenv.config();

const app = express();
const PORT = process.env.PORT || "3001";

// CORS — allow your React app
app.use(
  cors({
    origin: ["http://localhost:3000", "http://127.0.0.1:3000"],
    credentials: true,
  })
);

// Parse JSON
app.use(express.json());

// Mount auth routes at /api/auth
app.use("/api/auth", authRoutes);

// Optional: health check
app.get("/health", (_, res) => res.json({ status: "OK" }));

app.listen(PORT, () => {
  console.log(`Auth server running on http://localhost:${PORT}`);
  console.log(`Sign-up endpoint → http://localhost:${PORT}/api/auth/sign-up/email`);
  console.log(`Sign-in endpoint → http://localhost:${PORT}/api/auth/sign-in/email`);
});
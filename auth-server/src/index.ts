import express from "express";
import cors from "cors";
import { betterAuth } from "better-auth";
import { Pool } from "pg";

const app = express();

app.use(cors({
  origin: process.env.FRONTEND_URL,
  credentials: true
}));

app.use(express.json());

/* PostgreSQL (Neon / Railway) */
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: { rejectUnauthorized: false }
});

/* Better Auth instance */
const auth = betterAuth({
  database: pool,
  secret: process.env.AUTH_SECRET!,
  baseUrl: process.env.AUTH_BASE_URL!,
});

/* Better Auth routes */
app.use("/auth", auth.handler);

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
  console.log("Auth server running on port", PORT);
});

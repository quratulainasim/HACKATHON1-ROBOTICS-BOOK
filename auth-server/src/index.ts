import express from "express";
import cors from "cors";
import { betterAuth } from "better-auth";
import { postgresAdapter } from "better-auth/adapters/postgres";

const app = express();

app.use(cors({
  origin: process.env.FRONTEND_URL,
  credentials: true
}));

app.use(express.json());

/* Better Auth setup */
const auth = betterAuth({
  secret: process.env.AUTH_SECRET!,
  baseUrl: process.env.AUTH_BASE_URL!,
  database: postgresAdapter({
    connectionString: process.env.DATABASE_URL!,
    ssl: { rejectUnauthorized: false }
  })
});

/* Mount Better Auth routes */
app.use("/auth", auth.router);

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
  console.log("✅ Auth server running on port", PORT);
});

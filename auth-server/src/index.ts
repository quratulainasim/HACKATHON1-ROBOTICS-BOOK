import express from "express";
import { betterAuth } from "better-auth";

const app = express();
app.use(express.json());

const auth = betterAuth({
  secret: process.env.AUTH_SECRET!, // ✅ uses AUTH_SECRET only
  baseURL: process.env.AUTH_BASE_URL!,
  database: {
    url: process.env.DATABASE_URL!,
  },
});

app.use("/auth", auth.handler);

app.get("/", (_req, res) => {
  res.send("Auth server running");
});

const PORT = Number(process.env.PORT) || 3000;
app.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
});
